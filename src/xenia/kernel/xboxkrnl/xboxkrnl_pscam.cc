/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

// PsCamDeviceRequest, answered with a Kinect v1 camera on the host (see
// hid/kinect/usb_camera.h). Calibration math is from libfreenect
// (src/registration.c, Apache-2.0).

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>

#include "third_party/fmt/include/fmt/format.h"
#include "xenia/base/cvar.h"
#include "xenia/base/filesystem.h"
#include "xenia/base/logging.h"
#include "xenia/base/memory.h"
#include "xenia/base/mutex.h"
#include "xenia/base/threading.h"
#include "xenia/cpu/processor.h"
#include "xenia/hid/kinect/usb_audio.h"
#include "xenia/hid/kinect/usb_camera.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/nui.h"
#include "xenia/kernel/util/shim_utils.h"
#include "xenia/kernel/xboxkrnl/xboxkrnl_private.h"
#include "xenia/kernel/xthread.h"
#include "xenia/memory.h"
#include "xenia/vfs/devices/xcontent_container_device.h"
#include "xenia/vfs/entry.h"
#include "xenia/vfs/file.h"
#include "xenia/xbox.h"

DEFINE_bool(allow_kinect_camera, false,
            "Let titles use a connected Kinect for Xbox 360 (v1) camera.",
            "HID");

DEFINE_bool(kinect_use_usbdk, true,
            "Access a Kinect v1 camera through UsbDk, which borrows it from "
            "its Windows driver while the emulator runs. When disabled, the "
            "camera must use the WinUSB driver. Windows only; Linux uses "
            "usbfs and detaches the kernel driver itself.",
            "HID");

DEFINE_bool(kinect_mirror, false,
            "Mirror the Kinect depth and color images horizontally before "
            "titles get them.",
            "HID");

DEFINE_path(kinect_replay_path, "",
            "Play the Kinect recording in this directory (made with "
            "xenia-kinect-camera-test --record_dir) as the camera instead of "
            "a connected Kinect.",
            "HID");

DEFINE_path(kinect_audio_firmware_path, "",
            "Microsoft's Kinect audio firmware to upload to the sensor when "
            "the camera opens: audios.bin, or the Xbox 360 system update "
            "package holding it ($SystemUpdate/FFFE07DF00000001). Model "
            "1473's motor, LED and accelerometer need it.",
            "HID");

DEFINE_path(kinect_record_path, "",
            "Record every frame handed to the title into this directory, in "
            "the format kinect_replay_path plays back.",
            "HID");

DEFINE_path(kinect_dump_depth_path, "",
            "Write the latest depth frame handed to the title to this file "
            "every two seconds, as a 16-bit PGM of raw 11-bit disparities, "
            "for checking what the title's tracker sees.",
            "HID");

namespace xe {
namespace kernel {
namespace xboxkrnl {

namespace {

enum class CameraRequestCode : uint32_t {
  kGetState = 0,
  kStop = 2,
  kReadFrame = 5,
  kVersion = 13,
  kRegistration = 14,
  kTimeSync = 15,
  kDepthTable = 24,
  kTiming = 25,
  kConfigure = 27,
  kUnknown33 = 33,
  kSetControls = 35,
  kGetControls = 36,
  // Replies at +0x04 with the depth calibration's scale and offset
  // (XamNuiGetLoadedDepthCalibration, 0x817FCB40).
  kLoadedDepthCalibration = 44,
};

enum class CameraStream : uint32_t {
  kColor = 2,
  kDepth = 4,
};

// Device states reported through code 0.
constexpr uint32_t kDeviceStateAbsent = 0;
constexpr uint32_t kDeviceStateStreaming = 2;
constexpr uint32_t kDeviceStateReady = 4;
constexpr uint32_t kNoCompletionRoutine = 0xFFFFFFFF;
constexpr uint32_t kFrameWidth = 640;
constexpr uint32_t kFrameHeight = 480;
constexpr uint32_t kDepthFrameBytes = kFrameWidth * kFrameHeight * 2;
constexpr uint32_t kColorFrameBytes = kFrameWidth * kFrameHeight;
constexpr uint32_t kDepthTableEntries = 8192;
constexpr uint16_t kNoDepthRaw = 2047;
// Frame period in microseconds, returned for both streams by code 25.
constexpr uint32_t kFramePeriodUs = 33333;
// Host-side choices, not console values.
constexpr uint32_t kWorkerStackSize = 128 * 1024;
constexpr auto kWorkerWaitTimeout = std::chrono::milliseconds(100);
// Reported through code 13; 5.1 or later selects the registration request.
constexpr uint8_t kDriverVersionMajor = 5;
constexpr uint8_t kDriverVersionMinor = 1;

// libfreenect registration constants (src/registration.c).
constexpr uint16_t kShiftToDepthPixelConst = 10;
constexpr double kShiftToDepthConstOffset = 0.375;
constexpr double kParameterCoefficient = 4.0;
constexpr double kShiftScale = 10.0;
constexpr double kMaxDepthMillimetres = 10000.0;

// Request header. Codes 14 and 25 reuse +0x08 onwards for their replies.
struct X_PSCAM_REQUEST {
  xe::be<uint32_t> code;
  xe::be<uint32_t> unknown_04[3];
  xe::be<uint32_t> context;
  // Called with (request, status, bytes), 0 or kNoCompletionRoutine for none.
  xe::be<uint32_t> completion_routine;
};
static_assert_size(X_PSCAM_REQUEST, 0x18);

struct X_PSCAM_STATE_REQUEST {
  xe::be<uint32_t> code;
  xe::be<uint32_t> state;
};
static_assert_size(X_PSCAM_STATE_REQUEST, 0x8);

struct X_PSCAM_READ_FRAME_REQUEST {
  X_PSCAM_REQUEST header;
  xe::be<uint32_t> stream;
  xe::be<uint32_t> buffer_ptr;
  xe::be<uint32_t> buffer_size;
  xe::be<uint32_t> unknown_24;
  xe::be<uint32_t> frame_counter_ptr;
  // Color frames only.
  uint8_t settings_generation;
  uint8_t unknown_2D[3];
};
static_assert_size(X_PSCAM_READ_FRAME_REQUEST, 0x30);

// Color camera settings, as the NUI runtime of 4D53090F keeps and sends them
// (its property setters at 0x82F51FA0 and 0x82F527E8, defaults at
// 0x82F50D98). It keeps two sets: one while playing, one while Kinect ID
// captures faces (0x82F5E990). Fractional values are 16.16 fixed point.
struct X_PSCAM_CAMERA_SETTINGS {
  // kSettingsPlay or kSettingsIdentity, and in the low bits which values the
  // title set itself (kSettingsHas*); without them it asks the camera for the
  // current ones (code 36).
  xe::be<uint32_t> flags;
  // 1 to 3: 1 normal color, 2 high quality color (the title's NUI
  // initialization flag 0x40, at 15 frames per second).
  xe::be<uint32_t> color_mode;
  // Region the exposure is taken from, off by default: on, and its center
  // from the image's in pixels (-320 to 320, -240 to 240).
  xe::be<uint32_t> region_enabled;
  xe::be<int32_t> region_x;
  xe::be<int32_t> region_y;
  // Exposure compensation in stops, -1.75 to 1.75.
  xe::be<int32_t> exposure_compensation;
  // 1 automatic exposure (the default), 0 manual. Titles also run their own
  // automatic exposure, setting the time each frame.
  xe::be<uint32_t> auto_exposure;
  // 0 to 3, up to how many frame periods automatic exposure may take; 2 by
  // default for high quality color.
  xe::be<uint32_t> auto_exposure_frames;
  // Metering window: left, top, width and height, as fractions of the image
  // (0.1, 0.1, 0.8, 0.8 by default).
  xe::be<int32_t> metering_window[4];
  // Red, green and blue gains, 1.0 by default.
  xe::be<int32_t> gains[3];
  // Manual exposure time in milliseconds, 0 to 2000.
  xe::be<int32_t> exposure_time;
  // Reported by the camera; 4D53090F reads it after two seconds of automatic
  // exposure, taking 60 when it's 0.
  xe::be<uint32_t> measured;
  // The console's display refresh rate in Hz.
  xe::be<int32_t> refresh_rate;
  // 0 to 7; 0 while playing, 6 (set as 101) while capturing faces. Its
  // meaning isn't known.
  xe::be<uint32_t> mode;
  uint8_t unknown_4C[0x10];
};
static_assert_size(X_PSCAM_CAMERA_SETTINGS, 0x5C);

constexpr uint32_t kSettingsPlay = 0x40000000;
constexpr uint32_t kSettingsIdentity = 0x80000000;
constexpr uint32_t kSettingsHasExposureTime = 0x1;
constexpr uint32_t kSettingsHasGains = 0xE;
constexpr int32_t kFixedOne = 0x10000;

struct X_PSCAM_CONTROLS_REQUEST {
  X_PSCAM_REQUEST header;
  xe::be<uint32_t> settings_generation;
  X_PSCAM_CAMERA_SETTINGS settings;
};
static_assert_size(X_PSCAM_CONTROLS_REQUEST, 0x78);

struct X_PSCAM_VERSION_REQUEST {
  xe::be<uint32_t> code;
  uint8_t major;
  uint8_t minor;
  uint8_t unknown_06[2];
};
static_assert_size(X_PSCAM_VERSION_REQUEST, 0x8);

struct X_PSCAM_REGISTRATION_REQUEST {
  xe::be<uint32_t> code;
  xe::be<uint32_t> unknown_04;
  xe::be<uint32_t> registration[29];
  uint8_t unknown_7C[0x100C];
  // Depth-to-color shift parameters, used like libfreenect's
  // freenect_init_depth_to_rgb.
  xe::be<double> reference_distance;
  xe::be<double> reference_pixel_size;
  // The title divides by this; libfreenect's equivalent factor is 1.
  xe::be<uint16_t> frame_width;
  xe::be<uint16_t> shift_to_depth_pixel_const;
  uint8_t unknown_109C[4];
  xe::be<double> shift_to_depth_const_offset;
  xe::be<double> dcmos_rcmos_distance;
};
static_assert_size(X_PSCAM_REGISTRATION_REQUEST, 0x10B0);

struct X_PSCAM_DEPTH_TABLE_REQUEST {
  xe::be<uint32_t> code;
  xe::be<uint32_t> unknown_04;
  xe::be<uint32_t> table_ptr;
  xe::be<uint32_t> entry_count;
};
static_assert_size(X_PSCAM_DEPTH_TABLE_REQUEST, 0x10);

struct X_PSCAM_TIMING_REQUEST {
  xe::be<uint32_t> code;
  // Microseconds.
  xe::be<uint32_t> depth_frame_period;
  xe::be<uint32_t> unknown_08[4];
  // Microseconds.
  xe::be<uint32_t> color_frame_period;
  xe::be<uint32_t> unknown_1C[2];
  // 0 disables time sync (code 15).
  xe::be<uint32_t> time_sync;
  xe::be<uint32_t> unknown_28[5];
};
static_assert_size(X_PSCAM_TIMING_REQUEST, 0x3C);

struct PendingRequest {
  uint32_t request_ptr = 0;
  // Read at request time, the title may overwrite the field afterwards. 0 for
  // none.
  uint32_t completion = 0;
  X_STATUS status = X_STATUS_SUCCESS;
  uint32_t bytes = 0;
  // Camera settings request that sets this generation once completed.
  bool new_settings = false;
  uint8_t generation = 0;
  // Frame reads that were queued when streaming was stopped (code 2), completed
  // as cancelled before this request.
  std::vector<PendingRequest> cancelled_reads;
};

// Reads the firmware from audios.bin itself or from the system update package
// it ships in.
std::vector<uint8_t> ReadAudioFirmware(const std::filesystem::path& path) {
  std::vector<uint8_t> image;
  auto header = vfs::XContentContainerDevice::ReadContainerHeader(path);
  if (header && header->content_header.is_magic_valid()) {
    auto device = vfs::XContentContainerDevice::CreateContentDevice("", path);
    if (!device || !device->Initialize()) {
      return image;
    }
    // Paths resolve through Device, as the file system does it.
    vfs::Entry* entry =
        static_cast<vfs::Device&>(*device).ResolvePath("audios.bin");
    vfs::File* file = nullptr;
    if (!entry ||
        entry->Open(xe::filesystem::FileAccess::kFileReadData, &file) !=
            X_STATUS_SUCCESS) {
      return image;
    }
    image.resize(entry->size());
    size_t bytes_read = 0;
    if (file->ReadSync(image, 0, &bytes_read) != X_STATUS_SUCCESS ||
        bytes_read != image.size()) {
      image.clear();
    }
    file->Destroy();
    return image;
  }
  if (FILE* file = xe::filesystem::OpenFile(path, "rb")) {
    uint8_t buffer[0x4000];
    size_t read;
    while ((read = std::fread(buffer, 1, sizeof(buffer), file)) > 0) {
      image.insert(image.end(), buffer, buffer + read);
    }
    std::fclose(file);
  }
  return image;
}

class CameraDevice {
 public:
  explicit CameraDevice(KernelState* kernel_state)
      : kernel_state_(kernel_state), memory_(kernel_state->memory()) {
    if (!cvars::kinect_replay_path.empty()) {
      camera_ = hid::kinect::UsbCamera::OpenRecording(
          cvars::kinect_replay_path, [this]() { Notify(); });
    } else if (cvars::allow_kinect_camera) {
      if (!cvars::kinect_audio_firmware_path.empty()) {
        const std::vector<uint8_t> image =
            ReadAudioFirmware(cvars::kinect_audio_firmware_path);
        if (image.empty()) {
          XELOGE("PsCamDeviceRequest: cannot read the Kinect audio firmware "
                 "from {}",
                 xe::strip_pii(
                     xe::path_to_utf8(cvars::kinect_audio_firmware_path)));
        } else {
          XELOGI("PsCamDeviceRequest: Kinect audio firmware is {} bytes",
                 image.size());
          hid::kinect::LoadAudioFirmware(image);
        }
      }
      kernel_state->nui()->SetMotor(hid::kinect::UsbAudio::Open());
      camera_ = hid::kinect::UsbCamera::Open(cvars::kinect_use_usbdk,
                                             [this]() { Notify(); });
    }
    if (!camera_) {
      XELOGI("PsCamDeviceRequest: no Kinect camera found");
      return;
    }
    worker_thread_ = object_ref<XHostThread>(new XHostThread(
        kernel_state, kWorkerStackSize, 0,
        [this]() {
          WorkerMain();
          return 0;
        },
        kernel_state->GetSystemProcess()));
    worker_thread_->set_can_debugger_suspend(true);
    worker_thread_->set_name("Camera Worker");
    worker_thread_->Create();
  }

  ~CameraDevice() {
    worker_running_ = false;
    Notify();
    if (worker_thread_) {
      worker_thread_->Wait(0, 0, 0, nullptr);
      worker_thread_.reset();
    }
    // Closing the camera stops its streams and returns it to its driver.
    camera_.reset();
  }

  X_STATUS Request(uint32_t request_ptr) {
    if (!IsGuestRange(request_ptr, sizeof(X_PSCAM_REQUEST))) {
      return X_STATUS_INVALID_PARAMETER;
    }
    auto request = Guest<X_PSCAM_REQUEST>(request_ptr);
    const auto code = static_cast<CameraRequestCode>(uint32_t(request->code));
    if (!IsGuestRange(request_ptr, GetRequestSize(code))) {
      return X_STATUS_INVALID_PARAMETER;
    }
    uint32_t completion = request->completion_routine;
    if (completion == kNoCompletionRoutine) {
      completion = 0;
    }
    LogFirstRequest(code, request_ptr);

    if (!camera_) {
      if (code == CameraRequestCode::kGetState) {
        uint32_t bytes = 0;
        return Handle(code, request_ptr, bytes);
      }
      // TODO(knuckleslee): Find the status the console returns without a
      // camera.
      return X_STATUS_DEVICE_NOT_CONNECTED;
    }

    if (code == CameraRequestCode::kReadFrame) {
      auto read = Guest<X_PSCAM_READ_FRAME_REQUEST>(request_ptr);
      const auto stream = static_cast<CameraStream>(uint32_t(read->stream));
      const uint32_t counter_ptr = read->frame_counter_ptr;
      if (!completion ||
          (stream != CameraStream::kDepth && stream != CameraStream::kColor) ||
          (counter_ptr && !IsGuestRange(counter_ptr, sizeof(uint32_t)))) {
        return X_STATUS_INVALID_PARAMETER;
      }
      PendingRequest pending;
      pending.request_ptr = request_ptr;
      pending.completion = completion;
      {
        std::lock_guard<xe_mutex> lock(mutex_);
        streaming_ = true;
        (stream == CameraStream::kDepth ? depth_reads_ : color_reads_)
            .push_back(pending);
      }
      Notify();
      return X_STATUS_PENDING;
    }

    uint32_t bytes = 0;
    PendingRequest pending;
    {
      // Titles send camera settings from more than one thread.
      std::lock_guard<xe_mutex> lock(settings_mutex_);
      pending.status = Handle(code, request_ptr, bytes);
      pending.generation = requested_generation_;
    }
    const X_STATUS status = pending.status;
    pending.request_ptr = request_ptr;
    pending.completion = completion;
    pending.bytes = bytes;
    pending.new_settings =
        code == CameraRequestCode::kSetControls && XSUCCEEDED(status);
    {
      std::lock_guard<xe_mutex> lock(mutex_);
      if (code == CameraRequestCode::kStop) {
        // TODO(knuckleslee): Find how the console completes queued reads on a
        // stop. They are completed with X_STATUS_CANCELLED here.
        pending.cancelled_reads.assign(depth_reads_.begin(),
                                       depth_reads_.end());
        pending.cancelled_reads.insert(pending.cancelled_reads.end(),
                                       color_reads_.begin(),
                                       color_reads_.end());
        depth_reads_.clear();
        color_reads_.clear();
        streaming_ = false;
        ++stop_count_;
      }
      if (!completion && pending.cancelled_reads.empty()) {
        return status;
      }
      completions_.push_back(std::move(pending));
    }
    Notify();
    return completion ? X_STATUS_PENDING : status;
  }

  // For the Kinect Tuner: the newest frames, if newer than the sequence
  // numbers given, which are updated. False without a camera.
  bool CopyPreview(uint32_t& depth_sequence, std::vector<uint8_t>& packed_depth,
                   uint32_t& color_sequence, std::vector<uint8_t>& bayer) {
    if (!camera_) {
      return false;
    }
    if (uint32_t sequence = camera_->CopyDepthFrame(depth_sequence,
                                                    packed_depth)) {
      depth_sequence = sequence;
    }
    if (uint32_t sequence = camera_->CopyColorFrame(color_sequence, bayer)) {
      color_sequence = sequence;
    }
    return true;
  }

 private:
  template <typename T>
  T* Guest(uint32_t address) const {
    return memory_->TranslateVirtual<T*>(address);
  }

  // Logs the first request of each code with its leading words, to see what a
  // title's Kinect runtime asks for.
  void LogFirstRequest(CameraRequestCode code, uint32_t request_ptr) {
    const uint32_t code_value = static_cast<uint32_t>(code);
    {
      std::lock_guard<xe_mutex> lock(mutex_);
      if (code_value < 64 && (logged_codes_ & (uint64_t(1) << code_value))) {
        return;
      }
      if (code_value < 64) {
        logged_codes_ |= uint64_t(1) << code_value;
      }
    }
    auto words = Guest<xe::be<uint32_t>>(request_ptr);
    XELOGI(
        "PsCamDeviceRequest: first code {} at {:08X}: {:08X} {:08X} {:08X} "
        "{:08X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X}",
        code_value, request_ptr, uint32_t(words[0]), uint32_t(words[1]),
        uint32_t(words[2]), uint32_t(words[3]), uint32_t(words[4]),
        uint32_t(words[5]), uint32_t(words[6]), uint32_t(words[7]),
        uint32_t(words[8]), uint32_t(words[9]), uint32_t(words[10]),
        uint32_t(words[11]));
  }

  // Checks that the first and last byte of a guest range are mapped.
  // TODO(knuckleslee): Find whether the console validates request ranges.
  // X_STATUS_INVALID_PARAMETER is returned for bad ones here.
  bool IsGuestRange(uint32_t address, size_t size) const {
    if (!address || !size || size > UINT32_MAX) {
      return false;
    }
    const uint32_t last_address = address + static_cast<uint32_t>(size - 1);
    return last_address >= address && memory_->LookupHeap(address) &&
           memory_->LookupHeap(last_address);
  }

  // The settings titles start from, before any code 35.
  static X_PSCAM_CAMERA_SETTINGS DefaultSettings(uint32_t set) {
    X_PSCAM_CAMERA_SETTINGS settings = {};
    settings.flags = set;
    settings.color_mode = 1;
    settings.auto_exposure = 1;
    settings.metering_window[0] = kFixedOne / 10;
    settings.metering_window[1] = kFixedOne / 10;
    settings.metering_window[2] = kFixedOne * 4 / 5;
    settings.metering_window[3] = kFixedOne * 4 / 5;
    for (auto& gain : settings.gains) {
      gain = kFixedOne;
    }
    settings.refresh_rate = 60 * kFixedOne;
    return settings;
  }

  static size_t GetRequestSize(CameraRequestCode code) {
    size_t size = 0;
    switch (code) {
      case CameraRequestCode::kGetState:
        size = sizeof(X_PSCAM_STATE_REQUEST);
        break;
      case CameraRequestCode::kReadFrame:
        size = sizeof(X_PSCAM_READ_FRAME_REQUEST);
        break;
      case CameraRequestCode::kVersion:
        size = sizeof(X_PSCAM_VERSION_REQUEST);
        break;
      case CameraRequestCode::kRegistration:
        size = sizeof(X_PSCAM_REGISTRATION_REQUEST);
        break;
      case CameraRequestCode::kDepthTable:
        size = sizeof(X_PSCAM_DEPTH_TABLE_REQUEST);
        break;
      case CameraRequestCode::kTiming:
        size = sizeof(X_PSCAM_TIMING_REQUEST);
        break;
      case CameraRequestCode::kSetControls:
      case CameraRequestCode::kGetControls:
        size = sizeof(X_PSCAM_CONTROLS_REQUEST);
        break;
      default:
        break;
    }
    return std::max(size, sizeof(X_PSCAM_REQUEST));
  }

  // Wakes the worker for a new request or camera frame.
  void Notify() { work_event_->Set(); }

  X_STATUS Handle(CameraRequestCode code, uint32_t request_ptr,
                  uint32_t& bytes) {
    switch (code) {
      case CameraRequestCode::kGetState:
        Guest<X_PSCAM_STATE_REQUEST>(request_ptr)->state =
            !camera_     ? kDeviceStateAbsent
            : streaming_ ? kDeviceStateStreaming
                         : kDeviceStateReady;
        return X_STATUS_SUCCESS;
      case CameraRequestCode::kSetControls: {
        // Kept for code 36. Titles leave exposure and white balance to the
        // sensor's automatic ones, so they aren't sent to it.
        auto request = Guest<X_PSCAM_CONTROLS_REQUEST>(request_ptr);
        active_settings_ = request->settings.flags & kSettingsIdentity ? 1 : 0;
        settings_[active_settings_] = request->settings;
        // Color frames carry the generation once this request has completed
        // (see Complete).
        request->settings_generation = ++requested_generation_;
        return X_STATUS_SUCCESS;
      }
      case CameraRequestCode::kGetControls: {
        // The current settings of the set last sent. The values the title
        // didn't set report the defaults: no manual exposure is running, and
        // the white balance gains are neutral.
        X_PSCAM_CAMERA_SETTINGS& settings =
            Guest<X_PSCAM_CONTROLS_REQUEST>(request_ptr)->settings;
        settings = settings_[active_settings_];
        if (!(settings.flags & kSettingsHasGains)) {
          for (auto& gain : settings.gains) {
            gain = kFixedOne;
          }
        }
        settings.measured = 0;
        return X_STATUS_SUCCESS;
      }
      case CameraRequestCode::kStop:
      case CameraRequestCode::kConfigure:
      case CameraRequestCode::kUnknown33:
      case CameraRequestCode::kTimeSync:
        return X_STATUS_SUCCESS;
      case CameraRequestCode::kVersion: {
        auto version = Guest<X_PSCAM_VERSION_REQUEST>(request_ptr);
        version->major = kDriverVersionMajor;
        version->minor = kDriverVersionMinor;
        return X_STATUS_SUCCESS;
      }
      case CameraRequestCode::kRegistration:
        WriteRegistration(request_ptr);
        return X_STATUS_SUCCESS;
      case CameraRequestCode::kLoadedDepthCalibration: {
        const auto words = kernel_state_->nui()->GetLoadedDepthCalibration();
        auto request = Guest<X_PSCAM_REQUEST>(request_ptr);
        request->unknown_04[0] = words[0];
        request->unknown_04[1] = words[1];
        return X_STATUS_SUCCESS;
      }
      case CameraRequestCode::kDepthTable:
        return WriteDepthTable(request_ptr);
      case CameraRequestCode::kTiming: {
        auto timing = Guest<X_PSCAM_TIMING_REQUEST>(request_ptr);
        timing->depth_frame_period = kFramePeriodUs;
        std::memset(timing->unknown_08, 0, sizeof(timing->unknown_08));
        timing->color_frame_period = kFramePeriodUs;
        std::memset(timing->unknown_1C, 0, sizeof(timing->unknown_1C));
        timing->time_sync = 0;
        std::memset(timing->unknown_28, 0, sizeof(timing->unknown_28));
        return X_STATUS_SUCCESS;
      }
      default:
        XELOGE("PsCamDeviceRequest: unsupported code {}",
               static_cast<uint32_t>(code));
        return X_STATUS_NOT_SUPPORTED;
    }
  }

  void WriteRegistration(uint32_t request_ptr) {
    const hid::kinect::UsbCameraCalibration& calibration =
        camera_->calibration();
    auto request = Guest<X_PSCAM_REGISTRATION_REQUEST>(request_ptr);
    static_assert(sizeof(request->registration) ==
                  sizeof(calibration.registration));
    for (size_t i = 0; i < xe::countof(request->registration); ++i) {
      request->registration[i] =
          xe::load<uint32_t>(calibration.registration.data() + i * 4);
    }
    request->reference_distance = calibration.reference_distance;
    request->reference_pixel_size = calibration.reference_pixel_size;
    request->frame_width = static_cast<uint16_t>(kFrameWidth);
    request->shift_to_depth_pixel_const = kShiftToDepthPixelConst;
    request->shift_to_depth_const_offset = kShiftToDepthConstOffset;
    request->dcmos_rcmos_distance = calibration.dcmos_rcmos_distance;
  }

  X_STATUS WriteDepthTable(uint32_t request_ptr) {
    auto request = Guest<X_PSCAM_DEPTH_TABLE_REQUEST>(request_ptr);
    const uint32_t table_ptr = request->table_ptr;
    const uint32_t entries = request->entry_count;
    if (entries != kDepthTableEntries ||
        !IsGuestRange(table_ptr, entries * sizeof(uint16_t))) {
      return X_STATUS_INVALID_PARAMETER;
    }
    const hid::kinect::UsbCameraCalibration& calibration =
        camera_->calibration();
    auto table = Guest<xe::be<uint16_t>>(table_ptr);
    for (uint32_t raw = 0; raw < entries; ++raw) {
      uint16_t millimetres = 0;
      // Raw 0 is used for pixels without depth, see UnpackDepth.
      if (raw != 0 && raw < kNoDepthRaw) {
        // libfreenect freenect_raw_to_mm.
        const double fixed_ref_x =
            (raw - kParameterCoefficient * calibration.const_shift) /
                kParameterCoefficient -
            kShiftToDepthConstOffset;
        const double metric = fixed_ref_x * calibration.reference_pixel_size;
        const double value =
            kShiftScale * (metric * calibration.reference_distance /
                               (calibration.dcmos_emitter_distance - metric) +
                           calibration.reference_distance);
        if (value > 0 && value < kMaxDepthMillimetres) {
          millimetres = static_cast<uint16_t>(value);
        }
      }
      table[raw] = millimetres;
    }
    return X_STATUS_SUCCESS;
  }

  // Unpacks big-endian 11-bit samples into big-endian 16-bit words.
  static void UnpackDepth(const std::vector<uint8_t>& packed, uint8_t* out,
                          size_t pixels) {
    const bool mirror = cvars::kinect_mirror;
    uint32_t bits = 0;
    int bit_count = 0;
    size_t in = 0;
    for (size_t i = 0; i < pixels; ++i) {
      while (bit_count < 11) {
        bits = (bits << 8) | packed[in++];
        bit_count += 8;
      }
      bit_count -= 11;
      uint16_t raw = static_cast<uint16_t>((bits >> bit_count) & 0x7FF);
      // Pixels without depth are passed as 0 rather than 2047.
      // TODO(knuckleslee): Verify what the console's camera driver returns.
      if (raw == kNoDepthRaw) {
        raw = 0;
      }
      size_t index = i;
      if (mirror) {
        const size_t x = i % kFrameWidth;
        index = i - x + (kFrameWidth - 1 - x);
      }
      xe::store_and_swap<uint16_t>(out + index * 2, raw);
    }
  }

  bool CompleteFrame(const PendingRequest& read, bool depth) {
    const uint32_t frame_bytes = depth ? kDepthFrameBytes : kColorFrameBytes;
    uint32_t& last_sequence = depth ? depth_sequence_ : color_sequence_;
    const uint32_t sequence =
        depth ? camera_->CopyDepthFrame(last_sequence, frame_)
              : camera_->CopyColorFrame(last_sequence, frame_);
    if (!sequence) {
      return false;
    }
    last_sequence = sequence;

    PendingRequest done = read;
    auto request = Guest<X_PSCAM_READ_FRAME_REQUEST>(read.request_ptr);
    const uint32_t buffer = request->buffer_ptr;
    if (request->buffer_size < frame_bytes ||
        !IsGuestRange(buffer, frame_bytes)) {
      done.status = X_STATUS_INVALID_PARAMETER;
      Complete(done);
      return true;
    }
    if (depth) {
      Record();
      UnpackDepth(frame_, Guest<uint8_t>(buffer), frame_bytes / 2);
      DumpDepth(Guest<uint8_t>(buffer), frame_bytes);
    } else {
      if (!cvars::kinect_record_path.empty()) {
        record_color_ = frame_;
      }
      if (cvars::kinect_mirror) {
        // Pairs of pixels are reversed, keeping the Bayer pattern in place.
        uint8_t* dest = Guest<uint8_t>(buffer);
        for (uint32_t y = 0; y < kFrameHeight; ++y) {
          const uint8_t* src_row = frame_.data() + y * kFrameWidth;
          uint8_t* dest_row = dest + y * kFrameWidth;
          for (uint32_t x = 0; x < kFrameWidth; x += 2) {
            dest_row[kFrameWidth - 2 - x] = src_row[x];
            dest_row[kFrameWidth - 1 - x] = src_row[x + 1];
          }
        }
      } else {
        std::memcpy(Guest<uint8_t>(buffer), frame_.data(), frame_bytes);
      }
      request->settings_generation = settings_generation_;
    }
    const uint32_t counter_ptr = request->frame_counter_ptr;
    if (counter_ptr) {
      *Guest<xe::be<uint32_t>>(counter_ptr) = sequence;
    }
    done.bytes = frame_bytes;
    if (!(depth ? logged_depth_frame_ : logged_color_frame_)) {
      (depth ? logged_depth_frame_ : logged_color_frame_) = true;
      XELOGI(
          "PsCamDeviceRequest: first {} frame {} to {:08X} (size {:X}, "
          "request {:08X})",
          depth ? "depth" : "color", sequence, buffer,
          uint32_t(request->buffer_size), read.request_ptr);
    }
    Complete(done);
    return true;
  }

  // Writes the depth frame in frame_, with the latest color frame, to
  // kinect_record_path. Starts once a color frame has arrived, since playback
  // expects both for every frame.
  void Record() {
    if (cvars::kinect_record_path.empty() || record_color_.empty() ||
        record_failed_) {
      return;
    }
    const std::filesystem::path& directory = cvars::kinect_record_path;
    if (!record_index_) {
      std::error_code error;
      std::filesystem::create_directories(directory, error);
      if (!hid::kinect::WriteRecordingCalibration(directory,
                                                  camera_->calibration())) {
        XELOGE("PsCamDeviceRequest: can't record to {}",
               xe::strip_pii(xe::path_to_utf8(directory)));
        record_failed_ = true;
        return;
      }
      XELOGI("PsCamDeviceRequest: recording to {}",
             xe::strip_pii(xe::path_to_utf8(directory)));
    }
    if (!hid::kinect::WriteRecordingFrame(directory, record_index_, frame_,
                                          record_color_)) {
      record_failed_ = true;
      return;
    }
    ++record_index_;
  }

  // Writes the unpacked depth frame (big-endian 16-bit words, as the title
  // gets it) to kinect_dump_depth_path now and then.
  void DumpDepth(const uint8_t* unpacked, uint32_t bytes) {
    if (cvars::kinect_dump_depth_path.empty()) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (last_dump_ != std::chrono::steady_clock::time_point() &&
        now - last_dump_ < std::chrono::seconds(2)) {
      return;
    }
    last_dump_ = now;
    FILE* file = xe::filesystem::OpenFile(cvars::kinect_dump_depth_path, "wb");
    if (!file) {
      return;
    }
    // PGM 16-bit samples are big-endian, like the frame.
    const std::string header =
        fmt::format("P5\n{} {}\n{}\n", kFrameWidth, kFrameHeight, kNoDepthRaw);
    fwrite(header.data(), 1, header.size(), file);
    fwrite(unpacked, 1, bytes, file);
    fclose(file);
  }

  // Does not read the request, which the title may have reused.
  void Complete(const PendingRequest& request) {
    if (request.completion) {
      uint64_t args[] = {request.request_ptr,
                         static_cast<uint32_t>(request.status), request.bytes};
      kernel_state_->processor()->Execute(worker_thread_->thread_state(),
                                          request.completion, args,
                                          xe::countof(args));
    }
    if (request.new_settings) {
      settings_generation_ = request.generation;
    }
  }

  void WorkerMain() {
    while (worker_running_) {
      std::deque<PendingRequest> completions;
      PendingRequest depth_read;
      PendingRequest color_read;
      uint32_t stop_count;
      xe::threading::Wait(work_event_.get(), false, kWorkerWaitTimeout);
      {
        std::lock_guard<xe_mutex> lock(mutex_);
        completions.swap(completions_);
        // Taken out of the queues so that a stop can't cancel them meanwhile.
        if (!depth_reads_.empty()) {
          depth_read = depth_reads_.front();
          depth_reads_.pop_front();
        }
        if (!color_reads_.empty()) {
          color_read = color_reads_.front();
          color_reads_.pop_front();
        }
        stop_count = stop_count_;
      }
      for (PendingRequest& request : completions) {
        for (PendingRequest& read : request.cancelled_reads) {
          read.status = X_STATUS_CANCELLED;
          Complete(read);
        }
        if (request.completion) {
          Complete(request);
        }
      }
      for (bool depth : {true, false}) {
        PendingRequest& read = depth ? depth_read : color_read;
        if (!read.request_ptr || CompleteFrame(read, depth)) {
          continue;
        }
        // No new frame yet: requeue, unless streaming was stopped meanwhile.
        bool cancelled;
        {
          std::lock_guard<xe_mutex> lock(mutex_);
          cancelled = stop_count != stop_count_;
          if (!cancelled) {
            (depth ? depth_reads_ : color_reads_).push_front(read);
          }
        }
        if (cancelled) {
          read.status = X_STATUS_CANCELLED;
          Complete(read);
        }
      }
    }
  }

  KernelState* kernel_state_;
  Memory* memory_;
  xe_mutex mutex_;
  std::unique_ptr<xe::threading::Event> work_event_ =
      xe::threading::Event::CreateAutoResetEvent(false);
  std::deque<PendingRequest> completions_;
  // Incremented by every stop (code 2).
  uint32_t stop_count_ = 0;
  std::deque<PendingRequest> depth_reads_;
  std::deque<PendingRequest> color_reads_;
  std::unique_ptr<hid::kinect::UsbCamera> camera_;
  object_ref<XHostThread> worker_thread_;
  std::atomic<bool> worker_running_ = true;
  uint32_t depth_sequence_ = 0;
  uint32_t color_sequence_ = 0;
  std::atomic<bool> streaming_ = false;
  // Guards Handle: the settings below and the replies built from them.
  xe_mutex settings_mutex_;
  // Generation of the latest camera settings request, and of the latest one
  // whose completion has run, which color frames are tagged with.
  uint8_t requested_generation_ = 0;
  std::atomic<uint8_t> settings_generation_ = 0;
  // The play and Kinect ID settings, and which set was sent last.
  std::array<X_PSCAM_CAMERA_SETTINGS, 2> settings_ = {
      DefaultSettings(kSettingsPlay), DefaultSettings(kSettingsIdentity)};
  uint32_t active_settings_ = 0;
  std::vector<uint8_t> frame_;
  std::vector<uint8_t> record_color_;
  uint32_t record_index_ = 0;
  bool record_failed_ = false;
  std::chrono::steady_clock::time_point last_dump_;
  uint64_t logged_codes_ = 0;
  bool logged_depth_frame_ = false;
  bool logged_color_frame_ = false;
};

xe_mutex g_camera_device_mutex;
std::unique_ptr<CameraDevice> g_camera_device;

CameraDevice* GetCameraDevice() {
  std::lock_guard<xe_mutex> lock(g_camera_device_mutex);
  if (!g_camera_device) {
    g_camera_device = std::make_unique<CameraDevice>(kernel_state());
  }
  return g_camera_device.get();
}

}  // namespace

void ShutdownCameraDevice() {
  std::unique_ptr<CameraDevice> camera_device;
  {
    std::lock_guard<xe_mutex> lock(g_camera_device_mutex);
    camera_device = std::move(g_camera_device);
  }
  // Destroyed unlocked, as its worker may still be completing a request.
  camera_device.reset();
}

bool CopyKinectCameraPreview(uint32_t& depth_sequence,
                             std::vector<uint8_t>& packed_depth,
                             uint32_t& color_sequence,
                             std::vector<uint8_t>& bayer) {
  return GetCameraDevice()->CopyPreview(depth_sequence, packed_depth,
                                        color_sequence, bayer);
}

dword_result_t PsCamDeviceRequest_entry(dword_t request_ptr) {
  if (!request_ptr) {
    return X_STATUS_INVALID_PARAMETER;
  }
  return GetCameraDevice()->Request(request_ptr);
}
DECLARE_XBOXKRNL_EXPORT1(PsCamDeviceRequest, kNone, kSketchy);

}  // namespace xboxkrnl
}  // namespace kernel
}  // namespace xe

DECLARE_XBOXKRNL_EMPTY_REGISTER_EXPORTS(PsCam);

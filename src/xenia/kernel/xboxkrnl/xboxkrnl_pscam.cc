/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

// PsCamDeviceRequest (0x30D), the console's driver of the Kinect cameras.
// Titles' NUI runtimes talk to it directly: a request starts with a code, has
// its completion routine at +0x14, called with (request, status, bytes), and
// the reply in the request. Frame reads (code 5) complete when a frame comes;
// the others at once, through the same completion.
//
// The order 4D53090F and 4D5308C9 send them in, once the state (code 0) is
// ready: 27 start, 35 color settings, 24 depth table, 33, 13 version, 14
// registration, 25 frame timing, then reads, and 36 now and then.

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <deque>
#include <mutex>
#include <vector>

#include "xenia/base/cvar.h"
#include "xenia/base/logging.h"
#include "xenia/base/string.h"
#include "xenia/base/threading.h"
#include "xenia/cpu/processor.h"
#include "xenia/hid/kinect/usb_camera.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/kinect/kinect_device.h"
#include "xenia/kernel/nui.h"
#include "xenia/kernel/util/shim_utils.h"
#include "xenia/kernel/xboxkrnl/xboxkrnl_private.h"
#include "xenia/kernel/xthread.h"
#include "xenia/xbox.h"

DEFINE_path(kinect_record_path, "",
            "Record every frame the Kinect gives titles into this directory, "
            "in the format kinect_replay_path plays back.",
            "HID");

namespace xe {
namespace kernel {
namespace xboxkrnl {

namespace {

enum class PsCamCode : uint32_t {
  kGetState = 0,
  kStop = 2,
  kReadFrame = 5,
  kVersion = 13,
  kRegistration = 14,
  kTimeSync = 15,
  kDepthTable = 24,
  kTiming = 25,
  kStart = 27,
  // Sent after the depth table with +0x1C = 1; its meaning isn't known.
  kUnknown33 = 33,
  kSetColorSettings = 35,
  kGetColorSettings = 36,
  // The depth calibration the sensor runs with: two words from +0x04
  // (XamNuiGetLoadedDepthCalibration, 0x817FCB40).
  kLoadedDepthCalibration = 44,
};

// Code 0's reply.
enum PsCamState : uint32_t {
  kStateAbsent = 0,
  kStateStreaming = 2,
  kStateReady = 4,
};

// Code 5's streams.
enum PsCamStream : uint32_t {
  kStreamColor = 2,
  kStreamDepth = 4,
};

// Titles test for version 5 with a nonzero minor before using the
// registration reply (4D53090F at 0x82F4BE38).
constexpr uint8_t kDriverVersionMajor = 5;
constexpr uint8_t kDriverVersionMinor = 1;

constexpr uint32_t kNoCompletionRoutine = 0xFFFFFFFF;
constexpr uint32_t kFrameWidth = 640;
constexpr uint32_t kFrameHeight = 480;
constexpr uint32_t kFramePeriodMicroseconds = 33333;
// Depth comes unpacked, 640x480 big-endian 16-bit words: titles also take it
// packed (0x67200 bytes) and mark it so (4D53090F at 0x82F4C2B0).
constexpr uint32_t kDepthFrameBytes = kFrameWidth * kFrameHeight * 2;
constexpr uint32_t kColorFrameBytes = kFrameWidth * kFrameHeight;
// The 11-bit raw depth, and the value the sensor sends for no depth.
constexpr uint32_t kDepthTableEntries = 0x2000;
constexpr uint16_t kNoDepthRaw = 0x7FF;

// libfreenect's registration constants (src/registration.c).
constexpr uint16_t kShiftToDepthPixelConst = 10;
constexpr double kShiftToDepthConstOffset = 0.375;
constexpr double kParameterCoefficient = 4.0;
constexpr double kShiftScale = 10.0;
constexpr double kMaxDepthMillimetres = 10000.0;

struct X_PSCAM_REQUEST {
  xe::be<uint32_t> code;
  xe::be<uint32_t> unknown_04[3];
  xe::be<uint32_t> context;
  xe::be<uint32_t> completion_routine;
};
static_assert_size(X_PSCAM_REQUEST, 0x18);

struct X_PSCAM_STATE_REQUEST {
  xe::be<uint32_t> code;
  xe::be<uint32_t> state;
};
static_assert_size(X_PSCAM_STATE_REQUEST, 0x8);

struct X_PSCAM_READ_REQUEST {
  X_PSCAM_REQUEST header;
  xe::be<uint32_t> stream;
  xe::be<uint32_t> buffer_ptr;
  xe::be<uint32_t> buffer_size;
  xe::be<uint32_t> unknown_24;
  // Gets the frame's sequence number.
  xe::be<uint32_t> frame_counter_ptr;
  // Color frames: the generation of the settings they were taken with.
  uint8_t settings_generation;
  uint8_t unknown_2D[3];
};
static_assert_size(X_PSCAM_READ_REQUEST, 0x30);

// The color camera settings the title's NUI runtime keeps, in two sets: while
// playing, and while Kinect ID captures faces. 16.16 fixed point.
constexpr uint32_t kSettingsPlay = 0x40000000;
constexpr uint32_t kSettingsIdentity = 0x80000000;
// Which values the title set itself.
constexpr uint32_t kSettingsHasExposureTime = 0x1;
constexpr uint32_t kSettingsHasGains = 0xE;
constexpr int32_t kFixedOne = 0x10000;

struct X_PSCAM_COLOR_SETTINGS {
  xe::be<uint32_t> flags;
  // 1 normal, 2 high quality color (NUI initialization flag 0x40).
  xe::be<uint32_t> color_mode;
  // Region the exposure is taken from: on, center in pixels from the image's.
  xe::be<uint32_t> region_enabled;
  xe::be<int32_t> region_x;
  xe::be<int32_t> region_y;
  // Stops, -1.75 to 1.75.
  xe::be<int32_t> exposure_compensation;
  // 1 the sensor's automatic exposure, 0 manual, 2 the title's own.
  xe::be<uint32_t> auto_exposure;
  // 0 to 3 frame periods automatic exposure may take.
  xe::be<uint32_t> auto_exposure_frames;
  // Metering window: left, top, width, height as fractions of the image.
  xe::be<int32_t> metering_window[4];
  // Red, green and blue gains, 2^x with x from -6 to 6.
  xe::be<int32_t> gains[3];
  // Manual exposure in milliseconds, 0 to 2000.
  xe::be<int32_t> exposure_time;
  // What the camera measured (read only); 4D53090F reads it after two
  // seconds of automatic exposure, taking 60 for 0.
  xe::be<uint32_t> measured;
  // The display's refresh rate.
  xe::be<int32_t> refresh_rate;
  // 0 while playing, 101 (kept as 6) while capturing faces; not understood.
  xe::be<uint32_t> mode;
  uint8_t unknown_4C[0x10];
};
static_assert_size(X_PSCAM_COLOR_SETTINGS, 0x5C);

struct X_PSCAM_COLOR_SETTINGS_REQUEST {
  X_PSCAM_REQUEST header;
  // Filled in by the driver; color frames carry it once the settings apply.
  xe::be<uint32_t> settings_generation;
  X_PSCAM_COLOR_SETTINGS settings;
};
static_assert_size(X_PSCAM_COLOR_SETTINGS_REQUEST, 0x78);

struct X_PSCAM_VERSION_REQUEST {
  xe::be<uint32_t> code;
  uint8_t major;
  uint8_t minor;
  uint8_t unknown_06[2];
};
static_assert_size(X_PSCAM_VERSION_REQUEST, 0x8);

// Color-depth registration. 4D53090F (0x82F4B840) reads the 29 words and the
// shift parameters at the end; the block between isn't read by any title.
struct X_PSCAM_REGISTRATION_REQUEST {
  xe::be<uint32_t> code;
  xe::be<uint32_t> unknown_04;
  xe::be<uint32_t> registration[29];
  uint8_t unknown_7C[0x100C];
  // As libfreenect's freenect_init_depth_to_rgb uses them.
  xe::be<double> reference_distance;
  xe::be<double> reference_pixel_size;
  xe::be<uint16_t> frame_width;
  xe::be<uint16_t> shift_to_depth_pixel_const;
  uint8_t unknown_109C[4];
  xe::be<double> shift_to_depth_const_offset;
  xe::be<double> dcmos_rcmos_distance;
};
static_assert_size(X_PSCAM_REGISTRATION_REQUEST, 0x10B0);

// The table from raw depth to millimeters; without it titles take raw = mm.
struct X_PSCAM_DEPTH_TABLE_REQUEST {
  xe::be<uint32_t> code;
  xe::be<uint32_t> unknown_04;
  xe::be<uint32_t> table_ptr;
  xe::be<uint32_t> entry_count;
};
static_assert_size(X_PSCAM_DEPTH_TABLE_REQUEST, 0x10);

struct X_PSCAM_TIMING_REQUEST {
  xe::be<uint32_t> code;
  xe::be<uint32_t> depth_frame_period;
  xe::be<uint32_t> unknown_08[4];
  xe::be<uint32_t> color_frame_period;
  xe::be<uint32_t> unknown_1C[2];
  // 0: no time sync (code 15).
  xe::be<uint32_t> time_sync;
  xe::be<uint32_t> unknown_28[5];
};
static_assert_size(X_PSCAM_TIMING_REQUEST, 0x3C);

X_PSCAM_COLOR_SETTINGS DefaultColorSettings(uint32_t set) {
  // 4D53090F's defaults (0x82F50D98).
  X_PSCAM_COLOR_SETTINGS settings = {};
  settings.flags = set;
  settings.color_mode = 1;
  settings.auto_exposure = 1;
  settings.metering_window[0] = kFixedOne / 10;
  settings.metering_window[1] = kFixedOne / 10;
  settings.metering_window[2] = kFixedOne * 8 / 10;
  settings.metering_window[3] = kFixedOne * 8 / 10;
  for (auto& gain : settings.gains) {
    gain = kFixedOne;
  }
  settings.refresh_rate = 60 * kFixedOne;
  return settings;
}

struct Completion {
  uint32_t request_ptr = 0;
  uint32_t routine = 0;
  X_STATUS status = X_STATUS_SUCCESS;
  uint32_t bytes = 0;
  // A settings request: color frames carry this generation once it's done.
  bool sets_generation = false;
  uint8_t generation = 0;
};

class CameraService {
 public:
  explicit CameraService(KernelState* kernel_state)
      : kernel_state_(kernel_state),
        memory_(kernel_state->memory()),
        device_(kernel_state->nui()->device()) {
    if (!device_->camera_present()) {
      XELOGI("PsCamDeviceRequest: no Kinect camera");
      return;
    }
    device_->SetFrameListener([this]() { work_event_->Set(); });
    worker_ = object_ref<XHostThread>(new XHostThread(
        kernel_state, 64 * 1024, 0,
        [this]() {
          WorkerMain();
          return 0;
        },
        kernel_state->GetSystemProcess()));
    worker_->set_can_debugger_suspend(true);
    worker_->set_name("Kinect Camera");
    worker_->Create();
  }

  ~CameraService() {
    if (device_->camera_present()) {
      device_->SetFrameListener(nullptr);
    }
    running_ = false;
    work_event_->Set();
    if (worker_) {
      xe::threading::Wait(worker_->thread(), false);
    }
  }

  X_STATUS Request(uint32_t request_ptr) {
    if (!IsGuestRange(request_ptr, sizeof(X_PSCAM_REQUEST))) {
      return X_STATUS_INVALID_PARAMETER;
    }
    auto header = Guest<X_PSCAM_REQUEST>(request_ptr);
    const auto code = PsCamCode(uint32_t(header->code));
    if (!IsGuestRange(request_ptr, RequestSize(code))) {
      return X_STATUS_INVALID_PARAMETER;
    }
    uint32_t routine = header->completion_routine;
    if (routine == kNoCompletionRoutine) {
      routine = 0;
    }
    LogFirst(code, request_ptr);

    if (!device_->camera_present()) {
      // Titles with optional Kinect support, such as Forza Horizon 2, ask for
      // the state and leave the camera alone when it's absent.
      if (code == PsCamCode::kGetState) {
        Guest<X_PSCAM_STATE_REQUEST>(request_ptr)->state = kStateAbsent;
        return X_STATUS_SUCCESS;
      }
      return X_STATUS_DEVICE_NOT_CONNECTED;
    }

    if (code == PsCamCode::kReadFrame) {
      return QueueRead(request_ptr, routine);
    }

    Completion completion;
    completion.request_ptr = request_ptr;
    completion.routine = routine;
    std::vector<Completion> cancelled;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      completion.status = Handle(code, request_ptr, completion.bytes);
      if (code == PsCamCode::kSetColorSettings &&
          XSUCCEEDED(completion.status)) {
        completion.sets_generation = true;
        completion.generation = requested_generation_;
      }
      if (code == PsCamCode::kStop) {
        // Reads waiting for a frame end with the stream.
        for (auto* reads : {&depth_reads_, &color_reads_}) {
          for (Completion& read : *reads) {
            read.status = X_STATUS_CANCELLED;
            cancelled.push_back(read);
          }
          reads->clear();
        }
        streaming_ = false;
        ++stop_count_;
      }
      if (!routine && cancelled.empty()) {
        return completion.status;
      }
      for (Completion& read : cancelled) {
        completions_.push_back(read);
      }
      if (routine) {
        completions_.push_back(completion);
      }
    }
    work_event_->Set();
    return routine ? X_STATUS_PENDING : completion.status;
  }

  // For the Kinect Tuner: the newest frames, if newer than the sequence
  // numbers given, which are updated. False without a camera.
  bool CopyPreview(uint32_t& depth_sequence, std::vector<uint8_t>& packed_depth,
                   uint32_t& color_sequence, std::vector<uint8_t>& bayer) {
    auto* camera = device_->camera();
    if (!camera) {
      return false;
    }
    if (uint32_t sequence =
            camera->CopyDepthFrame(depth_sequence, packed_depth)) {
      depth_sequence = sequence;
    }
    if (uint32_t sequence = camera->CopyColorFrame(color_sequence, bayer)) {
      color_sequence = sequence;
    }
    return true;
  }

 private:
  template <typename T>
  T* Guest(uint32_t address) {
    return memory_->TranslateVirtual<T*>(address);
  }

  bool IsGuestRange(uint32_t address, size_t size) {
    return address && size && memory_->LookupHeap(address) &&
           memory_->LookupHeap(uint32_t(address + size - 1));
  }

  static size_t RequestSize(PsCamCode code) {
    size_t size = 0;
    switch (code) {
      case PsCamCode::kGetState:
        size = sizeof(X_PSCAM_STATE_REQUEST);
        break;
      case PsCamCode::kReadFrame:
        size = sizeof(X_PSCAM_READ_REQUEST);
        break;
      case PsCamCode::kVersion:
        size = sizeof(X_PSCAM_VERSION_REQUEST);
        break;
      case PsCamCode::kRegistration:
        size = sizeof(X_PSCAM_REGISTRATION_REQUEST);
        break;
      case PsCamCode::kDepthTable:
        size = sizeof(X_PSCAM_DEPTH_TABLE_REQUEST);
        break;
      case PsCamCode::kTiming:
        size = sizeof(X_PSCAM_TIMING_REQUEST);
        break;
      case PsCamCode::kSetColorSettings:
      case PsCamCode::kGetColorSettings:
        size = sizeof(X_PSCAM_COLOR_SETTINGS_REQUEST);
        break;
      default:
        break;
    }
    return std::max(size, sizeof(X_PSCAM_REQUEST));
  }

  void LogFirst(PsCamCode code, uint32_t request_ptr) {
    const uint32_t bit = uint32_t(code);
    if (bit >= 64 || (logged_codes_.fetch_or(uint64_t(1) << bit) &
                      (uint64_t(1) << bit))) {
      return;
    }
    const auto* words = Guest<xe::be<uint32_t>>(request_ptr);
    XELOGI("PsCamDeviceRequest: first code {} at {:08X}: {:08X} {:08X} "
           "{:08X} {:08X} {:08X} {:08X} {:08X} {:08X}",
           bit, request_ptr, uint32_t(words[0]), uint32_t(words[1]),
           uint32_t(words[2]), uint32_t(words[3]), uint32_t(words[4]),
           uint32_t(words[5]), uint32_t(words[6]), uint32_t(words[7]));
  }

  X_STATUS QueueRead(uint32_t request_ptr, uint32_t routine) {
    auto read = Guest<X_PSCAM_READ_REQUEST>(request_ptr);
    const uint32_t stream = read->stream;
    const uint32_t counter_ptr = read->frame_counter_ptr;
    if (!routine || (stream != kStreamDepth && stream != kStreamColor) ||
        (counter_ptr && !IsGuestRange(counter_ptr, sizeof(uint32_t)))) {
      return X_STATUS_INVALID_PARAMETER;
    }
    Completion completion;
    completion.request_ptr = request_ptr;
    completion.routine = routine;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      streaming_ = true;
      (stream == kStreamDepth ? depth_reads_ : color_reads_)
          .push_back(completion);
    }
    work_event_->Set();
    return X_STATUS_PENDING;
  }

  // Expects mutex_ held.
  X_STATUS Handle(PsCamCode code, uint32_t request_ptr, uint32_t& bytes) {
    switch (code) {
      case PsCamCode::kGetState:
        Guest<X_PSCAM_STATE_REQUEST>(request_ptr)->state =
            streaming_ ? kStateStreaming : kStateReady;
        return X_STATUS_SUCCESS;
      case PsCamCode::kStop:
      case PsCamCode::kStart:
      case PsCamCode::kTimeSync:
      case PsCamCode::kUnknown33:
        return X_STATUS_SUCCESS;
      case PsCamCode::kVersion: {
        auto version = Guest<X_PSCAM_VERSION_REQUEST>(request_ptr);
        version->major = kDriverVersionMajor;
        version->minor = kDriverVersionMinor;
        return X_STATUS_SUCCESS;
      }
      case PsCamCode::kRegistration:
        WriteRegistration(request_ptr);
        return X_STATUS_SUCCESS;
      case PsCamCode::kDepthTable:
        return WriteDepthTable(request_ptr);
      case PsCamCode::kTiming: {
        auto timing = Guest<X_PSCAM_TIMING_REQUEST>(request_ptr);
        timing->depth_frame_period = kFramePeriodMicroseconds;
        std::memset(timing->unknown_08, 0, sizeof(timing->unknown_08));
        timing->color_frame_period = kFramePeriodMicroseconds;
        std::memset(timing->unknown_1C, 0, sizeof(timing->unknown_1C));
        timing->time_sync = 0;
        std::memset(timing->unknown_28, 0, sizeof(timing->unknown_28));
        return X_STATUS_SUCCESS;
      }
      case PsCamCode::kSetColorSettings: {
        // Kept for code 36. The sensor keeps its own automatic exposure and
        // white balance, which is what titles ask for.
        auto request = Guest<X_PSCAM_COLOR_SETTINGS_REQUEST>(request_ptr);
        active_set_ = request->settings.flags & kSettingsIdentity ? 1 : 0;
        color_settings_[active_set_] = request->settings;
        request->settings_generation = ++requested_generation_;
        return X_STATUS_SUCCESS;
      }
      case PsCamCode::kGetColorSettings: {
        // The set sent last; values the title didn't set read as the
        // defaults, with neutral gains and nothing measured.
        X_PSCAM_COLOR_SETTINGS& settings =
            Guest<X_PSCAM_COLOR_SETTINGS_REQUEST>(request_ptr)->settings;
        settings = color_settings_[active_set_];
        if (!(settings.flags & kSettingsHasGains)) {
          for (auto& gain : settings.gains) {
            gain = kFixedOne;
          }
        }
        settings.measured = 0;
        return X_STATUS_SUCCESS;
      }
      case PsCamCode::kLoadedDepthCalibration: {
        auto words = Guest<xe::be<uint32_t>>(request_ptr);
        const auto calibration =
            kernel_state_->nui()->GetLoadedDepthCalibration();
        words[1] = calibration[0];
        words[2] = calibration[1];
        return X_STATUS_SUCCESS;
      }
      default:
        XELOGW("PsCamDeviceRequest: unsupported code {}", uint32_t(code));
        return X_STATUS_NOT_SUPPORTED;
    }
  }

  void WriteRegistration(uint32_t request_ptr) {
    const auto& calibration = device_->camera()->calibration();
    auto request = Guest<X_PSCAM_REGISTRATION_REQUEST>(request_ptr);
    static_assert(sizeof(request->registration) ==
                  sizeof(calibration.registration));
    for (size_t i = 0; i < xe::countof(request->registration); ++i) {
      // The camera sends the record little-endian.
      request->registration[i] =
          xe::load<uint32_t>(calibration.registration.data() + i * 4);
    }
    request->reference_distance = calibration.reference_distance;
    request->reference_pixel_size = calibration.reference_pixel_size;
    request->frame_width = uint16_t(kFrameWidth);
    request->shift_to_depth_pixel_const = kShiftToDepthPixelConst;
    request->shift_to_depth_const_offset = kShiftToDepthConstOffset;
    request->dcmos_rcmos_distance = calibration.dcmos_rcmos_distance;
  }

  X_STATUS WriteDepthTable(uint32_t request_ptr) {
    auto request = Guest<X_PSCAM_DEPTH_TABLE_REQUEST>(request_ptr);
    const uint32_t table_ptr = request->table_ptr;
    if (request->entry_count != kDepthTableEntries ||
        !IsGuestRange(table_ptr, kDepthTableEntries * sizeof(uint16_t))) {
      return X_STATUS_INVALID_PARAMETER;
    }
    const auto& calibration = device_->camera()->calibration();
    auto table = Guest<xe::be<uint16_t>>(table_ptr);
    for (uint32_t raw = 0; raw < kDepthTableEntries; ++raw) {
      uint16_t millimeters = 0;
      // Raw 0 is no depth, as frames give it (see UnpackDepth).
      if (raw != 0 && raw < kNoDepthRaw) {
        // libfreenect's freenect_raw_to_mm.
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
          millimeters = uint16_t(value);
        }
      }
      table[raw] = millimeters;
    }
    return X_STATUS_SUCCESS;
  }

  // Big-endian 11-bit samples, from the high bit, into big-endian words.
  // Pixels without depth (2047) come as 0, which the depth table maps to no
  // distance.
  static void UnpackDepth(const std::vector<uint8_t>& packed, uint8_t* out) {
    uint32_t bits = 0;
    int bit_count = 0;
    size_t in = 0;
    for (size_t i = 0; i < kFrameWidth * kFrameHeight; ++i) {
      while (bit_count < 11) {
        bits = (bits << 8) | packed[in++];
        bit_count += 8;
      }
      bit_count -= 11;
      uint16_t raw = uint16_t((bits >> bit_count) & 0x7FF);
      if (raw == kNoDepthRaw) {
        raw = 0;
      }
      xe::store_and_swap<uint16_t>(out + i * 2, raw);
    }
  }

  // Completes a read with the newest frame of its stream; false when there's
  // no newer frame yet.
  bool CompleteRead(Completion read, bool depth) {
    auto* camera = device_->camera();
    uint32_t& last = depth ? depth_sequence_ : color_sequence_;
    const uint32_t sequence = depth ? camera->CopyDepthFrame(last, frame_)
                                    : camera->CopyColorFrame(last, frame_);
    if (!sequence) {
      return false;
    }
    last = sequence;
    auto request = Guest<X_PSCAM_READ_REQUEST>(read.request_ptr);
    const uint32_t bytes = depth ? kDepthFrameBytes : kColorFrameBytes;
    const uint32_t buffer = request->buffer_ptr;
    if (request->buffer_size < bytes || !IsGuestRange(buffer, bytes)) {
      read.status = X_STATUS_INVALID_PARAMETER;
      Complete(read);
      return true;
    }
    // Through the virtual view, as the title's own stores would: telling the
    // memory watches made 4D5308C9 lose the floor it had found.
    if (depth) {
      Record();
      UnpackDepth(frame_, Guest<uint8_t>(buffer));
    } else {
      if (!cvars::kinect_record_path.empty()) {
        record_color_ = frame_;
      }
      std::memcpy(Guest<uint8_t>(buffer), frame_.data(), bytes);
      request->settings_generation = settings_generation_;
    }
    if (const uint32_t counter_ptr = request->frame_counter_ptr) {
      *Guest<xe::be<uint32_t>>(counter_ptr) = sequence;
    }
    if (!(depth ? logged_depth_ : logged_color_)) {
      (depth ? logged_depth_ : logged_color_) = true;
      XELOGI("PsCamDeviceRequest: first {} frame {} to {:08X}",
             depth ? "depth" : "color", sequence, buffer);
    }
    read.bytes = bytes;
    Complete(read);
    return true;
  }

  // The depth frame in frame_ with the latest color frame, to
  // kinect_record_path; from the first color frame on, as playback wants both.
  void Record() {
    if (cvars::kinect_record_path.empty() || record_color_.empty() ||
        record_failed_) {
      return;
    }
    const std::filesystem::path& directory = cvars::kinect_record_path;
    if (!record_index_) {
      std::error_code error;
      std::filesystem::create_directories(directory, error);
      if (!hid::kinect::WriteRecordingCalibration(
              directory, device_->camera()->calibration())) {
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

  // Runs the completion routine on the worker, which doesn't read the
  // request: the title may reuse it.
  void Complete(const Completion& completion) {
    if (completion.routine) {
      uint64_t args[] = {completion.request_ptr,
                         uint32_t(completion.status), completion.bytes};
      kernel_state_->processor()->Execute(worker_->thread_state(),
                                          completion.routine, args,
                                          xe::countof(args));
    }
    if (completion.sets_generation) {
      settings_generation_ = completion.generation;
    }
  }

  void WorkerMain() {
    while (running_) {
      xe::threading::Wait(work_event_.get(), false,
                          std::chrono::milliseconds(10));
      std::deque<Completion> completions;
      Completion depth_read, color_read;
      uint32_t stop_count;
      {
        std::lock_guard<std::mutex> lock(mutex_);
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
      for (const Completion& completion : completions) {
        Complete(completion);
      }
      for (bool depth : {true, false}) {
        Completion& read = depth ? depth_read : color_read;
        if (!read.request_ptr || CompleteRead(read, depth)) {
          continue;
        }
        // No new frame yet: back in front, unless streaming stopped meanwhile.
        bool cancelled;
        {
          std::lock_guard<std::mutex> lock(mutex_);
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
  kinect::KinectDevice* device_;
  std::unique_ptr<xe::threading::Event> work_event_ =
      xe::threading::Event::CreateAutoResetEvent(false);
  object_ref<XHostThread> worker_;
  std::atomic<bool> running_ = true;

  std::mutex mutex_;
  std::deque<Completion> completions_;
  std::deque<Completion> depth_reads_;
  std::deque<Completion> color_reads_;
  bool streaming_ = false;
  uint32_t stop_count_ = 0;
  uint8_t requested_generation_ = 0;
  std::array<X_PSCAM_COLOR_SETTINGS, 2> color_settings_ = {
      DefaultColorSettings(kSettingsPlay),
      DefaultColorSettings(kSettingsIdentity)};
  uint32_t active_set_ = 0;

  // Worker only.
  uint32_t depth_sequence_ = 0;
  uint32_t color_sequence_ = 0;
  std::vector<uint8_t> frame_;
  std::vector<uint8_t> record_color_;
  uint32_t record_index_ = 0;
  bool record_failed_ = false;
  bool logged_depth_ = false;
  bool logged_color_ = false;

  std::atomic<uint8_t> settings_generation_ = 0;
  std::atomic<uint64_t> logged_codes_ = 0;
};

std::mutex camera_service_mutex;
std::unique_ptr<CameraService> camera_service;

CameraService* GetCameraService() {
  std::lock_guard<std::mutex> lock(camera_service_mutex);
  if (!camera_service) {
    camera_service = std::make_unique<CameraService>(kernel_state());
  }
  return camera_service.get();
}

}  // namespace

void ShutdownCameraService() {
  std::unique_ptr<CameraService> service;
  {
    std::lock_guard<std::mutex> lock(camera_service_mutex);
    service = std::move(camera_service);
  }
  // Destroyed unlocked: its worker may be completing a request.
  service.reset();
}

bool CopyKinectCameraPreview(uint32_t& depth_sequence,
                             std::vector<uint8_t>& packed_depth,
                             uint32_t& color_sequence,
                             std::vector<uint8_t>& bayer) {
  return GetCameraService()->CopyPreview(depth_sequence, packed_depth,
                                         color_sequence, bayer);
}

dword_result_t PsCamDeviceRequest_entry(dword_t request_ptr) {
  if (!request_ptr) {
    return X_STATUS_INVALID_PARAMETER;
  }
  return GetCameraService()->Request(request_ptr);
}
DECLARE_XBOXKRNL_EXPORT1(PsCamDeviceRequest, kNone, kImplemented);

}  // namespace xboxkrnl
}  // namespace kernel
}  // namespace xe

DECLARE_XBOXKRNL_EMPTY_REGISTER_EXPORTS(PsCam);

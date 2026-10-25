/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/kernel/kinect/kinect_device.h"

#include <cstdio>

#include "xenia/base/cvar.h"
#include "xenia/base/filesystem.h"
#include "xenia/base/logging.h"
#include "xenia/base/memory.h"
#include "xenia/base/string.h"
#include "xenia/vfs/devices/xcontent_container_device.h"
#include "xenia/vfs/entry.h"
#include "xenia/vfs/file.h"

DEFINE_bool(allow_kinect_camera, false,
            "Give titles a connected Kinect for Xbox 360 (model 1473): its "
            "depth and color cameras, microphone array and motor.",
            "HID");

DEFINE_bool(kinect_use_usbdk, true,
            "Access the Kinect through UsbDk, which borrows it from its "
            "Windows driver while the emulator runs. When disabled, the "
            "camera must use the WinUSB driver. Windows only; Linux uses "
            "usbfs and detaches the kernel driver itself.",
            "HID");

DEFINE_path(kinect_replay_path, "",
            "Play the Kinect recording in this directory (made with "
            "xenia-kinect-camera-test --record_dir) instead of a connected "
            "Kinect.",
            "HID");

DEFINE_path(kinect_audio_firmware_path, "",
            "Microsoft's Kinect audio firmware, uploaded to the sensor when it "
            "opens: audios.bin, or the Xbox 360 system update package that "
            "carries it ($SystemUpdate/FFFE07DF00000001). Without it the "
            "microphones and model 1473's motor don't work.",
            "HID");

DEFINE_path(kinect_audio_calibration_path, "",
            "The console's Kinect microphone array calibration "
            "(MicArrayCalibrate.data from the MicArrayCalibrate.mec system "
            "package), sent to the sensor when it opens.",
            "HID");

namespace xe {
namespace kernel {
namespace kinect {

namespace {

// audios.bin itself, or read out of the system update package it ships in.
std::vector<uint8_t> ReadAudioFirmware(const std::filesystem::path& path) {
  std::vector<uint8_t> image;
  auto header = vfs::XContentContainerDevice::ReadContainerHeader(path);
  if (header && header->content_header.is_magic_valid()) {
    auto device = vfs::XContentContainerDevice::CreateContentDevice("", path);
    if (!device || !device->Initialize()) {
      return image;
    }
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

}  // namespace

KinectDevice::KinectDevice() {
  auto on_frame = [this]() { OnFrame(); };
  if (!cvars::kinect_replay_path.empty()) {
    camera_ = hid::kinect::UsbCamera::OpenRecording(cvars::kinect_replay_path,
                                                    on_frame);
    is_recording_ = camera_ != nullptr;
    XELOGI("Kinect: {}", camera_ ? "playing a recording"
                                 : "no recording in kinect_replay_path");
    return;
  }
  if (!cvars::allow_kinect_camera) {
    return;
  }
  // The audio device runs Microsoft's firmware, which must be uploaded before
  // it shows up with its microphones and motor.
  if (!cvars::kinect_audio_firmware_path.empty()) {
    const std::vector<uint8_t> image =
        ReadAudioFirmware(cvars::kinect_audio_firmware_path);
    if (image.empty()) {
      XELOGE("Kinect: cannot read the audio firmware from {}",
             xe::strip_pii(
                 xe::path_to_utf8(cvars::kinect_audio_firmware_path)));
    } else {
      XELOGI("Kinect: uploading the {}-byte audio firmware", image.size());
      hid::kinect::LoadAudioFirmware(image);
    }
  }
  audio_ = hid::kinect::UsbAudio::Open();
  if (audio_ && !cvars::kinect_audio_calibration_path.empty()) {
    SendAudioCalibration();
  }
  camera_ = hid::kinect::UsbCamera::Open(cvars::kinect_use_usbdk, on_frame);
  XELOGI("Kinect: camera {}, audio device {}", camera_ ? "open" : "not found",
         audio_ ? "open" : "not found");
}

KinectDevice::~KinectDevice() {
  if (audio_) {
    audio_->StopMicrophones();
  }
  // The camera threads stop before the listener goes away.
  camera_.reset();
}

void KinectDevice::SetFrameListener(std::function<void()> listener) {
  std::lock_guard<std::mutex> lock(listener_mutex_);
  frame_listener_ = std::move(listener);
}

void KinectDevice::OnFrame() {
  std::lock_guard<std::mutex> lock(listener_mutex_);
  if (frame_listener_) {
    frame_listener_();
  }
}

bool KinectDevice::SetTiltAngle(int32_t degrees) {
  std::lock_guard<std::mutex> lock(audio_mutex_);
  return audio_ && audio_->SetTiltAngle(degrees);
}

bool KinectDevice::StopTilt() {
  std::lock_guard<std::mutex> lock(audio_mutex_);
  return audio_ && audio_->StopMovement();
}

bool KinectDevice::ReadMotorState(hid::kinect::UsbAudio::State& state) {
  std::lock_guard<std::mutex> lock(audio_mutex_);
  return audio_ && audio_->ReadState(state);
}

bool KinectDevice::SetLed(hid::kinect::UsbAudio::Led led) {
  std::lock_guard<std::mutex> lock(audio_mutex_);
  return audio_ && audio_->SetLed(led);
}

bool KinectDevice::SendAudioMessage(const std::vector<uint8_t>& message) {
  std::lock_guard<std::mutex> lock(audio_mutex_);
  return audio_ && audio_->SendAudioMessage(message);
}

bool KinectDevice::SendAudioCalibration() {
  if (!audio_ || cvars::kinect_audio_calibration_path.empty()) {
    return false;
  }
  std::vector<uint8_t> message;
  if (FILE* file = xe::filesystem::OpenFile(
          cvars::kinect_audio_calibration_path, "rb")) {
    message.resize(hid::kinect::UsbAudio::kAudioCalibrationBytes + 1);
    message.resize(std::fread(message.data(), 1, message.size(), file));
    std::fclose(file);
  }
  if (message.size() != hid::kinect::UsbAudio::kAudioCalibrationBytes ||
      xe::load<uint32_t>(message.data()) !=
          hid::kinect::UsbAudio::kAudioMessageMagic) {
    XELOGE("Kinect: {} isn't a Kinect microphone array calibration",
           xe::strip_pii(
               xe::path_to_utf8(cvars::kinect_audio_calibration_path)));
    return false;
  }
  const bool sent = SendAudioMessage(message);
  XELOGI("Kinect: microphone array calibration {}",
         sent ? "sent" : "refused");
  return sent;
}

bool KinectDevice::StartMicrophones(std::function<void()> on_window) {
  std::lock_guard<std::mutex> lock(audio_mutex_);
  return audio_ && audio_->StartMicrophones(std::move(on_window));
}

void KinectDevice::StopMicrophones() {
  std::lock_guard<std::mutex> lock(audio_mutex_);
  if (audio_) {
    audio_->StopMicrophones();
  }
}

bool KinectDevice::ReadMicrophoneWindow(
    uint64_t& next, hid::kinect::UsbAudio::MicrophoneWindow& window) {
  // Windows are read from their own buffer, without a control transfer.
  return audio_ && audio_->ReadMicrophoneWindow(next, window);
}

const std::string& KinectDevice::serial_number() {
  std::call_once(serial_once_, [this]() {
    if (camera_ && !is_recording_) {
      serial_number_ = hid::kinect::ReadSerialNumber();
    }
  });
  return serial_number_;
}

}  // namespace kinect
}  // namespace kernel
}  // namespace xe

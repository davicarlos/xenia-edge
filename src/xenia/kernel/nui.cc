/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2020 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/kernel/nui.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include "xenia/base/assert.h"
#include "xenia/base/cvar.h"
#include "xenia/base/filesystem.h"
#include "xenia/config.h"
#include "xenia/base/logging.h"
#include "xenia/hid/kinect/usb_audio.h"
#include "xenia/hid/kinect/usb_camera.h"
#include "xenia/kernel/nui_audio.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/title_id_utils.h"
#include "xenia/kernel/xam/profile_manager.h"
#include "xenia/kernel/xam/xam_state.h"
#include "xenia/xbox.h"

DEFINE_bool(kinect_auto_tilt, true,
            "Tilt the Kinect motor so the tracked player's whole body fits "
            "the depth image, and save that elevation as kinect_elevation.",
            "HID");

DECLARE_bool(allow_kinect_camera);

DEFINE_path(kinect_audio_calibration_path, "",
            "The console's Kinect microphone array calibration, "
            "MicArrayCalibrate.data from its MicArrayCalibrate.mec system "
            "package. Sent to the sensor's audio processing when it opens.",
            "HID");

DEFINE_int32(kinect_elevation, 0,
             "Kinect elevation in degrees the console is calibrated to, as "
             "the Kinect Tuner saves it. Titles tilt the sensor straight there "
             "instead of searching for the floor at -25 degrees first.",
             "HID");

namespace xe {
namespace kernel {
namespace {
constexpr auto kTiltStatePeriod = std::chrono::milliseconds(50);
// Automatic tilt: the depth camera sees 21.5 degrees above and below its axis;
// the body should fit with a margin, from a bit above the head joint to a bit
// below the feet. It tilts once the player has been out of the picture for
// about two seconds and the image has settled after any tilt. Once the player
// fit, it lets them leave the picture by a few degrees before tilting again.
constexpr float kDepthHalfFieldOfView = 21.5f;
constexpr float kAutoTiltMargin = 2.0f;
constexpr float kAutoTiltFramedTolerance = 2.0f;
constexpr float kAboveHead = 0.15f;
constexpr float kBelowFeet = 0.05f;
constexpr uint32_t kAutoTiltOutFrames = 60;
constexpr auto kAutoTiltSettleTime = std::chrono::seconds(4);
constexpr int32_t kAutoTiltMinimumStep = 2;
constexpr uint32_t kHeadJoint = 3;
// Knees, ankles and feet, whichever is lowest stands for the feet.
constexpr uint32_t kLegJoints[] = {13, 14, 15, 17, 18, 19};
// Motor status flags meaning it's moving or settling.
constexpr uint32_t kMotorMovingFlags = 0x3;
}  // namespace

NUIController::NUIController() : audio_(std::make_unique<NuiAudio>()) {}

NUIController::~NUIController() = default;
bool NUIController::NuiInitialized() {
  // TODO(boma): Proper device readiness.
  return (KernelState::shared()->xconfig()->ReadSetting<uint32_t>(
              X_CONFIG_CATEGORY::XCONFIG_USER_CATEGORY,
              XCONFIG_USER_RETAIL_FLAGS) &
          X_RETAIL_FLAGS::KinectInitialized) != 0;
}
uint32_t NUIController::GetNUIDataPtr() { return nui_data_ptr; }
char NUIController::GetUnknown2() { return nui_unknown_2; }
void NUIController::SetTiltCallback(uint32_t callback) {
  tilt_callback_ = callback;
}
void NUIController::SetEngagedTrackingId(uint32_t tracking_id) {
  engaged_tracking_id = tracking_id;
}
uint32_t NUIController::GetEngagedTrackingId() { return engaged_tracking_id; }
uint32_t NUIController::GetInitFlags() { return init_flags_; }
void NUIController::SetInitFlags(uint32_t flags) { init_flags_ = flags; }
uint64_t NUIController::GetNUIVersion(uint32_t index) {
  if (index > 1) {
    // You set index out of bounds
    assert_always();
    return 0;
  }
  return nui_versions_[index];
}
void NUIController::SetNUIVersion(uint64_t version, uint32_t index) {
  nui_versions_[index] = version;
}
uint64_t NUIController::GetSessionId() { return session_id; }
void NUIController::SetSessionId(uint64_t id) { session_id = id; }

void NUIController::SetElevationAngle(int32_t angle) {
  MoveMotor(angle);
}

bool NUIController::MoveMotor(int32_t angle) {
  elevation_angle_ =
      std::clamp(angle, kElevationMinimum, kElevationMaximum);
  std::lock_guard<std::mutex> lock(motor_mutex_);
  last_tilt_time_ = std::chrono::steady_clock::now();
  auto_tilt_out_frames_ = 0;
  auto_tilt_framed_ = false;
  if (!motor_) {
    return false;
  }
  if (!motor_->SetTiltAngle(elevation_angle_)) {
    XELOGW("NUI: the Kinect motor did not accept tilt {}", elevation_angle_);
    return false;
  }
  XELOGI("NUI: tilting the Kinect to {} degrees", elevation_angle_);
  return true;
}

void NUIController::UpdateAutoTilt(const NuiSkeletonSnapshot& snapshot) {
  if (!cvars::kinect_auto_tilt) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    if (!motor_ || std::chrono::steady_clock::now() - last_tilt_time_ <
                       kAutoTiltSettleTime) {
      return;
    }
  }
  // The closest tracked player.
  const NuiSkeletonSnapshot::Skeleton* player = nullptr;
  for (const auto& skeleton : snapshot.skeletons) {
    if (skeleton.state == NuiSkeletonSnapshot::TrackingState::kTracked &&
        skeleton.position[2] > 0.0f &&
        (!player || skeleton.position[2] < player->position[2])) {
      player = &skeleton;
    }
  }
  if (!player) {
    auto_tilt_out_frames_ = 0;
    return;
  }
  // Inferred joints still carry the tracker's estimate, which is what's
  // needed for parts outside the image.
  const auto& head = player->joints[kHeadJoint];
  const std::array<float, 3>* feet = nullptr;
  for (uint32_t joint : kLegJoints) {
    if (player->joint_states[joint] &&
        (!feet || player->joints[joint][1] < (*feet)[1])) {
      feet = &player->joints[joint];
    }
  }
  if (!player->joint_states[kHeadJoint] || !feet || head[2] <= 0.0f ||
      (*feet)[2] <= 0.0f) {
    return;
  }
  constexpr float kDegrees = 180.0f / 3.14159265f;
  const float top = std::atan2(head[1] + kAboveHead, head[2]) * kDegrees;
  const float bottom =
      std::atan2((*feet)[1] - kBelowFeet, (*feet)[2]) * kDegrees;
  const float limit = kDepthHalfFieldOfView - kAutoTiltMargin;
  const float allowed =
      auto_tilt_framed_ ? kDepthHalfFieldOfView + kAutoTiltFramedTolerance
                        : limit;
  if (top <= allowed && bottom >= -allowed) {
    auto_tilt_out_frames_ = 0;
    if (top > limit || bottom < -limit) {
      return;
    }
    // Framed. Titles start at this elevation from now on.
    auto_tilt_framed_ = true;
    if (auto_tilted_) {
      auto_tilted_ = false;
      if (GetCalibratedElevation() != elevation_angle_) {
        SetCalibratedElevation(elevation_angle_);
        config::SaveConfig();
        XELOGI("NUI: saved {} degrees as the Kinect elevation",
               elevation_angle_);
      }
    }
    return;
  }
  if (++auto_tilt_out_frames_ < kAutoTiltOutFrames) {
    return;
  }
  // Center the body vertically; a player too tall for the image keeps the
  // feet in it, which the skeleton tracker needs more than the top of the
  // head.
  float shift = (top + bottom) * 0.5f;
  if (top - bottom > 2.0f * limit) {
    shift = bottom + limit;
  }
  const int32_t target =
      std::clamp(int32_t(std::lround(elevation_angle_ + shift)),
                 kElevationMinimum, kElevationMaximum);
  if (std::abs(target - elevation_angle_) < kAutoTiltMinimumStep) {
    return;
  }
  XELOGI("NUI: player spans {:.0f} to {:.0f} degrees; tilting to frame them",
         bottom, top);
  if (MoveMotor(target)) {
    auto_tilted_ = true;
  }
}

const std::string& NUIController::GetSerialNumber() {
  std::call_once(serial_number_once_, [this]() {
    if (cvars::allow_kinect_camera) {
      serial_number_ = hid::kinect::ReadSerialNumber();
      XELOGI("NUI: Kinect serial number {}",
             serial_number_.empty() ? "unknown" : serial_number_);
    }
  });
  return serial_number_;
}

NUIController::BiometricData NUIController::GetCachedBiometricData(
    uint64_t xuid) {
  std::lock_guard<std::mutex> lock(biometric_mutex_);
  auto it = cached_biometric_data_.find(xuid);
  return it != cached_biometric_data_.end() ? it->second : BiometricData();
}

void NUIController::SetCachedBiometricData(uint64_t xuid, BiometricData data) {
  std::lock_guard<std::mutex> lock(biometric_mutex_);
  cached_biometric_data_[xuid] = std::move(data);
}

namespace {
// The profile's copy: the version (big-endian) and the title's data.
std::filesystem::path GetBiometricDataPath(uint64_t xuid) {
  return KernelState::shared()
             ->xam_state()
             ->profile_manager()
             ->GetProfileContentPath(
             xuid, kDashboardID) /
         "NuiBiometric.bin";
}
}  // namespace

bool NUIController::ReadStoredBiometricData(uint64_t xuid,
                                            BiometricData& data) {
  std::lock_guard<std::mutex> lock(biometric_mutex_);
  FILE* file = xe::filesystem::OpenFile(GetBiometricDataPath(xuid), "rb");
  if (!file) {
    return false;
  }
  uint8_t version[4];
  bool read = std::fread(version, 1, sizeof(version), file) == sizeof(version);
  if (read) {
    data.version = xe::load_and_swap<uint32_t>(version);
    data.data.clear();
    uint8_t buffer[4096];
    size_t length;
    while ((length = std::fread(buffer, 1, sizeof(buffer), file)) > 0) {
      data.data.insert(data.data.end(), buffer, buffer + length);
    }
  }
  std::fclose(file);
  return read && !data.data.empty();
}

bool NUIController::WriteStoredBiometricData(uint64_t xuid,
                                             const BiometricData& data) {
  std::lock_guard<std::mutex> lock(biometric_mutex_);
  const std::filesystem::path path = GetBiometricDataPath(xuid);
  std::error_code error;
  if (data.data.empty()) {
    std::filesystem::remove(path, error);
    return true;
  }
  std::filesystem::create_directories(path.parent_path(), error);
  FILE* file = xe::filesystem::OpenFile(path, "wb");
  if (!file) {
    XELOGE("NUI: cannot write the Kinect ID data to {}",
           xe::strip_pii(xe::path_to_utf8(path)));
    return false;
  }
  uint8_t version[4];
  xe::store_and_swap<uint32_t>(version, data.version);
  const bool written =
      std::fwrite(version, 1, sizeof(version), file) == sizeof(version) &&
      std::fwrite(data.data.data(), 1, data.data.size(), file) ==
          data.data.size();
  std::fclose(file);
  XELOGI("NUI: saved {} bytes of Kinect ID data for {:016X}", data.data.size(),
         xuid);
  return written;
}

int32_t NUIController::GetCalibratedElevation() const {
  return std::clamp(int32_t(cvars::kinect_elevation), kElevationMinimum,
                    kElevationMaximum);
}

void NUIController::SetCalibratedElevation(int32_t angle) {
  OVERRIDE_PERSIST_int32(
      kinect_elevation,
      std::clamp(angle, kElevationMinimum, kElevationMaximum));
}

void NUIController::StopElevationMovement() {
  std::lock_guard<std::mutex> lock(motor_mutex_);
  if (motor_ && !motor_->StopMovement()) {
    XELOGW("NUI: the Kinect motor did not stop");
  }
}

void NUIController::SetMotor(std::unique_ptr<hid::kinect::UsbAudio> motor) {
  std::lock_guard<std::mutex> lock(motor_mutex_);
  audio_->SetDevice(motor.get());
  motor_ = std::move(motor);
  tilt_state_time_ = {};
  if (motor_) {
    motor_->SetLed(hid::kinect::UsbAudio::Led::kGreen);
    XELOGI("NUI: using the Kinect tilt motor");
  }
  if (motor_ && !cvars::kinect_audio_calibration_path.empty()) {
    SendAudioCalibration();
  }
}

bool NUIController::SetMicrophoneBeamAngle(float radians) {
  std::lock_guard<std::mutex> lock(motor_mutex_);
  if (!motor_) {
    return false;
  }
  const uint32_t words[3] = {hid::kinect::UsbAudio::kAudioMessageMagic, 0,
                             hid::kinect::UsbAudio::kAudioMessageBeamAngle};
  std::vector<uint8_t> message(sizeof(words) + sizeof(radians));
  std::memcpy(message.data(), words, sizeof(words));
  std::memcpy(message.data() + sizeof(words), &radians, sizeof(radians));
  return motor_->SendAudioMessage(message);
}

bool NUIController::LoadAudioCalibration() {
  std::lock_guard<std::mutex> lock(motor_mutex_);
  return motor_ && SendAudioCalibration();
}

bool NUIController::SendAudioCalibration() {
  const std::filesystem::path& path = cvars::kinect_audio_calibration_path;
  if (path.empty()) {
    return false;
  }
  std::vector<uint8_t> message;
  if (FILE* file = xe::filesystem::OpenFile(path, "rb")) {
    uint8_t buffer[0x4000];
    size_t read;
    while ((read = std::fread(buffer, 1, sizeof(buffer), file)) > 0 &&
           message.size() <= hid::kinect::UsbAudio::kAudioCalibrationBytes) {
      message.insert(message.end(), buffer, buffer + read);
    }
    std::fclose(file);
  }
  uint32_t magic = 0;
  if (message.size() >= sizeof(magic)) {
    std::memcpy(&magic, message.data(), sizeof(magic));
  }
  if (message.size() != hid::kinect::UsbAudio::kAudioCalibrationBytes ||
      magic != hid::kinect::UsbAudio::kAudioMessageMagic) {
    XELOGE("NUI: {} isn't a Kinect microphone array calibration",
           xe::strip_pii(xe::path_to_utf8(path)));
    return false;
  }
  if (!motor_->SendAudioMessage(message)) {
    XELOGE("NUI: the Kinect didn't take the microphone array calibration");
    return false;
  }
  XELOGI("NUI: sent the Kinect microphone array calibration");
  return true;
}

bool NUIController::GetTiltState(TiltState& state) {
  std::lock_guard<std::mutex> lock(motor_mutex_);
  if (!motor_) {
    return false;
  }
  const auto now = std::chrono::steady_clock::now();
  if (now - tilt_state_time_ >= kTiltStatePeriod) {
    hid::kinect::UsbAudio::State motor_state;
    if (!motor_->ReadState(motor_state)) {
      return false;
    }
    for (size_t i = 0; i < 3; ++i) {
      tilt_state_.gravity[i] = motor_state.accelerometer[i];
    }
    tilt_state_.flags = motor_state.flags & kMotorMovingFlags;
    tilt_state_time_ = now;
  }
  state = tilt_state_;
  return true;
}

void NUIController::SetSkeletonSnapshot(const NuiSkeletonSnapshot& snapshot) {
  {
    std::lock_guard<std::mutex> lock(skeleton_mutex_);
    skeleton_snapshot_ = snapshot;
  }
  UpdateAutoTilt(snapshot);
}

NuiSkeletonSnapshot NUIController::GetSkeletonSnapshot() const {
  std::lock_guard<std::mutex> lock(skeleton_mutex_);
  return skeleton_snapshot_;
}

uint32_t NUIController::NuiHudCheck(uint64_t tracking_id) {
  if (!GetNUIDataPtr() || !GetUnknown2() || !NuiInitialized()) {
    return X_ERROR_ACCESS_DENIED;
  }
  if (tracking_id) {
    SetEngagedTrackingId(tracking_id);
  };
  return X_ERROR_SUCCESS;
};

}  // namespace kernel
}  // namespace xe

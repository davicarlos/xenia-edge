/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2022 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/kernel/nui.h"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <random>

#include "xenia/base/filesystem.h"
#include "xenia/base/logging.h"
#include "xenia/base/memory.h"
#include "xenia/emulator.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/kinect/kinect_audio.h"
#include "xenia/kernel/kinect/kinect_device.h"
#include "xenia/xbox.h"

namespace xe {
namespace kernel {

NUIController::NUIController() {
  for (auto& enrollment : user_enrollments_) {
    enrollment = kNoEnrollment;
  }
}

NUIController::~NUIController() {
  // The audio thread reads the device.
  audio_.reset();
  device_.reset();
}

// Opened on first use rather than at start, so that titles without Kinect
// support leave the sensor to the host.
kinect::KinectDevice* NUIController::device() {
  std::call_once(device_once_, [this]() {
    device_ = std::make_unique<kinect::KinectDevice>();
  });
  return device_.get();
}

kinect::KinectAudio* NUIController::audio() {
  std::call_once(audio_once_, [this]() {
    audio_ = std::make_unique<kinect::KinectAudio>(device());
  });
  return audio_.get();
}

// The retail flags in the user settings (XCONFIG_USER_RETAIL_FLAGS).
bool NUIController::NuiInitialized() {
  const uint32_t flags =
      KernelState::shared()->xconfig()->ReadSetting<uint32_t>(
          X_CONFIG_CATEGORY::XCONFIG_USER_CATEGORY, XCONFIG_USER_RETAIL_FLAGS);
  return (flags & X_RETAIL_FLAGS::KinectInitialized) &&
         !(flags & X_RETAIL_FLAGS::KinectDisabled);
}

bool NUIController::IsDeviceReady() {
  return NuiInitialized() && device()->camera_present();
}

void NUIController::InitializeTitle(uint64_t xam_version, uint64_t xtl_version,
                                    uint32_t flags) {
  nui_versions_[0] = xam_version;
  nui_versions_[1] = xtl_version;
  init_flags_ = flags;
  // The HUD starts with nobody engaged (0x816AD130).
  hud_enabled_ = !(flags & 0x80000000);
  engaged_tracking_id_ = 0;
  XELOGI("NUI: title runtime {:X}/{:X} with flags {:08X}, HUD {}",
         xam_version, xtl_version, flags, hud_enabled_ ? "on" : "off");
}

void NUIController::ShutdownTitle() {
  nui_versions_[0] = 0;
  nui_versions_[1] = 0;
  init_flags_ = 0;
  hud_enabled_ = false;
  engaged_tracking_id_ = 0;
  tilt_engine_ = 0;
  tilt_status_listener_ = 0;
  frame_observer_ = 0;
  identity_engine_ = 0;
}

uint32_t NUIController::NuiHudCheck(uint32_t tracking_id) {
  // A tracking ID of 0 keeps whoever is engaged.
  if (!IsDeviceReady()) {
    return X_ERROR_ACCESS_DENIED;
  }
  if (tracking_id) {
    engaged_tracking_id_ = tracking_id;
  }
  return X_ERROR_SUCCESS;
}

uint32_t NUIController::GetUserEnrollment(uint32_t user_index) const {
  return user_index < kUserCount ? user_enrollments_[user_index].load()
                                 : kNoEnrollment;
}

void NUIController::SetUserEnrollment(uint32_t user_index,
                                      uint32_t enrollment_index) {
  if (user_index < kUserCount) {
    user_enrollments_[user_index] = enrollment_index;
  }
}

// Kinect ID matches a skeleton to an enrollment; this is how xam turns that
// into the signed-in user it belongs to.
uint32_t NUIController::FindEnrolledUser(uint32_t enrollment_index) const {
  if (enrollment_index == kNoEnrollment) {
    return kUserCount;
  }
  for (uint32_t i = 0; i < kUserCount; ++i) {
    if (user_enrollments_[i] == enrollment_index) {
      return i;
    }
  }
  return kUserCount;
}

NUIController::BiometricData NUIController::GetCachedBiometricData(
    uint64_t xuid) {
  std::lock_guard<std::mutex> lock(biometric_mutex_);
  auto it = biometric_cache_.find(xuid);
  return it != biometric_cache_.end() ? it->second : BiometricData();
}

void NUIController::SetCachedBiometricData(uint64_t xuid, BiometricData data) {
  std::lock_guard<std::mutex> lock(biometric_mutex_);
  biometric_cache_[xuid] = std::move(data);
}

bool NUIController::HasCachedBiometricData(uint64_t xuid) {
  std::lock_guard<std::mutex> lock(biometric_mutex_);
  return biometric_cache_.count(xuid) != 0;
}

uint64_t NUIController::GetSessionId() {
  // Fixed for the emulator's run; never 0.
  std::call_once(session_once_, [this]() {
    std::random_device random;
    do {
      session_id_ = (uint64_t(random()) << 32) | random();
    } while (!session_id_);
  });
  return session_id_;
}

bool NUIController::SetElevationAngle(int32_t degrees) {
  std::lock_guard<std::mutex> lock(motor_mutex_);
  elevation_angle_ = std::clamp(degrees, kElevationMinimum, kElevationMaximum);
  const bool moved = device()->SetTiltAngle(elevation_angle_);
  XELOGI("NUI: tilt to {} degrees{}", elevation_angle_.load(),
         moved ? "" : " (no motor)");
  return moved;
}

bool NUIController::StopElevation() {
  std::lock_guard<std::mutex> lock(motor_mutex_);
  return device()->StopTilt();
}

std::array<float, 4> NUIController::GetRememberedFloor() const {
  std::lock_guard<std::mutex> lock(floor_mutex_);
  return remembered_floor_;
}

void NUIController::SetRememberedFloor(const std::array<float, 4>& plane) {
  std::lock_guard<std::mutex> lock(floor_mutex_);
  remembered_floor_ = plane;
}

namespace {
// One file for every title, in the storage root.
std::filesystem::path DepthCalibrationPath() {
  return KernelState::shared()->emulator()->storage_root() /
         "kinect_depth_calibration.bin";
}
}  // namespace

bool NUIController::ReadDepthCalibration(
    std::array<uint8_t, kDepthCalibrationBytes>& data) {
  FILE* file = xe::filesystem::OpenFile(DepthCalibrationPath(), "rb");
  if (!file) {
    return false;
  }
  const bool read = std::fread(data.data(), 1, data.size(), file) == data.size();
  std::fclose(file);
  return read;
}

bool NUIController::WriteDepthCalibration(
    const std::array<uint8_t, kDepthCalibrationBytes>& data) {
  FILE* file = xe::filesystem::OpenFile(DepthCalibrationPath(), "wb");
  if (!file) {
    return false;
  }
  const bool written =
      std::fwrite(data.data(), 1, data.size(), file) == data.size();
  std::fclose(file);
  return written;
}

std::array<uint32_t, 2> NUIController::GetLoadedDepthCalibration() {
  // Dash.FieldCalibration's block: the scale A at +0 and the offset B at +4,
  // big-endian floats.
  std::array<uint32_t, 2> words = {0x3F800000, 0};
  std::array<uint8_t, kDepthCalibrationBytes> data;
  if (ReadDepthCalibration(data)) {
    words[0] = xe::load_and_swap<uint32_t>(data.data());
    words[1] = xe::load_and_swap<uint32_t>(data.data() + 4);
  }
  return words;
}

NuiSkeletonSnapshot NUIController::GetSkeletonSnapshot() const {
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return snapshot_;
}

void NUIController::SetSkeletonSnapshot(const NuiSkeletonSnapshot& snapshot) {
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  snapshot_ = snapshot;
}

}  // namespace kernel
}  // namespace xe

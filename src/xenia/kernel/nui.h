/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2022 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_KERNEL_NUI_H_
#define XENIA_KERNEL_NUI_H_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "xenia/base/byte_order.h"

namespace xe {
namespace kernel {

namespace kinect {
class KinectAudio;
class KinectDevice;
}  // namespace kinect

// The latest skeleton frame a title published (XamApp 0x2B002), for the
// emulator's Kinect Tuner.
struct NuiSkeletonSnapshot {
  static constexpr uint32_t kSkeletonCount = 6;
  static constexpr uint32_t kJointCount = 20;
  enum class TrackingState : uint32_t {
    kNotTracked = 0,
    kPositionOnly = 1,
    kTracked = 2,
  };
  struct Skeleton {
    TrackingState state = TrackingState::kNotTracked;
    uint32_t tracking_id = 0;
    std::array<float, 3> position = {};
    std::array<std::array<float, 3>, kJointCount> joints = {};
    std::array<uint32_t, kJointCount> joint_states = {};
  };
  uint64_t frame_number = 0;
  std::array<float, 4> floor_plane = {};
  std::array<Skeleton, kSkeletonCount> skeletons;
};

// What xam keeps for the Kinect: the title's NUI runtime registers itself and
// its engines with it through XamApp messages (0x2B0xx, 0x2C0xx, 0x2A0xx), the
// system HUD watches the frames it publishes, and the sensor sits behind it.
class NUIController {
 public:
  NUIController();
  ~NUIController();

  // The sensor, opened on first use, and its microphones as titles read them.
  kinect::KinectDevice* device();
  kinect::KinectAudio* audio();
  // Whether the console has a Kinect set up and switched on: the
  // KinectInitialized retail flag, without KinectDisabled.
  bool NuiInitialized();
  // Whether a sensor is there to use (NuiInitialized and connected).
  bool IsDeviceReady();

  // XamApp 0x2B003 (title initialize) and 0x2B004 (title shutdown), version 5
  // of the runtime: its xam and XTL versions and its flags. Bit 31 of the
  // flags leaves the system HUD off.
  static constexpr uint64_t kRuntimeVersion = 5;
  void InitializeTitle(uint64_t xam_version, uint64_t xtl_version,
                       uint32_t flags);
  void ShutdownTitle();
  uint32_t GetInitFlags() const { return init_flags_; }
  uint64_t GetNUIVersion(uint32_t index) const {
    return index < 2 ? nui_versions_[index].load() : 0;
  }
  // XamNuiHudIsEnabled: the HUD exists while a title runs with it.
  bool IsHudEnabled() const { return hud_enabled_; }

  // The player the system HUD engaged (hud+0x118 on the console).
  uint32_t GetEngagedTrackingId() const { return engaged_tracking_id_; }
  void SetEngagedTrackingId(uint32_t tracking_id) {
    engaged_tracking_id_ = tracking_id;
  }
  // The checks the system's Kinect screens make first (0x816C1808): the HUD
  // and a sensor, and the tracking ID becomes the engaged player.
  uint32_t NuiHudCheck(uint32_t tracking_id);

  // Callbacks the title registers. The tilt engine (0x2B007) receives the
  // system's tilt requests; the status listener (0x2B00D) receives the tilt
  // engine's reports; the frame observer (0x2B00E) gets each skeleton frame
  // before the HUD; the identity engine (0x2C009) answers Kinect ID requests.
  uint32_t tilt_engine() const { return tilt_engine_; }
  void set_tilt_engine(uint32_t callback) { tilt_engine_ = callback; }
  uint32_t tilt_status_listener() const { return tilt_status_listener_; }
  void set_tilt_status_listener(uint32_t callback) {
    tilt_status_listener_ = callback;
  }
  uint32_t frame_observer() const { return frame_observer_; }
  void set_frame_observer(uint32_t callback) { frame_observer_ = callback; }
  uint32_t identity_engine() const { return identity_engine_; }
  void set_identity_engine(uint32_t callback) { identity_engine_ = callback; }

  // Users and their Kinect ID enrollments (xam's user table, +0x30 bound and
  // +0x34 the enrollment index).
  static constexpr uint32_t kUserCount = 4;
  static constexpr uint32_t kNoEnrollment = 0xFFFFFFFE;
  uint32_t GetUserEnrollment(uint32_t user_index) const;
  void SetUserEnrollment(uint32_t user_index, uint32_t enrollment_index);
  // The user bound to an enrollment, or kUserCount.
  uint32_t FindEnrolledUser(uint32_t enrollment_index) const;

  // The Kinect ID data titles hand xam for a profile (0x2A007) and read back
  // (0x2A008): {version, data}. kBiometricNone is a profile without any.
  static constexpr uint32_t kBiometricNone = 0x80000000;
  struct BiometricData {
    uint32_t version = kBiometricNone;
    std::vector<uint8_t> data;
  };
  BiometricData GetCachedBiometricData(uint64_t xuid);
  void SetCachedBiometricData(uint64_t xuid, BiometricData data);

  bool HasCachedBiometricData(uint64_t xuid);

  // Whose the system's HUD and Guide are (XamHudSetUserIndex), 0xFF for
  // nobody's.
  static constexpr uint32_t kNoHudUser = 0xFF;
  uint32_t GetHudUserIndex() const { return hud_user_index_; }
  void SetHudUserIndex(uint32_t user_index) { hud_user_index_ = user_index; }

  // XamNuiIdentityGetSessionId: random, made on first use.
  uint64_t GetSessionId();

  // The camera's tilt in degrees as the system sets it, after the calibration
  // offset; without a motor, where it would be.
  static constexpr int32_t kElevationMinimum = -27;
  static constexpr int32_t kElevationMaximum = 27;
  int32_t GetElevationAngle() const { return elevation_angle_; }
  // Moves the motor; false without one.
  bool SetElevationAngle(int32_t degrees);
  bool StopElevation();
  // The floor plane the title found last (XamNuiCameraRememberFloor).
  std::array<float, 4> GetRememberedFloor() const;
  void SetRememberedFloor(const std::array<float, 4>& plane);
  // Flags titles set on the tilt controller (XamNuiCameraSetFlags).
  uint32_t GetCameraFlags() const { return camera_flags_; }
  void SetCameraFlags(uint32_t flags) { camera_flags_ = flags; }

  // The depth calibration a title stored (XamNuiStoreDepthCalibration, 0x30
  // bytes in the storage root) and, from its first two words, the scale and
  // offset the sensor runs with (PsCam code 44): 1.0 and 0 without one.
  static constexpr size_t kDepthCalibrationBytes = 0x30;
  bool ReadDepthCalibration(std::array<uint8_t, kDepthCalibrationBytes>& data);
  bool WriteDepthCalibration(
      const std::array<uint8_t, kDepthCalibrationBytes>& data);
  std::array<uint32_t, 2> GetLoadedDepthCalibration();

  NuiSkeletonSnapshot GetSkeletonSnapshot() const;
  void SetSkeletonSnapshot(const NuiSkeletonSnapshot& snapshot);

 private:
  std::once_flag device_once_;
  std::unique_ptr<kinect::KinectDevice> device_;
  std::once_flag audio_once_;
  std::unique_ptr<kinect::KinectAudio> audio_;

  std::atomic<uint32_t> init_flags_ = 0;
  std::array<std::atomic<uint64_t>, 2> nui_versions_ = {};
  std::atomic<bool> hud_enabled_ = false;
  std::atomic<uint32_t> engaged_tracking_id_ = 0;

  std::atomic<uint32_t> tilt_engine_ = 0;
  std::atomic<uint32_t> tilt_status_listener_ = 0;
  std::atomic<uint32_t> frame_observer_ = 0;
  std::atomic<uint32_t> identity_engine_ = 0;

  std::array<std::atomic<uint32_t>, kUserCount> user_enrollments_;

  std::mutex biometric_mutex_;
  std::unordered_map<uint64_t, BiometricData> biometric_cache_;

  std::atomic<uint32_t> hud_user_index_ = kNoHudUser;

  std::once_flag session_once_;
  uint64_t session_id_ = 0;

  std::mutex motor_mutex_;
  std::atomic<int32_t> elevation_angle_ = 0;
  std::atomic<uint32_t> camera_flags_ = 0;
  mutable std::mutex floor_mutex_;
  std::array<float, 4> remembered_floor_ = {};

  mutable std::mutex snapshot_mutex_;
  NuiSkeletonSnapshot snapshot_;
};

}  // namespace kernel
}  // namespace xe

#endif  // XENIA_KERNEL_NUI_H_

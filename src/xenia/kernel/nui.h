/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2020 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_KERNEL_NUI_H_
#define XENIA_KERNEL_NUI_H_

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "xenia/base/byte_order.h"

namespace xe {
namespace hid {
namespace kinect {
class UsbAudio;
}  // namespace kinect
}  // namespace hid

namespace kernel {

class NuiAudio;

// What the title's NUI runtime last reported of the skeletons it tracks, for
// the host UI. Positions are in skeleton space: metres from the sensor, x to
// the sensor's left, y up, z away from it.
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
    // The whole body's position, for skeletons tracked by position only.
    std::array<float, 3> position = {};
    std::array<std::array<float, 3>, kJointCount> joints = {};
    // 0 not tracked, 1 inferred, 2 tracked.
    std::array<uint32_t, kJointCount> joint_states = {};
  };
  // Number of skeleton frames reported so far, 0 if none yet.
  uint64_t frame_number = 0;
  std::array<Skeleton, kSkeletonCount> skeletons = {};
};

class NUIController {
  /* NUI Notes:
     - init_flags_:
       - set by 0x2B003
       - set to 0 by title shutdown, alongside version_id
       - known values:
          - 0x40000000
          - 0x200
     - nui_versions_:
       - set by 0x2B003 (XAM and XTL versions)
       - set to 0 by title shutdown, alongside init_flags_
       - must be XAM vesion 5
  */
 public:
  NUIController();
  ~NUIController();

  bool NuiInitialized();
  uint32_t GetNUIDataPtr();
  char GetUnknown2();
  void SetTiltCallback(uint32_t callback);
  uint32_t GetEngagedTrackingId();
  void SetEngagedTrackingId(uint32_t tracking_id);
  uint32_t GetInitFlags();
  void SetInitFlags(uint32_t flags);
  uint64_t GetNUIVersion(uint32_t index);
  void SetNUIVersion(uint64_t version, uint32_t index);
  uint64_t GetSessionId();
  void SetSessionId(uint64_t id);
  uint32_t NuiHudCheck(uint64_t tracking_id);

  // Camera elevation in degrees, as the Kinect tilt motor would report it.
  // Without the motor device the angle is the last one a title set.
  static constexpr int32_t kElevationMinimum = -27;
  static constexpr int32_t kElevationMaximum = 27;
  int32_t GetElevationAngle() const { return elevation_angle_; }
  void SetElevationAngle(int32_t angle);
  // Stops the motor where it is.
  void StopElevationMovement();
  // The tilt controller flags titles set with XamNuiCameraSetFlags.
  uint32_t GetCameraFlags() const { return camera_flags_; }
  void SetCameraFlags(uint32_t flags) { camera_flags_ = flags; }
  // The elevation the console is calibrated to (kinect_elevation), which
  // titles read through XCONFIG_CONSOLE_NUI; Set persists it in the config.
  int32_t GetCalibratedElevation() const;
  void SetCalibratedElevation(int32_t angle);

  // The sensor's tilt motor, LED and accelerometer, when the host has them.
  // Lights the LED green while it's attached; the motor sets it back to
  // blinking when closed.
  void SetMotor(std::unique_ptr<hid::kinect::UsbAudio> motor);
  struct TiltState {
    // Accelerometer counts, about 819 per g: x, y (up when level), z.
    std::array<int32_t, 3> gravity;
    // X_TILT_STATUS_FLAGS: whether the motor is moving.
    uint32_t flags;
  };
  // Reads the motor; false without one.
  bool GetTiltState(TiltState& state);

  // The sensor's serial number, read from it the first time; empty without
  // a sensor.
  const std::string& GetSerialNumber();

  // Kinect ID data a title keeps for a profile: the faces it enrolled, in its
  // own format. xam holds a copy that titles fill and read through XMsg
  // 0x2A007 and 0x2A008 and that lasts until the console is turned off;
  // version 0 means xam has none yet, kBiometricNone that the profile has
  // none. The profile's own copy, which xam keeps on Xbox Live, is stored
  // with the profile here.
  static constexpr uint32_t kBiometricNone = 0x80000000;
  struct BiometricData {
    uint32_t version = 0;
    std::vector<uint8_t> data;
  };
  BiometricData GetCachedBiometricData(uint64_t xuid);
  void SetCachedBiometricData(uint64_t xuid, BiometricData data);
  // False when the profile has none.
  bool ReadStoredBiometricData(uint64_t xuid, BiometricData& data);
  // Empty data deletes the profile's copy.
  bool WriteStoredBiometricData(uint64_t xuid, const BiometricData& data);

  // The microphone array of the attached audio device.
  NuiAudio* audio() const { return audio_.get(); }

  // The skeleton frame the title's runtime last sent to xam. Also drives the
  // automatic tilt.
  void SetSkeletonSnapshot(const NuiSkeletonSnapshot& snapshot);
  NuiSkeletonSnapshot GetSkeletonSnapshot() const;

 private:
  mutable std::mutex skeleton_mutex_;
  NuiSkeletonSnapshot skeleton_snapshot_;
  uint32_t nui_data_ptr = 0x1;  // Meant to be a ptr to a larger structure
  char nui_unknown_2 = 0x1;     // exists at 0x50 within nui data structure
  uint32_t engaged_tracking_id =
      0x0;  // exists at 0x118 within nui data structure
  uint64_t session_id = 0x0;
  uint32_t tilt_callback_ = 0x0;
  uint32_t init_flags_ = 0x0;
  // Tilts the sensor so the closest tracked player fits the depth image, as
  // a console calibrated in its Kinect Tuner would be (kinect_auto_tilt).
  void UpdateAutoTilt(const NuiSkeletonSnapshot& snapshot);
  bool MoveMotor(int32_t angle);

  int32_t elevation_angle_ = 0;
  uint32_t camera_flags_ = 0;
  std::mutex biometric_mutex_;
  std::unordered_map<uint64_t, BiometricData> cached_biometric_data_;
  std::once_flag serial_number_once_;
  std::string serial_number_;
  // Last time anything tilted the sensor; the automatic tilt waits for the
  // motor and the depth image to settle after it.
  std::chrono::steady_clock::time_point last_tilt_time_;
  // Frames in a row the player has been out of the picture.
  uint32_t auto_tilt_out_frames_ = 0;
  bool auto_tilted_ = false;
  // The player fit since the last tilt; small moves out of the picture after
  // that (a jump, a squat) don't tilt again.
  bool auto_tilt_framed_ = false;
  std::mutex motor_mutex_;
  std::unique_ptr<hid::kinect::UsbAudio> motor_;
  // Reads motor_, so it's declared after it to go first.
  std::unique_ptr<NuiAudio> audio_;
  // The motor is read at most every kTiltStatePeriod; titles poll it often.
  TiltState tilt_state_ = {};
  std::chrono::steady_clock::time_point tilt_state_time_;
  uint64_t nui_versions_[2] = {};
};

enum X_TILT_STATUS_FLAGS : uint32_t {
  // 0x1 and 0x2 moving
  TILT_STATUS_UNK1 = 0x1,
  TILT_STATUS_UNK2 = 0x2,
  TILT_STATUS_STALLED = 0x4,
  TILT_STATUS_ERROR = 0x8,
};

struct X_NUI_TILT_STATUS {
  be<uint32_t> buffer_size;
  be<uint32_t> signature;  // 'XtSs'
  be<uint32_t> unknown_08[6];
  // Gravity direction.
  be<int32_t> gravity_long_avg[3];
  be<X_TILT_STATUS_FLAGS> flags;
  be<uint32_t> unknown_30[8];
};
static_assert_size(X_NUI_TILT_STATUS, 0x50);

}  // namespace kernel
}  // namespace xe

#endif  // XENIA_KERNEL_NUI_H_

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
#include <cstdint>
#include <mutex>

#include "xenia/base/byte_order.h"

namespace xe {
namespace kernel {

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
  ~NUIController() = default;

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

  // The skeleton frame the title's runtime last sent to xam.
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
  int32_t elevation_angle_ = 0;
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

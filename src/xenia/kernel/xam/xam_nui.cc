/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2025 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include <algorithm>
#include <chrono>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <mutex>
#include <string>
#include <utility>

#include "xenia/base/cvar.h"
#include "xenia/base/filesystem.h"
#include "xenia/base/logging.h"
#include "xenia/cpu/processor.h"
#include "xenia/kernel/xthread.h"
#include "xenia/emulator.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/nui.h"
#include "xenia/kernel/nui_audio.h"
#include "xenia/kernel/util/shim_utils.h"
#include "xenia/kernel/xam/user_profile.h"
#include "xenia/kernel/xam/user_settings.h"
#include "xenia/kernel/xam/user_tracker.h"
#include "xenia/kernel/xam/xam_private.h"
#include "xenia/kernel/xam/xam_ui.h"
#include "xenia/xbox.h"

DECLARE_bool(use_50Hz_mode);

DEFINE_bool(kinect_guide_gesture, true,
            "Open the Kinect Guide when a player holds the guide gesture in "
            "titles that leave it to the system.",
            "HID");

namespace xe {
namespace kernel {
namespace xam {
// https://web.cs.ucdavis.edu/~okreylos/ResDev/Kinect/MainPage.html

struct X_NUI_DEVICE_STATUS {
  /* Notes:
     - for one side func of XamNuiGetDeviceStatus
       - if some data address is less than zero then tilt_status = it
       - else another func is called and its return can set
     tilt_status = c0051200 or some value involving DetroitDeviceRequest
       - next PsCamDeviceRequest is called and if its return is less than zero
     then camera_status = return of PsCamDeviceRequest
       - else it equals an unknown local_1c
       - finally McaDeviceRequest is called and if its return is less than zero
     then mic_status = return of McaDeviceRequest
       - else it equals an unknown local_14
     - status_bits can be set to status_bits | 0x44 or | 0x40
  */
  xe::be<uint32_t> camera_status;
  xe::be<uint32_t> tilt_status;
  xe::be<uint32_t> mic_status;
  xe::be<uint32_t> status_bits;
};
static_assert_size(X_NUI_DEVICE_STATUS, 0x10);

// Get
dword_result_t XamNuiGetDeviceStatus_entry(
    pointer_t<X_NUI_DEVICE_STATUS> status_ptr) {
  /* Notes:
     - it does return a value that is not always used
     - returns values are X_ERROR_SUCCESS, 0xC0050006, and others
     - 1) On func start *status_ptr = 0, status_ptr->tilt_status = 0,
     status_ptr->mic_status = 0, and status_ptr->status_bits = 0
     - 2) calls XamXStudioRequest(6,&var <- = 0);
     - if return is greater than -1 && var & 0x80000000 != 0 then set
     status_ptr->tilt_status = 0xC000009D, status_ptr->mic_status = 0xC000009D,
     and status_ptr->status_bits = 0x20
     - lots of branching functions after
  */

  status_ptr.Zero();

  const bool kinect_initialized = kernel_state()->nui()->NuiInitialized();

  // Bit 0: the sensor is set up. Bit 1: its microphone array is there, which
  // titles check before starting their NUI audio.
  uint32_t status_bits = kinect_initialized ? 0x1 : 0x0;
  if (kinect_initialized &&
      kernel_state()->nui()->audio()->microphones_available()) {
    status_bits |= 0x2;
  }
  status_ptr->status_bits = status_bits;
  return kinect_initialized ? X_ERROR_SUCCESS : 0xC0050006;
}
DECLARE_XAM_EXPORT1(XamNuiGetDeviceStatus, kNone, kStub);

// xam passes the request to a handler that Kinect Studio, the development kit's
// recording tool, installs; without one, as on a retail console, it fails.
// Titles ask every frame.
dword_result_t XamXStudioRequest_entry(dword_t request_type,
                                       lpdword_t result_ptr) {
  return X_E_FAIL;
}
DECLARE_XAM_EXPORT1(XamXStudioRequest, kNone, kImplemented);

// Profiles and the Kinect ID enrollments bound to them (xam's user table at
// 0x81A82FF4, +0x30 bound and +0x34 the enrollment index).
dword_result_t XamUserNuiGetUserIndex_entry(dword_t enrollment_index,
                                            lpdword_t user_index) {
  if (enrollment_index >= 8 || !user_index) {
    return X_E_INVALIDARG;
  }
  auto nui = kernel_state()->nui();
  for (uint32_t i = 0; i < XUserMaxUserCount; ++i) {
    if (nui->GetUserEnrollment(i) == enrollment_index) {
      *user_index = i;
      return X_E_SUCCESS;
    }
  }
  return X_E_NO_SUCH_USER;
}
DECLARE_XAM_EXPORT1(XamUserNuiGetUserIndex, kNone, kImplemented);

dword_result_t XamUserNuiGetUserIndexForSignin_entry(lpdword_t user_index) {
  if (!user_index) {
    return X_E_INVALIDARG;
  }

  for (uint32_t i = 0; i < XUserMaxUserCount; i++) {
    auto profile = kernel_state()->xam_state()->GetUserProfile(i);
    if (profile) {
      *user_index = i;
      return X_E_SUCCESS;
    }
  }

  return X_E_FAIL;
}
DECLARE_XAM_EXPORT1(XamUserNuiGetUserIndexForSignin, kNone, kImplemented);

// The first signed-in profile without a Kinect ID enrollment bound to it.
dword_result_t XamUserNuiGetUserIndexForBind_entry(lpdword_t user_index) {
  if (!user_index) {
    return X_E_INVALIDARG;
  }
  auto nui = kernel_state()->nui();
  for (uint32_t i = 0; i < XUserMaxUserCount; ++i) {
    if (kernel_state()->xam_state()->IsUserSignedIn(i) &&
        nui->GetUserEnrollment(i) == NUIController::kNoEnrollment) {
      *user_index = i;
      return X_E_SUCCESS;
    }
  }
  return X_E_FAIL;
}
DECLARE_XAM_EXPORT1(XamUserNuiGetUserIndexForBind, kNone, kImplemented);

dword_result_t XamUserNuiBind_entry(dword_t enrollment_index,
                                    dword_t user_index) {
  if (enrollment_index >= 8 || user_index >= XUserMaxUserCount) {
    return X_E_INVALIDARG;
  }
  XELOGI("Kinect: enrollment {} bound to user {}", enrollment_index.value(),
         user_index.value());
  kernel_state()->nui()->SetUserEnrollment(user_index, enrollment_index);
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamUserNuiBind, kNone, kImplemented);

dword_result_t XamUserNuiUnbind_entry(dword_t user_index) {
  if (user_index >= XUserMaxUserCount) {
    return X_E_INVALIDARG;
  }
  auto nui = kernel_state()->nui();
  if (nui->GetUserEnrollment(user_index) == NUIController::kNoEnrollment) {
    return X_E_NO_SUCH_USER;
  }
  nui->SetUserEnrollment(user_index, NUIController::kNoEnrollment);
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamUserNuiUnbind, kNone, kImplemented);

struct X_NUI_DEPTH_CALIBRATION {
  uint8_t data[0x30];
};
static_assert_size(X_NUI_DEPTH_CALIBRATION, 0x30);

// The console keeps the depth calibration titles store in a system setting
// (0x816F9990 and 0x816F9A50); here it's a file in the storage root. Both
// answer with HRESULT_FROM_NT, so success is 0x10000000.
static std::filesystem::path GetDepthCalibrationPath() {
  return kernel_state()->emulator()->storage_root() /
         "kinect_depth_calibration.bin";
}

dword_result_t XamNuiGetDepthCalibration_entry(
    pointer_t<X_NUI_DEPTH_CALIBRATION> depth_data) {
  if (!depth_data) {
    return X_E_INVALIDARG;
  }
  // always cleared before new data written
  std::memset(depth_data, 0, sizeof(X_NUI_DEPTH_CALIBRATION));
  FILE* file = xe::filesystem::OpenFile(GetDepthCalibrationPath(), "rb");
  if (!file) {
    return X_STATUS_NO_SUCH_FILE | X_FACILITY_NT_BIT;
  }
  X_NUI_DEPTH_CALIBRATION calibration;
  const bool read = std::fread(&calibration, 1, sizeof(calibration), file) ==
                    sizeof(calibration);
  std::fclose(file);
  if (!read) {
    return X_STATUS_NO_SUCH_FILE | X_FACILITY_NT_BIT;
  }
  std::memcpy(depth_data, &calibration, sizeof(calibration));
  return X_STATUS_SUCCESS | X_FACILITY_NT_BIT;
}
DECLARE_XAM_EXPORT1(XamNuiGetDepthCalibration, kNone, kImplemented);

// The color camera's automatic exposure (PsCam code 46, 0x816F9B38), as
// 4D53090F reads it at 0x82F78E40: selector 0, a 4-byte value; selector 1,
// entries of {scene brightness, frames per second, exposure in seconds}, the
// exposures multiples of the mains period so lights don't flicker. These are
// the values titles fall back to without it, for 60 Hz mains, or 50 Hz with
// use_50Hz_mode.
struct X_NUI_TRUE_COLOR_EXPOSURE {
  xe::be<float> brightness;
  xe::be<float> frame_rate;
  xe::be<float> exposure;
};
static_assert_size(X_NUI_TRUE_COLOR_EXPOSURE, 0xC);

dword_result_t XamNuiGetTrueColorInfo_entry(dword_t unk, dword_t selector,
                                            lpvoid_t buffer, dword_t size,
                                            lpdword_t returned_size) {
  struct Exposure {
    float brightness, frame_rate, exposure;
  };
  static constexpr Exposure k60Hz[] = {
      {49.1f, 7.5f, 0.1333f},   {144.5f, 10.0f, 0.1f},
      {362.2f, 15.0f, 0.0667f}, {482.9f, 30.0f, 0.0333f},
      {724.3f, 30.0f, 0.025f},  {1448.7f, 30.0f, 0.0167f},
      {4102.6f, 30.0f, 0.0083f}};
  static constexpr Exposure k50Hz[] = {
      {49.1f, 7.5f, 0.13f},    {160.5f, 10.0f, 0.1f},
      {402.4f, 15.0f, 0.06f},  {603.6f, 30.0f, 0.03f},
      {1207.2f, 30.0f, 0.02f}, {3418.8f, 30.0f, 0.01f}};
  if (!buffer || !returned_size) {
    return X_E_INVALIDARG;
  }
  switch (selector) {
    case 0:
      if (size < 4) {
        return X_E_INVALIDARG;
      }
      *buffer.as<xe::be<uint32_t>*>() = 0x4208C083;
      *returned_size = 4;
      return X_E_SUCCESS;
    case 1: {
      const Exposure* table = cvars::use_50Hz_mode ? k50Hz : k60Hz;
      const uint32_t count = cvars::use_50Hz_mode ? uint32_t(std::size(k50Hz))
                                                  : uint32_t(std::size(k60Hz));
      if (size < count * sizeof(X_NUI_TRUE_COLOR_EXPOSURE)) {
        return X_E_INVALIDARG;
      }
      auto out = buffer.as<X_NUI_TRUE_COLOR_EXPOSURE*>();
      for (uint32_t i = 0; i < count; ++i) {
        out[i].brightness = table[i].brightness;
        out[i].frame_rate = table[i].frame_rate;
        out[i].exposure = table[i].exposure;
      }
      *returned_size = count * sizeof(X_NUI_TRUE_COLOR_EXPOSURE);
      return X_E_SUCCESS;
    }
    default:
      return X_E_INVALIDARG;
  }
}
DECLARE_XAM_EXPORT1(XamNuiGetTrueColorInfo, kNone, kSketchy);

dword_result_t XamNuiStoreDepthCalibration_entry(
    pointer_t<X_NUI_DEPTH_CALIBRATION> depth_data) {
  if (!depth_data) {
    return X_E_INVALIDARG;
  }
  FILE* file = xe::filesystem::OpenFile(GetDepthCalibrationPath(), "wb");
  if (!file) {
    return X_STATUS_UNSUCCESSFUL | X_FACILITY_NT_BIT;
  }
  const bool written =
      std::fwrite(depth_data, 1, sizeof(X_NUI_DEPTH_CALIBRATION), file) ==
      sizeof(X_NUI_DEPTH_CALIBRATION);
  std::fclose(file);
  return (written ? X_STATUS_SUCCESS : X_STATUS_UNSUCCESSFUL) |
         X_FACILITY_NT_BIT;
}
DECLARE_XAM_EXPORT1(XamNuiStoreDepthCalibration, kNone, kImplemented);

// The depth calibration the sensor runs with (PsCam code 44, 0x816F9A48 and
// 0x817FCB40): the scale and the offset.
dword_result_t XamNuiGetLoadedDepthCalibration_entry(lpdword_t calibration) {
  if (!calibration) {
    return X_E_INVALIDARG;
  }
  if (!kernel_state()->nui()->NuiInitialized()) {
    return X_STATUS_DEVICE_NOT_CONNECTED | X_FACILITY_NT_BIT;
  }
  const auto words = kernel_state()->nui()->GetLoadedDepthCalibration();
  calibration[0] = words[0];
  calibration[1] = words[1];
  return X_STATUS_SUCCESS | X_FACILITY_NT_BIT;
}
DECLARE_XAM_EXPORT1(XamNuiGetLoadedDepthCalibration, kNone, kImplemented);

// The color camera's intrinsics, PsCam code 46 with selector 0x100
// (0x816F9B90): fx, fy, cx and cy in pixels, in the order Dash.FieldCalibration
// builds its camera matrix from (0x9BD22C38). These are the Kinect's nominal
// ones at 640x480.
dword_result_t XamNuiGetCameraIntrinsics_entry(lpvoid_t buffer, dword_t size,
                                               lpdword_t returned_size) {
  if (!buffer || !returned_size || size < 4 * sizeof(float)) {
    return X_E_INVALIDARG;
  }
  if (!kernel_state()->nui()->NuiInitialized()) {
    return X_E_DEVICE_NOT_CONNECTED;
  }
  auto out = buffer.as<xe::be<float>*>();
  out[0] = 531.15f;
  out[1] = 531.15f;
  out[2] = 320.0f;
  out[3] = 240.0f;
  *returned_size = 4 * sizeof(float);
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiGetCameraIntrinsics, kNone, kSketchy);

// Skeleton
// NUI_SKELETON_FRAME as xam reads it (big-endian): six NUI_SKELETON_DATA from
// +0x30, each with its tracking state, tracking ID, enrollment index, user
// index and position, and the two tracking IDs XamNuiPlayerEngagementUpdate
// picks for the tracker at +0xACC.
constexpr uint32_t kSkeletonFrameSkeletons = 0x30;
constexpr uint32_t kSkeletonDataSize = 0x1C0;
constexpr uint32_t kSkeletonCount = 6;
constexpr uint32_t kSkeletonFrameChosenIds = 0xACC;
constexpr uint32_t kSkeletonTracked = 2;

struct X_NUI_SKELETON_DATA_HEADER {
  xe::be<uint32_t> tracking_state;
  xe::be<uint32_t> tracking_id;
  xe::be<uint32_t> enrollment_index;
  xe::be<uint32_t> user_index;
  xe::be<float> position[4];
};

static X_NUI_SKELETON_DATA_HEADER* GetSkeleton(uint8_t* frame,
                                               uint32_t index) {
  return reinterpret_cast<X_NUI_SKELETON_DATA_HEADER*>(
      frame + kSkeletonFrameSkeletons + index * kSkeletonDataSize);
}

// The tracked skeleton among the two XamNuiPlayerEngagementUpdate chose, in
// the order it chose them, as xam does.
qword_result_t XamNuiSkeletonGetBestSkeletonIndex_entry(
    lpvoid_t skeleton_frame) {
  if (!skeleton_frame) {
    return uint64_t(-1);
  }
  uint8_t* frame = skeleton_frame.as<uint8_t*>();
  for (uint32_t chosen = 0; chosen < 2; ++chosen) {
    const uint32_t id = xe::load_and_swap<uint32_t>(
        frame + kSkeletonFrameChosenIds + chosen * 4);
    if (!id) {
      continue;
    }
    for (uint32_t i = 0; i < kSkeletonCount; ++i) {
      const auto* skeleton = GetSkeleton(frame, i);
      if (skeleton->tracking_id == id) {
        if (skeleton->tracking_state == kSkeletonTracked) {
          return i;
        }
        break;
      }
    }
  }
  return uint64_t(-1);
}
DECLARE_XAM_EXPORT1(XamNuiSkeletonGetBestSkeletonIndex, kNone, kImplemented);

// XamNuiSkeletonScoreUpdate keeps a record of how well each of up to two
// tracked players stands for the sensor (0x8198B628 and 0x8198B390): the
// warnings behind "move to the center", "step back" and "face the sensor".
constexpr uint32_t kSkeletonFrameNormalToGravity = 0x20;
constexpr uint32_t kSkeletonJointPositions = 0x20;
constexpr uint32_t kSkeletonJointStates = 0x160;
constexpr uint32_t kJointShoulderCenter = 2;
constexpr uint32_t kJointHead = 3;
constexpr uint32_t kJointShoulderLeft = 4;
constexpr uint32_t kJointHandLeft = 7;
constexpr uint32_t kJointShoulderRight = 8;
constexpr uint32_t kJointHandRight = 11;
constexpr uint32_t kScoreRecords = 2;
// Focal length of the 320x240 depth image, in pixels; its center is 160, 120.
constexpr float kDepthFocalLength = 285.63f;

struct X_NUI_SKELETON_SCORE {
  xe::be<uint32_t> tracking_state;
  // tracking_state, or 1 when any warning below is set.
  xe::be<uint32_t> state;
  // Shoulders out of tracking, and both of them and the head.
  xe::be<float> shoulders_lost;
  xe::be<uint32_t> upper_body_lost;
  // From -1 at the image's left to 1 at its right, and near either edge.
  xe::be<float> horizontal_position;
  xe::be<uint32_t> near_side_edge;
  // Meters from the sensor, and closer than 1 m.
  xe::be<float> distance;
  xe::be<uint32_t> too_close;
  // Degrees the shoulders turn away from facing the sensor.
  xe::be<float> turn_degrees;
  xe::be<uint32_t> turned;
  // 1.0 once a hand has been tracked, and none tracked yet.
  xe::be<float> hand_tracked;
  xe::be<uint32_t> no_hands;
  xe::be<uint32_t> tracking_id;
  xe::be<uint32_t> skeleton_index;
  xe::be<uint32_t> new_player;
};
static_assert_size(X_NUI_SKELETON_SCORE, 0x3C);

struct X_NUI_SKELETON_SCORES {
  X_NUI_SKELETON_SCORE players[kScoreRecords];
  xe::be<uint32_t> tracked_count;
};

static void ScoreSkeleton(const uint8_t* skeleton, const float* up,
                          X_NUI_SKELETON_SCORE& score) {
  auto joint_state = [skeleton](uint32_t joint) {
    return xe::load_and_swap<uint32_t>(skeleton + kSkeletonJointStates +
                                       joint * 4);
  };
  auto joint = [skeleton](uint32_t joint, uint32_t axis) {
    return xe::load_and_swap<float>(skeleton + kSkeletonJointPositions +
                                    joint * 16 + axis * 4);
  };
  const auto* header =
      reinterpret_cast<const X_NUI_SKELETON_DATA_HEADER*>(skeleton);
  score.tracking_state = header->tracking_state;
  score.state = header->tracking_state;
  score.tracking_id = 0;
  score.skeleton_index = 0;
  score.new_player = 0;
  if (header->tracking_state != kSkeletonTracked) {
    return;
  }
  uint32_t shoulders_lost = 0;
  for (uint32_t shoulder : {kJointShoulderLeft, kJointShoulderRight}) {
    if (joint_state(shoulder) != kSkeletonTracked) {
      ++shoulders_lost;
    }
  }
  score.shoulders_lost = float(shoulders_lost);
  score.upper_body_lost =
      shoulders_lost >= 2 && joint_state(kJointHead) != kSkeletonTracked;
  // The player's position projected into the depth image.
  const float x = header->position[0], z = header->position[2];
  const int32_t column = z != 0.0f ? int32_t(x * kDepthFocalLength / z + 160.0f)
                                   : 0;
  const float horizontal = float(160 - column) / 160.0f;
  score.horizontal_position = horizontal;
  score.near_side_edge = std::abs(horizontal) >= 0.7f;
  score.distance = z;
  score.too_close = z <= 1.0f;
  // From the shoulder center to the nearer shoulder, across the floor.
  const uint32_t shoulder =
      joint(kJointShoulderLeft, 2) <= joint(kJointShoulderRight, 2)
          ? kJointShoulderLeft
          : kJointShoulderRight;
  float d[3];
  for (uint32_t i = 0; i < 3; ++i) {
    d[i] = joint(shoulder, i) - joint(kJointShoulderCenter, i);
  }
  const float along_up = d[0] * up[0] + d[1] * up[1] + d[2] * up[2];
  for (uint32_t i = 0; i < 3; ++i) {
    d[i] -= along_up * up[i];
  }
  const float length = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
  const float cosine =
      std::clamp(length != 0.0f ? d[0] / length : d[0], -1.0f, 1.0f);
  score.turn_degrees = std::acos(cosine) * 57.2957764f;
  score.turned = 0;
  if (joint_state(kJointHandLeft) == kSkeletonTracked ||
      joint_state(kJointHandRight) == kSkeletonTracked) {
    score.hand_tracked = 1.0f;
  }
  score.no_hands = score.hand_tracked == 0.0f;
  if (score.upper_body_lost || score.near_side_edge || score.too_close ||
      score.turned || score.no_hands) {
    score.state = 1;
  }
}

static void UpdateSkeletonScores(const uint8_t* frame,
                                 X_NUI_SKELETON_SCORES& scores) {
  float up[3];
  for (uint32_t i = 0; i < 3; ++i) {
    up[i] = xe::load_and_swap<float>(frame + kSkeletonFrameNormalToGravity +
                                     i * 4);
  }
  // Tracked skeletons keep the record that has their tracking ID; the others
  // take the free records in order.
  uint32_t record_skeleton[kScoreRecords] = {UINT32_MAX, UINT32_MAX};
  uint32_t unmatched[kSkeletonCount];
  uint32_t unmatched_count = 0, tracked = 0;
  for (uint32_t i = 0; i < kSkeletonCount; ++i) {
    const auto* skeleton = GetSkeleton(const_cast<uint8_t*>(frame), i);
    if (skeleton->tracking_state != kSkeletonTracked) {
      continue;
    }
    ++tracked;
    uint32_t record = 0;
    while (record < kScoreRecords &&
           scores.players[record].tracking_id != skeleton->tracking_id) {
      ++record;
    }
    if (record < kScoreRecords) {
      record_skeleton[record] = i;
    } else {
      unmatched[unmatched_count++] = i;
    }
  }
  // A lone player that kept the second record moves to the first.
  if (tracked == 1 && record_skeleton[1] != UINT32_MAX) {
    scores.players[0] = scores.players[1];
    std::memset(&scores.players[1], 0, sizeof(scores.players[1]));
    record_skeleton[0] = record_skeleton[1];
    record_skeleton[1] = UINT32_MAX;
  }
  uint32_t next_unmatched = 0;
  for (uint32_t record = 0; record < kScoreRecords; ++record) {
    bool new_player = false;
    if (record_skeleton[record] == UINT32_MAX) {
      if (next_unmatched == unmatched_count) {
        continue;
      }
      record_skeleton[record] = unmatched[next_unmatched++];
      std::memset(&scores.players[record], 0, sizeof(scores.players[record]));
      new_player = true;
    }
    const uint32_t index = record_skeleton[record];
    const auto* skeleton = GetSkeleton(const_cast<uint8_t*>(frame), index);
    ScoreSkeleton(reinterpret_cast<const uint8_t*>(skeleton), up,
                  scores.players[record]);
    scores.players[record].tracking_id = skeleton->tracking_id;
    scores.players[record].skeleton_index = index;
    scores.players[record].new_player = new_player;
  }
  scores.tracked_count = tracked;
}

dword_result_t XamNuiSkeletonScoreUpdate_entry(lpvoid_t skeleton_frame,
                                               lpvoid_t scores_ptr) {
  if (!skeleton_frame || !scores_ptr) {
    return X_E_INVALIDARG;
  }
  UpdateSkeletonScores(skeleton_frame.as<uint8_t*>(),
                       *scores_ptr.as<X_NUI_SKELETON_SCORES*>());
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiSkeletonScoreUpdate, kNone, kImplemented);

// Calls a callback a title registered with xam with a message and a request
// in memory, as xam does on the caller's thread; 0x8000FFFF without one.
static X_HRESULT CallTitleCallback(uint32_t callback, uint32_t message,
                                   uint32_t* words, uint32_t count) {
  XThread* thread = XThread::GetCurrentThread();
  if (!callback || !thread) {
    return 0x8000FFFF;
  }
  auto memory = kernel_state()->memory();
  uint32_t buffer = 0;
  if (count) {
    buffer = memory->SystemHeapAlloc(count * 4);
    if (!buffer) {
      return X_E_FAIL;
    }
    auto guest = memory->TranslateVirtual<xe::be<uint32_t>*>(buffer);
    for (uint32_t i = 0; i < count; ++i) {
      guest[i] = words[i];
    }
  }
  uint64_t args[] = {message, buffer};
  const auto result = static_cast<X_HRESULT>(kernel_state()->processor()->Execute(
      thread->thread_state(), callback, args, xe::countof(args)));
  if (count) {
    auto guest = memory->TranslateVirtual<xe::be<uint32_t>*>(buffer);
    for (uint32_t i = 0; i < count; ++i) {
      words[i] = guest[i];
    }
    memory->SystemHeapFree(buffer);
  }
  return result;
}

/* XamNuiCamera Notes
   - most require message calls to xam in 0x0002Bxxx area
*/
dword_result_t XamNuiCameraTiltSetCallback_entry(dword_t callback) {
  kernel_state()->nui()->SetTiltCallback(callback);
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraTiltSetCallback, kNone, kImplemented);

dword_result_t XamNuiCameraTiltGetStatus_entry(
    pointer_t<X_NUI_TILT_STATUS> tilt_status) {
  /* Notes:
     - Used by XamNuiCameraElevationGetAngle, and XamNuiCameraSetFlags
     - if it returns anything greater than -1 then both above functions continue
     - Both funcs request 0x50 bytes with XtSs header
  */
  if (!tilt_status || tilt_status->buffer_size < sizeof(X_NUI_TILT_STATUS) ||
      tilt_status->signature != make_fourcc("XtSs")) {
    return X_E_FAIL;
  }
  if (!kernel_state()->nui()->NuiInitialized()) {
    return X_E_DEVICE_NOT_CONNECTED;
  }
  std::memset(tilt_status->unknown_08, 0, sizeof(tilt_status->unknown_08));
  NUIController::TiltState tilt_state;
  if (kernel_state()->nui()->GetTiltState(tilt_state)) {
    for (size_t i = 0; i < 3; ++i) {
      tilt_status->gravity_long_avg[i] = tilt_state.gravity[i];
    }
    tilt_status->flags = X_TILT_STATUS_FLAGS(tilt_state.flags);
  } else {
    // Without the motor, a stationary camera at the elevation the title last
    // set: gravity in the Kinect accelerometer units (1 g is about 819),
    // tilted around the X axis.
    constexpr double kAccelerometerOneG = 819.0;
    const double elevation =
        kernel_state()->nui()->GetElevationAngle() * M_PI / 180.0;
    tilt_status->gravity_long_avg[0] = 0;
    tilt_status->gravity_long_avg[1] = static_cast<int32_t>(
        std::lround(kAccelerometerOneG * std::cos(elevation)));
    tilt_status->gravity_long_avg[2] = static_cast<int32_t>(
        std::lround(kAccelerometerOneG * std::sin(elevation)));
    tilt_status->flags = X_TILT_STATUS_FLAGS(0);
  }
  std::memset(tilt_status->unknown_30, 0, sizeof(tilt_status->unknown_30));
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraTiltGetStatus, kNone, kSketchy);

dword_result_t XamNuiCameraElevationGetAngle_entry(lpdword_t elevation_angle,
                                                   lpdword_t camera_moving) {
  X_NUI_TILT_STATUS tilt_status = {};
  tilt_status.buffer_size = sizeof(tilt_status);
  tilt_status.signature = make_fourcc("XtSs");  // (XtSs)?
  X_STATUS result = XamNuiCameraTiltGetStatus_entry(&tilt_status);
  if (XFAILED(result)) {
    return result;
  }
  // The angle in degrees from the gravity direction, which is the camera's
  // elevation; titles poll this until it succeeds.
  const double angle_radians =
      std::atan2(double(int32_t(tilt_status.gravity_long_avg[2])),
                 double(int32_t(tilt_status.gravity_long_avg[1])));
  if (elevation_angle) {
    *elevation_angle =
        static_cast<uint32_t>(std::lround(angle_radians * 180.0 / M_PI));
  }
  if (camera_moving) {
    *camera_moving =
        (tilt_status.flags & (X_TILT_STATUS_FLAGS::TILT_STATUS_UNK1 |
                              X_TILT_STATUS_FLAGS::TILT_STATUS_UNK2)) != 0;
  }
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraElevationGetAngle, kNone, kSketchy);

dword_result_t XamNuiCameraElevationSetAngle_entry(int_t elevation_angle) {
  if (!kernel_state()->nui()->NuiInitialized()) {
    return X_E_DEVICE_NOT_CONNECTED;
  }
  // Moves the Kinect motor when the host has it; otherwise the simulated
  // camera is at the new angle on the title's next XamNuiCameraTiltGetStatus.
  kernel_state()->nui()->SetElevationAngle(int32_t(elevation_angle));
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraElevationSetAngle, kNone, kSketchy);

dword_result_t XamNuiCameraElevationStopMovement_entry() {
  if (!kernel_state()->nui()->NuiInitialized()) {
    return X_E_DEVICE_NOT_CONNECTED;
  }
  // Without the Kinect motor the simulated camera never moves.
  kernel_state()->nui()->StopElevationMovement();
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraElevationStopMovement, kNone, kSketchy);

dword_result_t XamNuiCameraGetTiltControllerType_entry();

// Keeps the floor plane the title found (0x816FA618) for the system's
// automatic tilt, which here finds the floor itself; xam refuses it without a
// tilt controller.
dword_result_t XamNuiCameraRememberFloor_entry(qword_t normal_xy,
                                               qword_t normal_z_distance,
                                               dword_t unk) {
  const int controller_type = XamNuiCameraGetTiltControllerType_entry();
  if (controller_type != 1 && controller_type != 2) {
    return X_E_DEVICE_NOT_CONNECTED;
  }
  // The plane comes by value, two floats in each register.
  auto high = [](uint64_t v) {
    return xe::memory::Reinterpret<float>(uint32_t(v >> 32));
  };
  auto low = [](uint64_t v) {
    return xe::memory::Reinterpret<float>(uint32_t(v));
  };
  XELOGD("Kinect: title floor ({:.3f}, {:.3f}, {:.3f}) at {:.1f}",
         high(normal_xy), low(normal_xy), high(normal_z_distance),
         low(normal_z_distance));
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraRememberFloor, kNone, kSketchy);

// The title's tilt engine reports its state, 0x2B006 (0x10 bytes) or 0x2B00C
// (0xE0 bytes), to whoever set XamNuiCameraElevationSetCallback (0x816FA1A8
// and 0x816FA848); xam answers other messages with the message itself.
dword_result_t XamNuiCameraTiltReportStatus_entry(dword_t message,
                                                  lpvoid_t status) {
  if (message != 0x2B006 && message != 0x2B00C) {
    return message.value();
  }
  const uint32_t callback = kernel_state()->nui()->GetTiltStatusCallback();
  XThread* thread = XThread::GetCurrentThread();
  if (callback && thread) {
    uint64_t args[] = {message, status.guest_address()};
    kernel_state()->processor()->Execute(thread->thread_state(), callback, args,
                                         xe::countof(args));
  }
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraTiltReportStatus, kNone, kImplemented);

// Which tilt motor the sensor has, from UsbdGetNatalHardwareVersion
// (0x816FA088): 1 for the motor hub of model 1414 (DetroitDeviceRequest), 2
// for the motor model 1473 drives through its audio firmware
// (McaDeviceRequest), 0 for none. Xenia drives model 1473's motor, and the
// simulated tilt without one stands in for it too.
constexpr uint32_t kTiltControllerNone = 0;
constexpr uint32_t kTiltControllerAudio = 2;

dword_result_t XamNuiCameraGetTiltControllerType_entry() {
  return kernel_state()->nui()->NuiInitialized() ? kTiltControllerAudio
                                                 : kTiltControllerNone;
}
DECLARE_XAM_EXPORT1(XamNuiCameraGetTiltControllerType, kNone, kImplemented);

// The system's tilt requests (XamApp 0x2B005, 0x2B00A and 0x2B00B), which xam
// hands to the title's tilt engine (0x816F9CB8 and 0x816FA898) and only with
// a tilt motor.
static X_HRESULT RequestTitleTilt(uint32_t message, uint32_t* words,
                                  uint32_t count) {
  if (!XamNuiCameraGetTiltControllerType_entry()) {
    return 0x8007048F;
  }
  return CallTitleCallback(kernel_state()->nui()->GetTiltCallback(), message,
                           words, count);
}

dword_result_t XamNuiCameraElevationAutoTilt_entry(float_t angle) {
  const float value = angle;
  uint32_t request[1];
  std::memcpy(&request[0], &value, sizeof(value));
  return RequestTitleTilt(0x2B005, request, 1);
}
DECLARE_XAM_EXPORT1(XamNuiCameraElevationAutoTilt, kNone, kImplemented);

dword_result_t XamNuiCameraElevationReverseAutoTilt_entry(dword_t unk) {
  uint32_t request[1] = {unk};
  return RequestTitleTilt(0x2B00A, request, 1);
}
DECLARE_XAM_EXPORT1(XamNuiCameraElevationReverseAutoTilt, kNone,
                    kImplemented);

dword_result_t XamNuiCameraAdjustTilt_entry(float_t unk1, float_t unk2,
                                            dword_t unk3) {
  uint32_t request[3] = {unk3};
  const float values[2] = {float(unk1), float(unk2)};
  std::memcpy(&request[1], values, sizeof(values));
  return RequestTitleTilt(0x2B00B, request, 3);
}
DECLARE_XAM_EXPORT1(XamNuiCameraAdjustTilt, kNone, kImplemented);

void XamNuiCameraElevationSetCallback_entry(dword_t callback) {
  kernel_state()->nui()->SetTiltStatusCallback(callback);
}
DECLARE_XAM_EXPORT1(XamNuiCameraElevationSetCallback, kNone, kImplemented);

// Sends the console's microphone array calibration, which xam reads from the
// MicArrayCalibrate.mec system package, to the sensor (0x817F8DE8). Here it
// comes from kinect_audio_calibration_path; 0x8AC80005 is xam's answer when
// the package is missing.
dword_result_t XamNatalDeviceAudioCalibrate_entry() {
  return kernel_state()->nui()->LoadAudioCalibration() ? X_E_SUCCESS
                                                       : 0x8AC80005;
}
DECLARE_XAM_EXPORT1(XamNatalDeviceAudioCalibrate, kNone, kImplemented);

// Copies the sensor's serial number, NUL-terminated, into a buffer of `size`
// bytes (titles pass 0x11).
dword_result_t XamNuiGetDeviceSerialNumber_entry(lpvoid_t buffer_ptr,
                                                 dword_t size) {
  if (!buffer_ptr || !size) {
    return X_E_INVALIDARG;
  }
  const std::string& serial_number = kernel_state()->nui()->GetSerialNumber();
  if (serial_number.empty()) {
    return X_E_DEVICE_NOT_CONNECTED;
  }
  char* buffer = buffer_ptr.as<char*>();
  const size_t length = std::min<size_t>(serial_number.size(), size - 1);
  std::memcpy(buffer, serial_number.data(), length);
  buffer[length] = '\0';
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiGetDeviceSerialNumber, kNone, kImplemented);

// Sets the bits of the tilt controller's flags that `mask` selects. xam reads
// the controller's status, merges the flags and sends them to the motor; the
// motor here keeps no flags, so they're only remembered.
dword_result_t XamNuiCameraSetFlags_entry(qword_t flags, dword_t mask) {
  const int controller_type = XamNuiCameraGetTiltControllerType_entry();
  if (controller_type == 1) {
    X_NUI_TILT_STATUS tilt_status = {};
    tilt_status.buffer_size = sizeof(tilt_status);
    tilt_status.signature = make_fourcc("XtSs");
    const X_STATUS result = XamNuiCameraTiltGetStatus_entry(&tilt_status);
    if (XFAILED(result)) {
      return result;
    }
    NUIController* nui = kernel_state()->nui();
    nui->SetCameraFlags((nui->GetCameraFlags() & ~uint32_t(mask)) |
                        (uint32_t(flags) & uint32_t(mask)));
    return X_ERROR_SUCCESS;
  }
  if (controller_type == 2) {
    NUIController* nui = kernel_state()->nui();
    nui->SetCameraFlags((nui->GetCameraFlags() & ~uint32_t(mask)) |
                        (uint32_t(flags) & uint32_t(mask)));
    return X_ERROR_SUCCESS;
  }
  return X_E_DEVICE_NOT_CONNECTED;
}
DECLARE_XAM_EXPORT1(XamNuiCameraSetFlags, kNone, kImplemented);

dword_result_t XamIsNuiUIActive_entry() {
  return kernel_state()->xam_state()->is_xam_dialog_present_.load();
}
DECLARE_XAM_EXPORT1(XamIsNuiUIActive, kNone, kImplemented);

dword_result_t XamNuiIsDeviceReady_entry() {
  /* device_state Notes:
   - used with XNotifyBroadcast(kXNotificationSystemNUIHardwareStatusChanged,
   device_state)
   - known values:
     - 0x0001
     - 0x0004
     - 0x0040
  */

  return kernel_state()->nui()->NuiInitialized();
}
DECLARE_XAM_EXPORT1(XamNuiIsDeviceReady, kNone, kImplemented);

dword_result_t XamIsNuiAutomationEnabled_entry(unknown_t unk1, unknown_t unk2) {
  /* Notes:
     - XamIsNuiAutomationEnabled = XamIsNatalPlaybackEnabled
     - Always returns X_E_SUCCESS? Maybe check later versions
     - Recieves param but never interacts with them
     - No Operations
  */
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT2(XamIsNuiAutomationEnabled, kNone, kStub, kHighFrequency);

dword_result_t XamIsNatalPlaybackEnabled_entry(unknown_t unk1, unknown_t unk2) {
  /* Notes:
     - XamIsNuiAutomationEnabled = XamIsNatalPlaybackEnabled
     - Always returns X_E_SUCCESS? Maybe check later versions
     - Recieves param but never interacts with them
     - No Operations
  */
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT2(XamIsNatalPlaybackEnabled, kNone, kStub, kHighFrequency);

// Bit 3 of XCONFIG_CONSOLE_NUI's flags turns voice chat through the sensor
// off (0x816F9C38).
dword_result_t XamNuiIsChatMicEnabled_entry() {
  uint32_t nui_config[0x1C / 4] = {};
  kernel_state()->xconfig()->ReadSetting(XCONFIG_CONSOLE_CATEGORY,
                                         XCONFIG_CONSOLE_NUI, nui_config);
  return !(xe::byte_swap(nui_config[0]) & 0x8);
}
DECLARE_XAM_EXPORT1(XamNuiIsChatMicEnabled, kNone, kImplemented);

void XamNuiEnableChatMic_entry(dword_t enable) {
  kernel_state()->nui()->SetChatMicrophoneEnabled(enable != 0);
}
DECLARE_XAM_EXPORT1(XamNuiEnableChatMic, kNone, kImplemented);

// Turns the sensor off for the console, or back on, through KinectDisabled in
// the retail flags (0x817FA170). xam answers 0xD0000184 when it already is.
dword_result_t XamNuiSetForceDeviceOff_entry(dword_t off) {
  auto nui = kernel_state()->nui();
  if (nui->IsForcedOff() == (off != 0)) {
    return 0xD0000184;
  }
  XELOGI("Kinect: turned {} for the console", off ? "off" : "on");
  nui->SetForcedOff(off != 0);
  kernel_state()->BroadcastNotification(
      kXNotificationSystemNUIHardwareStatusChanged,
      kernel_state()->nui()->NuiInitialized());
  return X_STATUS_SUCCESS | X_FACILITY_NT_BIT;
}
DECLARE_XAM_EXPORT1(XamNuiSetForceDeviceOff, kNone, kImplemented);

// The support code the Kinect troubleshooter shows (0x817FA2E0): the sensor's
// mode and state, the kind and code of the problem (0 when its firmware is the
// one the console expects), that firmware's version and the serial number.
dword_result_t XamNuiGetSupportString_entry(lpvoid_t buffer, dword_t size) {
  if (!buffer || !size) {
    return X_E_INVALIDARG;
  }
  auto nui = kernel_state()->nui();
  const bool present = nui->NuiInitialized();
  std::string serial = present ? nui->GetSerialNumber() : std::string();
  serial.resize(12, '0');
  // The firmware in the system update, fwversions.txt 01.02.0810.00.
  const std::string text =
      fmt::format("K:{:02d}:{:02d}:{:02d}:{:02d}:{:02d}.{:02d}.{:04d}.{:02d}:{:>12}",
                  present ? 1 : 0, 0, 0, present ? 0 : 1, 1, 2, 810, 0,
                  serial);
  const size_t length = std::min<size_t>(text.size(), size - 1);
  std::memcpy(buffer.as<char*>(), text.data(), length);
  buffer.as<char*>()[length] = '\0';
  return X_STATUS_SUCCESS | X_FACILITY_NT_BIT;
}
DECLARE_XAM_EXPORT1(XamNuiGetSupportString, kNone, kSketchy);

// The sensor's firmware update, which the system runs (0x817FA228 and
// 0x817FA270); a missing firmware file opens the troubleshooter.
dword_result_t XamNuiNatalCameraUpdateStarting_entry(dword_t unk1,
                                                     dword_t unk2) {
  XELOGI("Kinect: firmware update starting");
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiNatalCameraUpdateStarting, kNone, kImplemented);

dword_result_t XamShowNuiTroubleshooterUI_entry(dword_t user_index,
                                                dword_t tracking_id,
                                                dword_t flags);

void XamNuiNatalCameraUpdateComplete_entry(dword_t result) {
  XELOGI("Kinect: firmware update finished, {:08X}", result.value());
  if (result == 0x80070002) {
    XamShowNuiTroubleshooterUI_entry(0xFF, 0, 0);
  }
}
DECLARE_XAM_EXPORT1(XamNuiNatalCameraUpdateComplete, kNone, kImplemented);

/* HUD Notes:
   - XamNuiHudGetEngagedTrackingID, XamNuiHudIsEnabled,
   XamNuiHudSetEngagedTrackingID, XamNuiHudInterpretFrame, and
   XamNuiHudGetEngagedEnrollmentIndex all utilize the same data address
*/
uint32_t nui_unknown_1 = 0;
uint32_t engaged_tracking_id = 0;
char nui_unknown_2 = '\0';

dword_result_t XamNuiHudSetEngagedTrackingID_entry(dword_t id) {
  auto nui_ = kernel_state()->nui();
  if (id) {
    if (!nui_->GetNUIDataPtr()) {
      return X_E_FAIL;
    }
    nui_->SetEngagedTrackingId(id);
  }

  return X_STATUS_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiHudSetEngagedTrackingID, kNone, kImplemented);

qword_result_t XamNuiHudGetEngagedTrackingID_entry() {
  auto nui_ = kernel_state()->nui();
  if (nui_->GetNUIDataPtr()) {
    return nui_->GetEngagedTrackingId();
  }

  return X_STATUS_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiHudGetEngagedTrackingID, kNone, kImplemented);

dword_result_t XamNuiHudIsEnabled_entry() {
  auto nui_ = kernel_state()->nui();
  bool result = XamNuiIsDeviceReady_entry();
  if (nui_->GetNUIDataPtr() && nui_->GetUnknown2() && result) {
    return true;
  }
  return false;
}
DECLARE_XAM_EXPORT1(XamNuiHudIsEnabled, kNone, kImplemented);

dword_result_t XamNuiHudGetInitializeFlags_entry() {
  return kernel_state()->nui()->GetInitFlags();
}
DECLARE_XAM_EXPORT1(XamNuiHudGetInitializeFlags, kNone, kImplemented);

void XamNuiHudGetVersions_entry(lpqword_t xam_version_ptr,
                                lpqword_t xtl_version_ptr) {
  auto nui_ = kernel_state()->nui();
  if (xam_version_ptr) {
    *xam_version_ptr = nui_->GetNUIVersion(0);
  }
  if (xtl_version_ptr) {
    *xtl_version_ptr = nui_->GetNUIVersion(1);
  }
}
DECLARE_XAM_EXPORT1(XamNuiHudGetVersions, kNone, kImplemented);

/* XamNuiIdentity Notes:
   - most require message calls to xam in 0x0002Cxxx area
*/
qword_result_t XamNuiIdentityGetSessionId_entry() {
  auto nui_ = kernel_state()->nui();
  if (nui_->GetSessionId() == 0) {
    // xboxkrnl::XeCryptRandom_entry(NUI_Session_Id, 8);
    nui_->SetSessionId(0xDEADF00DDEADF00D);
  }
  return nui_->GetSessionId();
}
DECLARE_XAM_EXPORT1(XamNuiIdentityGetSessionId, kNone, kImplemented);

dword_result_t XamNuiIdentityEnrollForSignIn_entry(dword_t unk1, qword_t unk2,
                                                   qword_t unk3, dword_t unk4) {
  /* Notes:
     - Decompiler issues so double check
  */
  if (XamNuiHudIsEnabled_entry() == false) {
    return X_E_FAIL;
  }
  // buffer [2]
  // buffer[0] = unk
  // var = unk4
  // return func(0xfe,0x2c010,buffer,0xc,unk3);
  return X_E_FAIL;
}
DECLARE_XAM_EXPORT1(XamNuiIdentityEnrollForSignIn, kNone, kStub);

// Cancels an identification (0x2C00E, 0x8180D910); they finish at once here.
dword_result_t XamNuiIdentityAbort_entry(dword_t unk) {
  if (XamNuiHudIsEnabled_entry() == false) {
    return X_E_FAIL;
  }
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiIdentityAbort, kNone, kImplemented);

// Kinect ID enrollment of a signed-in user; nobody is ever enrolled, for which
// xam reports index -2.
dword_result_t XamUserNuiGetEnrollmentIndex_entry(dword_t user_index,
                                                  lpdword_t enrollment_index) {
  if (user_index >= XUserMaxUserCount || !enrollment_index) {
    return X_E_INVALIDARG;
  }
  *enrollment_index = kernel_state()->nui()->GetUserEnrollment(user_index);
  return *enrollment_index == NUIController::kNoEnrollment ? X_E_NO_SUCH_USER
                                                           : X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamUserNuiGetEnrollmentIndex, kNone, kImplemented);

// The Kinect ID data a title keeps for a profile, which xam stores on Xbox
// Live and which is kept with the profile here: a version the title raises
// with each enrollment and the title's data, up to the size in `size_ptr`.
// 4D53090F reads it when a profile signs in and writes it after enrolling
// the profile's face.
static X_RESULT CompleteBiometricRequest(lpvoid_t overlapped, X_RESULT result) {
  if (overlapped) {
    kernel_state()->CompleteOverlappedImmediate(overlapped, result);
    return X_ERROR_IO_PENDING;
  }
  return result;
}

dword_result_t XamReadBiometricData_entry(qword_t xuid, lpdword_t version_ptr,
                                          lpvoid_t buffer, lpdword_t size_ptr,
                                          lpvoid_t overlapped) {
  if (!version_ptr || !size_ptr) {
    return X_ERROR_INVALID_PARAMETER;
  }
  NUIController::BiometricData data;
  if (!kernel_state()->nui()->ReadStoredBiometricData(xuid, data)) {
    *version_ptr = 0;
    *size_ptr = 0;
    return CompleteBiometricRequest(overlapped, X_ERROR_NOT_FOUND);
  }
  if (!buffer || data.data.size() > *size_ptr) {
    *size_ptr = static_cast<uint32_t>(data.data.size());
    return CompleteBiometricRequest(overlapped, X_ERROR_INSUFFICIENT_BUFFER);
  }
  std::memcpy(buffer, data.data.data(), data.data.size());
  *version_ptr = data.version;
  *size_ptr = static_cast<uint32_t>(data.data.size());
  return CompleteBiometricRequest(overlapped, X_ERROR_SUCCESS);
}
DECLARE_XAM_EXPORT1(XamReadBiometricData, kNone, kImplemented);

// Replaces the profile's copy; xam writes size 0 to delete it when Kinect ID
// sign-in is turned off.
dword_result_t XamWriteBiometricData_entry(qword_t xuid, lpdword_t version_ptr,
                                           lpvoid_t buffer, lpdword_t size_ptr,
                                           lpvoid_t overlapped) {
  NUIController::BiometricData data;
  data.version = version_ptr ? uint32_t(*version_ptr) : 0;
  const uint32_t size = size_ptr ? uint32_t(*size_ptr) : 0;
  if (size) {
    if (!buffer) {
      return X_ERROR_INVALID_PARAMETER;
    }
    const uint8_t* source = buffer.as<const uint8_t*>();
    data.data.assign(source, source + size);
  }
  const bool written =
      kernel_state()->nui()->WriteStoredBiometricData(xuid, data);
  return CompleteBiometricRequest(
      overlapped, written ? X_ERROR_SUCCESS : X_ERROR_FUNCTION_FAILED);
}
DECLARE_XAM_EXPORT1(XamWriteBiometricData, kNone, kImplemented);

// Other
// Whether Kinect ID signs the profile in, kept in its dashboard settings.
// Turning it off deletes the profile's Kinect ID data, as xam does.
dword_result_t XamUserNuiEnableBiometric_entry(dword_t user_index,
                                               int_t enable) {
  if (user_index >= XUserMaxUserCount) {
    return X_E_INVALIDARG;
  }
  const auto user_profile =
      kernel_state()->xam_state()->GetUserProfile(user_index);
  if (!user_profile) {
    return X_E_NO_SUCH_USER;
  }
  const UserSetting setting(UserSettingId::XPROFILE_NUI_BIOMETRIC_SIGNIN,
                            int32_t(enable));
  kernel_state()->xam_state()->user_tracker()->UpsertSetting(
      user_profile->xuid(), kDashboardID, &setting);
  if (!enable) {
    auto nui = kernel_state()->nui();
    nui->WriteStoredBiometricData(user_profile->xuid(), {});
    nui->SetCachedBiometricData(user_profile->xuid(),
                                {NUIController::kBiometricNone, {}});
  }
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamUserNuiEnableBiometric, kNone, kImplemented);

dword_result_t XamUserNuiIsBiometricEnabled_entry(dword_t user_index) {
  if (user_index >= XUserMaxUserCount) {
    return 0;
  }
  const auto user_profile =
      kernel_state()->xam_state()->GetUserProfile(user_index);
  if (!user_profile) {
    return 0;
  }
  const auto setting = kernel_state()->xam_state()->user_tracker()->GetSetting(
      user_profile, kDashboardID,
      static_cast<uint32_t>(UserSettingId::XPROFILE_NUI_BIOMETRIC_SIGNIN));
  return setting && std::get<int32_t>(setting->get_host_data()) != 0;
}
DECLARE_XAM_EXPORT1(XamUserNuiIsBiometricEnabled, kNone, kImplemented);

// xam hands identity messages to the title's Kinect ID engine, through the
// callback it registered (0x8180E750): message, then {size, data}.
static X_HRESULT CallIdentityCallback(uint32_t message, uint32_t* words,
                                      uint32_t count) {
  return CallTitleCallback(kernel_state()->nui()->GetIdentityCallback(),
                           message, words, count);
}

dword_result_t XamNuiIdentityGetEnrollmentInfo_entry(dword_t enrollment_index,
                                                     lpdword_t info1,
                                                     lpdword_t info2) {
  uint32_t request[4] = {0x10, enrollment_index, 0, 0};
  X_HRESULT result = CallIdentityCallback(0x2C013, request, 4);
  if (XSUCCEEDED(result)) {
    if (request[0] != 0x10) {
      return 0x8000FFFF;
    }
    if (info1) {
      *info1 = request[2];
    }
    if (info2) {
      *info2 = request[3];
    }
  }
  return result;
}
DECLARE_XAM_EXPORT1(XamNuiIdentityGetEnrollmentInfo, kNone, kImplemented);

void XamNuiIdentityUnenroll_entry(dword_t enrollment_index) {
  uint32_t request[2] = {8, enrollment_index};
  CallIdentityCallback(0x2C012, request, 2);
}
DECLARE_XAM_EXPORT1(XamNuiIdentityUnenroll, kNone, kImplemented);

// The face picture of the last identification, as a texture header pointing
// at the title's image (0x8180DF08): the engine answers 0x2C011 with its
// width, height, format, pitch and address.
dword_result_t XamNuiIdentityGetColorTexture_entry(lpvoid_t texture) {
  uint32_t request[6] = {0x18, 0, 0, 0, 0, 0};
  X_HRESULT result = CallIdentityCallback(0x2C011, request, 6);
  if (XSUCCEEDED(result)) {
    if (request[0] != 0x18) {
      return 0x8000FFFF;
    }
    if (texture) {
      auto header = texture.as<xe::be<uint32_t>*>();
      xeXGSetTextureHeaderEx(request[1], request[2], 1, 4, request[3], 0, 1, 0,
                             0, request[4], header, nullptr, nullptr);
      xeXGOffsetResourceAddress(header, request[5]);
    }
  }
  return result;
}
DECLARE_XAM_EXPORT1(XamNuiIdentityGetColorTexture, kNone, kSketchy);

struct X_NUI_PLAYER_ENGAGEMENT_UPDATE {
  xe::be<uint32_t> engaged_tracking_id;
  xe::be<uint32_t> engaged_index;
  xe::be<uint32_t> engaged_user_index;
  xe::be<uint32_t> engaged_enrollment_index;
  xe::be<uint32_t> tracked_count;
  xe::be<uint32_t> tracked_indices[2];
};
static_assert_size(X_NUI_PLAYER_ENGAGEMENT_UPDATE, 0x1C);

// Reports the engaged player and the tracked skeletons, and picks the two
// skeletons the tracker follows (+0xACC in the frame): the engaged player,
// then the others by distance across the floor, tracked ones counting 0.3 m
// closer. As xam does it.
static void UpdatePlayerEngagement(uint8_t* frame,
                                   uint32_t engaged_tracking_id,
                                   X_NUI_PLAYER_ENGAGEMENT_UPDATE* engagement) {
  std::memset(engagement, 0, sizeof(X_NUI_PLAYER_ENGAGEMENT_UPDATE));
  engagement->engaged_index = 0xFFFFFFFF;
  engagement->engaged_user_index = 0xFE;
  engagement->engaged_enrollment_index = 0xFFFFFFFE;
  if (!frame) {
    return;
  }
  constexpr float kTrackedBonus = 0.3f;
  constexpr float kMinimumDistance = 0.1f;
  float chosen_distance[2] = {FLT_MAX, FLT_MAX};
  uint32_t chosen_id[2] = {0, 0};
  for (uint32_t i = 0; i < kSkeletonCount; ++i) {
    const auto* skeleton = GetSkeleton(frame, i);
    const uint32_t state = skeleton->tracking_state;
    if (!state) {
      continue;
    }
    if (state == kSkeletonTracked && engagement->tracked_count < 2) {
      engagement->tracked_indices[engagement->tracked_count] = i;
      engagement->tracked_count = engagement->tracked_count + 1;
    }
    uint32_t id = skeleton->tracking_id;
    float distance = 0.0f;
    if (id == engaged_tracking_id) {
      engagement->engaged_tracking_id = id;
      engagement->engaged_index = i;
      engagement->engaged_user_index = skeleton->user_index;
      engagement->engaged_enrollment_index = skeleton->enrollment_index;
    } else {
      const float x = skeleton->position[0];
      const float z = skeleton->position[2];
      distance = std::sqrt(x * x + z * z);
      if (state == kSkeletonTracked) {
        distance -= kTrackedBonus;
      }
      distance = std::max(distance, kMinimumDistance);
    }
    for (uint32_t slot = 0; slot < 2; ++slot) {
      if (distance < chosen_distance[slot]) {
        std::swap(distance, chosen_distance[slot]);
        std::swap(id, chosen_id[slot]);
      }
    }
  }
  xe::store_and_swap<uint32_t>(frame + kSkeletonFrameChosenIds, chosen_id[0]);
  xe::store_and_swap<uint32_t>(frame + kSkeletonFrameChosenIds + 4,
                               chosen_id[1]);
}

void XamNuiPlayerEngagementUpdate_entry(
    lpvoid_t skeleton_frame, dword_t engaged_tracking_id,
    pointer_t<X_NUI_PLAYER_ENGAGEMENT_UPDATE> engagement) {
  UpdatePlayerEngagement(
      skeleton_frame ? skeleton_frame.as<uint8_t*>() : nullptr,
      engaged_tracking_id, engagement);
}
DECLARE_XAM_EXPORT1(XamNuiPlayerEngagementUpdate, kNone, kImplemented);

// The NUI HUD: what xam does with each skeleton frame a title hands it
// through XamNuiHudInterpretFrame (0x816ACC68 and 0x816AC688). It picks the
// two skeletons to track, marks players the sensor sees badly as position
// only, engages a player who waves, and opens the Kinect Guide on the guide
// gesture.
namespace {

constexpr uint32_t kJointElbowLeft = 5;
constexpr uint32_t kJointElbowRight = 9;
constexpr uint32_t kJointHipCenter = 0;

// The engagement wave (0x8180AAB8, parameters from 0x8180A388): four half
// waves of a raised hand, each at least three frames the same way and fast
// enough, at most a second apart and keeping a steady rhythm.
struct WaveHand {
  enum Direction : uint32_t { kNone = 1, kLeft = 2, kRight = 3 };
  static constexpr uint32_t kHalfWaves = 4;
  static constexpr uint32_t kDirectionFrames = 3;
  static constexpr float kStartMs = 1000.0f;
  static constexpr float kHalfWaveMs = 1000.0f;
  static constexpr float kSlowSpeed = 0.7f;
  static constexpr float kFastSpeed = 1.5f;
  static constexpr float kRhythmTolerance = 0.6f;

  uint32_t tracking_id = 0;
  bool started = false;
  float previous[2] = {};
  uint32_t direction = kNone;
  uint32_t direction_frames = 0;
  uint32_t candidate = kNone;
  float speed = 0.0f;
  float tracked_ms = 0.0f;
  uint32_t half_waves = 0;
  uint32_t last_direction = kNone;
  float elapsed_ms = 0.0f;
  float last_half_wave_ms = 0.0f;
  float period_ms = 0.0f;

  void ResetWaves() {
    speed = 0.0f;
    half_waves = 0;
    elapsed_ms = 0.0f;
    last_direction = kNone;
    last_half_wave_ms = 0.0f;
    period_ms = 0.0f;
  }
  void Reset(uint32_t id) {
    *this = WaveHand();
    tracking_id = id;
  }

  // The hand's sideways speed, in m/s, smoothed over up to three frames that
  // go the same way (0x8180A988).
  void UpdateDirection(float dt_ms, const float* hand, bool hand_tracked) {
    float measured = speed;
    if (hand_tracked) {
      const float dx = hand[0] - previous[0], dy = hand[1] - previous[1];
      measured = std::abs(dy) <= std::abs(dx) && dt_ms > 0.0f
                     ? dx / (dt_ms * 0.001f)
                     : 0.0f;
    }
    if (measured != 0.0f) {
      const uint32_t moving = measured > 0.0f ? kRight : kLeft;
      if (direction == moving) {
        ++direction_frames;
      } else {
        direction = moving;
        direction_frames = 1;
      }
    }
    if (direction_frames) {
      if (direction_frames == kDirectionFrames) {
        candidate = direction;
      }
      const float frames = float(std::min(direction_frames, 3u));
      speed += (measured - speed) / frames;
    }
  }

  // A swing the other way, fast enough for the frames it took (0x8180A5C0).
  bool HalfWave() const {
    const float slow = kSlowSpeed, fast = kFastSpeed;
    const uint32_t n = direction_frames;
    switch (last_direction) {
      case kLeft:
        return (candidate == kRight && n > kDirectionFrames && speed > slow) ||
               (n >= kDirectionFrames && speed > fast);
      case kRight:
        return (candidate == kLeft && n > kDirectionFrames && speed < -slow) ||
               (n >= kDirectionFrames && speed < -fast);
      default:
        return (n > kDirectionFrames && std::abs(speed) > slow) ||
               (n >= kDirectionFrames && std::abs(speed) > fast);
    }
  }

  // Returns true when the wave completes.
  bool Update(float dt_ms, const float* hand, bool hand_tracked, bool raised) {
    bool engaged = false;
    if (!started) {
      if (!hand_tracked) {
        return false;
      }
      started = true;
    } else {
      UpdateDirection(dt_ms, hand, hand_tracked);
      // 0x8180A518: a second of tracking first, then at most a second
      // between half waves.
      tracked_ms += dt_ms;
      bool counting = tracked_ms > kStartMs;
      if (!counting) {
        ResetWaves();
      } else if (half_waves) {
        elapsed_ms += dt_ms;
        if (elapsed_ms - last_half_wave_ms > kHalfWaveMs) {
          ResetWaves();
        }
      }
      if (counting && HalfWave()) {
        last_direction = candidate;
        ++half_waves;
        // 0x8180A6F8: the rhythm.
        const float interval = elapsed_ms - last_half_wave_ms;
        if (half_waves > 1) {
          if (half_waves < 3 ||
              std::abs(period_ms / interval - 1.0f) <= kRhythmTolerance) {
            period_ms += (interval - period_ms) / float(half_waves - 1);
          } else {
            half_waves = 2;
            period_ms = interval;
          }
        }
        last_half_wave_ms = elapsed_ms;
        if (!raised) {
          ResetWaves();
        }
        if (half_waves >= kHalfWaves) {
          ResetWaves();
          tracked_ms = 0.0f;
          engaged = true;
        }
      }
    }
    if (hand_tracked) {
      previous[0] = hand[0];
      previous[1] = hand[1];
    }
    return engaged;
  }
};

// The guide gesture (0x8180A0B0): a pose held for three seconds, after which
// the others wait 1.5 s. xam measures the arm against the trunk with angle
// windows it doesn't name; this takes the gesture as the Xbox 360 explains it,
// the left arm held straight out from the body, about 45 degrees down.
struct GuidePose {
  static constexpr float kHoldMs = 3000.0f;
  static constexpr float kCooldownMs = 1500.0f;
  // How far each frame moves the smoothed hand and shoulders (0x81AD218C and
  // 0x81AD2190), which keeps the tracker's jitter out of the pose.
  static constexpr float kHandSmoothing = 0.1f;
  static constexpr float kShoulderSmoothing = 0.05f;
  uint32_t tracking_id = 0;
  float held_ms = 0.0f;
  float cooldown_ms = 0.0f;
  bool held_last_frame = false;
  bool smoothed = false;
  float hand_left[3] = {}, hand_right[3] = {};
  float shoulder_left[3] = {}, shoulder_right[3] = {};
};

struct NuiHudState {
  std::mutex mutex;
  std::chrono::steady_clock::time_point last_frame;
  bool have_last_frame = false;
  X_NUI_SKELETON_SCORES scores = {};
  WaveHand waves[kScoreRecords][2];
  GuidePose guide[kSkeletonCount];
  // The skeletons of the last frame: tracking ID and enrollment index.
  uint32_t tracking_ids[kSkeletonCount] = {};
  uint32_t enrollment_indices[kSkeletonCount] = {};
  bool interpreted = false;
  uint32_t input_filter = 0;
};

NuiHudState& GetHudState() {
  static NuiHudState state;
  return state;
}

struct SkeletonJoints {
  const uint8_t* skeleton;
  uint32_t state(uint32_t joint) const {
    return xe::load_and_swap<uint32_t>(skeleton + kSkeletonJointStates +
                                       joint * 4);
  }
  void position(uint32_t joint, float* out) const {
    for (uint32_t i = 0; i < 3; ++i) {
      out[i] = xe::load_and_swap<float>(skeleton + kSkeletonJointPositions +
                                        joint * 16 + i * 4);
    }
  }
};

float Dot(const float* a, const float* b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
float Length(const float* a) { return std::sqrt(Dot(a, a)); }

// 0x8180A788: the hand above the elbow and not reaching far towards the
// sensor, with the player facing it.
bool HandRaised(const SkeletonJoints& joints, uint32_t hand, uint32_t elbow,
                uint32_t shoulder, const float* up,
                const X_NUI_SKELETON_SCORE& score) {
  float h[3], e[3], s[3];
  joints.position(hand, h);
  joints.position(elbow, e);
  joints.position(shoulder, s);
  const float above[3] = {h[0] - e[0], h[1] - e[1], h[2] - e[2]};
  if (Dot(above, up) < 0.0f || h[2] > s[2] + 0.06f) {
    return false;
  }
  // xam takes the turn as under 45 degrees; the score's angle is to either
  // shoulder, so both ends of it face the sensor.
  const float turn = score.turn_degrees;
  return std::min(turn, 180.0f - turn) < 45.0f;
}

float AngleDegrees(float y, float x) {
  return std::atan2(y, x) * (180.0f / 3.14159265f);
}

// The angle at the elbow between the upper arm and the forearm, 0 when the
// arm is straight (0x81809820).
float ElbowBendDegrees(const SkeletonJoints& joints, uint32_t shoulder,
                       uint32_t elbow, uint32_t hand) {
  float s[3], e[3], h[3];
  joints.position(shoulder, s);
  joints.position(elbow, e);
  joints.position(hand, h);
  const float upper[3] = {e[0] - s[0], e[1] - s[1], e[2] - s[2]};
  const float lower[3] = {h[0] - e[0], h[1] - e[1], h[2] - e[2]};
  const float lengths = Length(upper) * Length(lower);
  if (lengths == 0.0f) {
    return 180.0f;
  }
  return std::acos(std::clamp(Dot(upper, lower) / lengths, -1.0f, 1.0f)) *
         (180.0f / 3.14159265f);
}

// The guide gesture's pose (0x81809AF0, windows from 0x81809558): the left arm
// straight and 45 degrees out from hanging down, give or take 20, without
// reaching forwards; the right arm hanging down; the trunk upright. Hands and
// shoulders are smoothed (0x81809FD0), and both shoulders must be tracked.
// With frame flag 4 or 8 the left arm may rise further and the right arm and
// trunk don't count.
bool GuidePoseHeld(GuidePose& pose, const SkeletonJoints& joints,
                   const float* up, bool relaxed) {
  if (joints.state(kJointShoulderLeft) != kSkeletonTracked ||
      joints.state(kJointShoulderRight) != kSkeletonTracked ||
      !joints.state(kJointHandLeft)) {
    pose.smoothed = false;
    return false;
  }
  float hand_left[3], hand_right[3], shoulder_left[3], shoulder_right[3];
  joints.position(kJointHandLeft, hand_left);
  joints.position(kJointHandRight, hand_right);
  joints.position(kJointShoulderLeft, shoulder_left);
  joints.position(kJointShoulderRight, shoulder_right);
  for (uint32_t i = 0; i < 3; ++i) {
    if (!pose.smoothed) {
      pose.hand_left[i] = hand_left[i];
      pose.hand_right[i] = hand_right[i];
      pose.shoulder_left[i] = shoulder_left[i];
      pose.shoulder_right[i] = shoulder_right[i];
      continue;
    }
    pose.hand_left[i] +=
        (hand_left[i] - pose.hand_left[i]) * GuidePose::kHandSmoothing;
    pose.hand_right[i] +=
        (hand_right[i] - pose.hand_right[i]) * GuidePose::kHandSmoothing;
    pose.shoulder_left[i] += (shoulder_left[i] - pose.shoulder_left[i]) *
                             GuidePose::kShoulderSmoothing;
    pose.shoulder_right[i] += (shoulder_right[i] - pose.shoulder_right[i]) *
                              GuidePose::kShoulderSmoothing;
  }
  pose.smoothed = true;
  // The body: up from the floor, out towards the player's left across the
  // shoulders, and forwards.
  float out[3];
  for (uint32_t i = 0; i < 3; ++i) {
    out[i] = pose.shoulder_left[i] - pose.shoulder_right[i];
  }
  const float out_up = Dot(out, up);
  for (uint32_t i = 0; i < 3; ++i) {
    out[i] -= out_up * up[i];
  }
  const float out_length = Length(out);
  if (out_length == 0.0f) {
    return false;
  }
  for (float& v : out) {
    v /= out_length;
  }
  const float forward[3] = {up[1] * out[2] - up[2] * out[1],
                            up[2] * out[0] - up[0] * out[2],
                            up[0] * out[1] - up[1] * out[0]};
  // An arm's angle out to its side from hanging down, and forwards or back.
  auto arm_angles = [&](const float* shoulder, const float* hand, float side,
                        float& sideways, float& forwards) {
    const float arm[3] = {hand[0] - shoulder[0], hand[1] - shoulder[1],
                          hand[2] - shoulder[2]};
    const float down = -Dot(arm, up);
    sideways = AngleDegrees(side * Dot(arm, out), down);
    forwards = AngleDegrees(Dot(arm, forward), down);
  };
  float left_out, left_forward;
  arm_angles(pose.shoulder_left, pose.hand_left, 1.0f, left_out, left_forward);
  // xam: 205 to 245 (275) degrees across, 250 (245) to 290 (295) along, the
  // elbow bent under 35 (40) degrees; hanging down is 180 and 270.
  const float left_bend = ElbowBendDegrees(joints, kJointShoulderLeft,
                                           kJointElbowLeft, kJointHandLeft);
  const bool left =
      relaxed ? left_out > 25.0f && left_out < 95.0f &&
                    std::abs(left_forward) < 25.0f && left_bend < 40.0f
              : left_out > 25.0f && left_out < 65.0f &&
                    std::abs(left_forward) < 20.0f && left_bend < 35.0f;
  // The right arm within 20 degrees of hanging down and bent under 110,
  // unless its hand isn't tracked; the relaxed pose leaves it free.
  bool right = true;
  if (!relaxed && joints.state(kJointHandRight)) {
    float right_out, right_forward;
    arm_angles(pose.shoulder_right, pose.hand_right, -1.0f, right_out,
               right_forward);
    right = std::abs(right_out) < 20.0f &&
            ElbowBendDegrees(joints, kJointShoulderRight, kJointElbowRight,
                             kJointHandRight) < 110.0f;
  }
  // The trunk, from the hips to the shoulders across the body's side view,
  // within 9 degrees of upright.
  bool trunk = true;
  if (!relaxed) {
    float hips[3], shoulders[3];
    joints.position(kJointHipCenter, hips);
    joints.position(kJointShoulderCenter, shoulders);
    const float spine[3] = {shoulders[0] - hips[0], shoulders[1] - hips[1],
                            shoulders[2] - hips[2]};
    trunk = std::abs(AngleDegrees(Dot(spine, forward), Dot(spine, up))) < 9.0f;
  }
  return left && right && trunk;
}

// Returns whether the player held the guide gesture, to open the Guide once
// the HUD is unlocked.
bool InterpretFrame(NuiHudState& hud, uint8_t* frame, float dt_ms) {
  auto nui = kernel_state()->nui();
  float up[3];
  for (uint32_t i = 0; i < 3; ++i) {
    up[i] = xe::load_and_swap<float>(frame + kSkeletonFrameNormalToGravity +
                                     i * 4);
  }
  uint32_t engaged = nui->GetEngagedTrackingId();
  X_NUI_PLAYER_ENGAGEMENT_UPDATE engagement;
  UpdatePlayerEngagement(frame, engaged, &engagement);
  UpdateSkeletonScores(frame, hud.scores);
  // The guide gesture goes by what the tracker saw, before the warnings.
  uint32_t tracking_states[kSkeletonCount];
  for (uint32_t i = 0; i < kSkeletonCount; ++i) {
    tracking_states[i] = GetSkeleton(frame, i)->tracking_state;
  }
  // Players with a warning reach the title as position only.
  for (const auto& score : hud.scores.players) {
    const uint32_t index = score.skeleton_index;
    if (!score.tracking_id || index >= kSkeletonCount) {
      continue;
    }
    auto* skeleton = GetSkeleton(frame, index);
    if (skeleton->tracking_id == score.tracking_id &&
        skeleton->tracking_state == kSkeletonTracked) {
      skeleton->tracking_state = score.state;
    }
  }
  if (engaged && engagement.engaged_index == 0xFFFFFFFF) {
    XELOGI("Kinect: engaged player {:08X} left", engaged);
    nui->SetEngagedTrackingId(0);
    engaged = 0;
  }
  for (uint32_t record = 0; record < kScoreRecords; ++record) {
    const auto& score = hud.scores.players[record];
    const uint32_t id = score.tracking_id;
    const uint32_t index = score.skeleton_index;
    const auto* skeleton =
        index < kSkeletonCount ? GetSkeleton(frame, index) : nullptr;
    for (uint32_t side = 0; side < 2; ++side) {
      WaveHand& wave = hud.waves[record][side];
      if (!id || !skeleton || skeleton->tracking_id != id ||
          score.tracking_state != kSkeletonTracked) {
        wave.Reset(0);
        continue;
      }
      if (wave.tracking_id != id) {
        wave.Reset(id);
      }
      if (id == engaged) {
        continue;
      }
      const SkeletonJoints joints{reinterpret_cast<const uint8_t*>(skeleton)};
      const uint32_t hand = side ? kJointHandRight : kJointHandLeft;
      float position[3];
      joints.position(hand, position);
      const bool raised =
          HandRaised(joints, hand, side ? kJointElbowRight : kJointElbowLeft,
                     side ? kJointShoulderRight : kJointShoulderLeft, up,
                     score);
      if (wave.Update(dt_ms, position, joints.state(hand) != 0, raised)) {
        XELOGI("Kinect: player {:08X} engaged by waving", id);
        nui->SetEngagedTrackingId(id);
        engaged = id;
      }
    }
  }
  for (uint32_t i = 0; i < kSkeletonCount; ++i) {
    const auto* skeleton = GetSkeleton(frame, i);
    hud.tracking_ids[i] = skeleton->tracking_state ? uint32_t(skeleton->tracking_id) : 0;
    hud.enrollment_indices[i] = skeleton->enrollment_index;
  }
  hud.interpreted = true;
  // The guide gesture.
  constexpr uint32_t kSkeletonFrameFlags = 0xAB8;
  const bool relaxed_pose =
      xe::load_and_swap<uint32_t>(frame + kSkeletonFrameFlags) & (4 | 8);
  bool open_guide = false;
  uint32_t guide_player = 0;
  for (uint32_t i = 0; i < kSkeletonCount; ++i) {
    GuidePose& pose = hud.guide[i];
    const auto* skeleton = GetSkeleton(frame, i);
    pose.cooldown_ms = std::max(pose.cooldown_ms - dt_ms, 0.0f);
    if (pose.tracking_id != skeleton->tracking_id) {
      pose = GuidePose();
      pose.tracking_id = skeleton->tracking_id;
    }
    // The system ignores the gesture while its Guide is open.
    const bool held =
        cvars::kinect_guide_gesture && !xeXamIsGuideOpen() &&
        tracking_states[i] == kSkeletonTracked &&
        GuidePoseHeld(
            pose, SkeletonJoints{reinterpret_cast<const uint8_t*>(skeleton)},
            up, relaxed_pose);
    if (held && pose.cooldown_ms == 0.0f) {
      pose.held_ms += dt_ms;
    } else if (!pose.held_last_frame) {
      // A frame's slip doesn't lose the pose.
      pose.held_ms = 0.0f;
    }
    pose.held_last_frame = held;
    if (pose.held_ms >= GuidePose::kHoldMs) {
      open_guide = true;
      guide_player = pose.tracking_id;
    }
  }
  if (open_guide) {
    for (auto& pose : hud.guide) {
      pose.held_ms = 0.0f;
      pose.cooldown_ms = GuidePose::kCooldownMs;
    }
    XELOGI("Kinect: guide gesture by player {:08X}", guide_player);
  }
  return open_guide;
}

}  // namespace

X_RESULT xeXamNuiHudInterpretFrame(uint8_t* frame) {
  X_RESULT result = X_E_FAIL;
  bool open_guide = false;
  if (frame && XamNuiHudIsEnabled_entry()) {
    NuiHudState& hud = GetHudState();
    std::lock_guard<std::mutex> lock(hud.mutex);
    const auto now = std::chrono::steady_clock::now();
    float dt_ms = 0.0f;
    if (hud.have_last_frame) {
      dt_ms = std::chrono::duration<float, std::milli>(now - hud.last_frame)
                  .count();
      // After a pause, as one frame.
      dt_ms = std::min(dt_ms, 100.0f);
    }
    hud.last_frame = now;
    hud.have_last_frame = true;
    open_guide = InterpretFrame(hud, frame, dt_ms);
    result = X_E_SUCCESS;
  }
  // The Guide pauses the title, which may be waiting on the HUD.
  if (open_guide && xeXamShowGuide() != X_ERROR_SUCCESS) {
    XELOGI("Kinect: the Guide is unavailable");
  }
  // The system has the player's input while its UI is open.
  if (kernel_state()->xam_state()->IsUIActive()) {
    result = 0x8301000B;
  }
  return result;
}

dword_result_t XamNuiHudInterpretFrame_entry(lpvoid_t skeleton_frame) {
  return xeXamNuiHudInterpretFrame(
      skeleton_frame ? skeleton_frame.as<uint8_t*>() : nullptr);
}
DECLARE_XAM_EXPORT1(XamNuiHudInterpretFrame, kNone, kImplemented);

// The enrollment index of the engaged player in the last frame the HUD
// interpreted, or -2.
dword_result_t XamNuiHudGetEngagedEnrollmentIndex_entry() {
  NuiHudState& hud = GetHudState();
  std::lock_guard<std::mutex> lock(hud.mutex);
  const uint32_t engaged = kernel_state()->nui()->GetEngagedTrackingId();
  if (hud.interpreted && engaged) {
    for (uint32_t i = 0; i < kSkeletonCount; ++i) {
      if (hud.tracking_ids[i] == engaged) {
        return hud.enrollment_indices[i];
      }
    }
  }
  return 0xFFFFFFFE;
}
DECLARE_XAM_EXPORT1(XamNuiHudGetEngagedEnrollmentIndex, kNone, kImplemented);

// Who the player with the tracking ID is (0x8180DA50): xam asks the title's
// identity engine (0x2C007) and finishes the overlapped later; here it's the
// user bound to the enrollment index the skeleton had in the last frame the HUD
// interpreted, or 0xFE for nobody. nuihud waits for 0x800703E5.
dword_result_t XamNuiIdentityIdentifyWithBiometric_entry(
    qword_t tracking_id, lpvoid_t overlapped, lpdword_t user_index,
    lpdword_t unk) {
  if (!XamNuiHudIsEnabled_entry()) {
    return X_E_FAIL;
  }
  uint32_t enrollment = NUIController::kNoEnrollment;
  {
    NuiHudState& hud = GetHudState();
    std::lock_guard<std::mutex> lock(hud.mutex);
    for (uint32_t i = 0; hud.interpreted && i < kSkeletonCount; ++i) {
      if (hud.tracking_ids[i] && hud.tracking_ids[i] == uint32_t(tracking_id)) {
        enrollment = hud.enrollment_indices[i];
      }
    }
  }
  uint32_t user = 0xFE;
  NUIController* nui = kernel_state()->nui();
  for (uint32_t i = 0; enrollment != NUIController::kNoEnrollment &&
                       i < XUserMaxUserCount;
       ++i) {
    if (nui->GetUserEnrollment(i) == enrollment) {
      user = i;
    }
  }
  XELOGD("Kinect: identified player {:08X} (enrollment {:08X}) as user {:02X}",
         uint32_t(tracking_id), enrollment, user);
  if (user_index) {
    *user_index = user;
  }
  if (!overlapped) {
    return X_E_SUCCESS;
  }
  kernel_state()->CompleteOverlappedImmediate(overlapped.guest_address(),
                                              X_ERROR_SUCCESS);
  return X_HRESULT_FROM_WIN32(X_ERROR_IO_PENDING);
}
DECLARE_XAM_EXPORT1(XamNuiIdentityIdentifyWithBiometric, kNone,
                    kImplemented);

// The user the system's HUD and Guide belong to (0x81692440 and its setter),
// 0xFF for none.
dword_result_t XamHudGetUserIndex_entry() {
  return kernel_state()->nui()->GetHudUserIndex();
}
DECLARE_XAM_EXPORT1(XamHudGetUserIndex, kNone, kImplemented);

void XamHudSetUserIndex_entry(dword_t user_index) {
  kernel_state()->nui()->SetHudUserIndex(user_index);
}
DECLARE_XAM_EXPORT1(XamHudSetUserIndex, kNone, kImplemented);

// xam hands the setting to its HUD (0x81692C48), which nothing here draws.
dword_result_t XamNuiHudEnableInputFilter_entry(dword_t enable) {
  if (XamNuiHudIsEnabled_entry()) {
    NuiHudState& hud = GetHudState();
    std::lock_guard<std::mutex> lock(hud.mutex);
    hud.input_filter = enable;
  }
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiHudEnableInputFilter, kNone, kSketchy);

}  // namespace xam
}  // namespace kernel
}  // namespace xe

DECLARE_XAM_EMPTY_REGISTER_EXPORTS(NUI);

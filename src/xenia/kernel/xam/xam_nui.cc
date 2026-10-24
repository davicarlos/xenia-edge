/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2025 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstring>
#include <string>
#include <utility>

#include "xenia/base/logging.h"
#include "xenia/emulator.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/nui.h"
#include "xenia/kernel/nui_audio.h"
#include "xenia/kernel/util/shim_utils.h"
#include "xenia/kernel/xam/user_profile.h"
#include "xenia/kernel/xam/user_settings.h"
#include "xenia/kernel/xam/user_tracker.h"
#include "xenia/kernel/xam/xam_private.h"
#include "xenia/xbox.h"

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

dword_result_t XamUserNuiGetUserIndex_entry(unknown_t unk, lpdword_t index) {
  return X_E_NO_SUCH_USER;
}
DECLARE_XAM_EXPORT1(XamUserNuiGetUserIndex, kNone, kStub);

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

dword_result_t XamUserNuiGetUserIndexForBind_entry(lpdword_t user_index) {
  if (!user_index) {
    return X_E_INVALIDARG;
  }

  return X_E_FAIL;
}
DECLARE_XAM_EXPORT1(XamUserNuiGetUserIndexForBind, kNone, kStub);

struct X_NUI_DEPTH_CALIBRATION {
  uint8_t data[0x30];
};
static_assert_size(X_NUI_DEPTH_CALIBRATION, 0x30);

dword_result_t XamNuiGetDepthCalibration_entry(
    pointer_t<X_NUI_DEPTH_CALIBRATION> depth_data) {
  // always cleared before new data written
  std::memset(depth_data, 0, sizeof(X_NUI_DEPTH_CALIBRATION));
  return X_STATUS_NO_SUCH_FILE | X_FACILITY_NT_BIT;
}
DECLARE_XAM_EXPORT1(XamNuiGetDepthCalibration, kNone, kStub);

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

/* XamNuiCamera Notes
   - most require message calls to xam in 0x0002Bxxx area
*/
dword_result_t XamNuiCameraTiltSetCallback_entry(dword_t callback) {
  kernel_state()->nui()->SetTiltCallback(callback);
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraTiltSetCallback, kNone, kStub);

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

dword_result_t XamNuiCameraRememberFloor_entry(unknown_t unk1, unknown_t unk2) {
  // Stores the floor plane the title found for the camera's automatic tilt.
  // Nothing uses it without the motor.
  if (!kernel_state()->nui()->NuiInitialized()) {
    return X_E_DEVICE_NOT_CONNECTED;
  }
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraRememberFloor, kNone, kStub);

dword_result_t XamNuiCameraTiltReportStatus_entry(unknown_t unk1,
                                                  unknown_t unk2) {
  // Lets the title's tilt code report its state to the system; nothing
  // listens without the motor.
  if (!kernel_state()->nui()->NuiInitialized()) {
    return X_E_DEVICE_NOT_CONNECTED;
  }
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraTiltReportStatus, kNone, kStub);

dword_result_t XamNuiCameraGetTiltControllerType_entry() {
  /* Notes:
     - Earlier decompile notes: DetroitDeviceRequest and
       (LZCOUNT(result) << 0x20) >> 0x25.
  */
  // TODO(boma): Get the controller type from connected device.
  // Currently only reports no controller or type 1.
  const bool kinect_initialized =
      kernel_state()->xconfig()->ReadSetting<uint32_t>(
          X_CONFIG_CATEGORY::XCONFIG_USER_CATEGORY, XCONFIG_USER_RETAIL_FLAGS) &
      X_RETAIL_FLAGS::KinectInitialized;
  return kinect_initialized;
}
DECLARE_XAM_EXPORT1(XamNuiCameraGetTiltControllerType, kNone, kStub);

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

dword_result_t XamNuiIsChatMicEnabled_entry() {
  /* Notes:
     - call ExGetXConfigSetting(7,9,&xconfig_nui,0x1c,local_40);
     - check xconfig_nui.flag & 0x8
     - return True or false based on result
  */
  return false;
}
DECLARE_XAM_EXPORT1(XamNuiIsChatMicEnabled, kNone, kImplemented);

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

dword_result_t XamNuiIdentityAbort_entry(dword_t unk) {
  if (XamNuiHudIsEnabled_entry() == false) {
    return X_E_FAIL;
  }
  // buffer [4]
  // buffer[0] = unk
  // return func(0xfe,0x2c00e,buffer,4,0)
  return X_E_FAIL;
}
DECLARE_XAM_EXPORT1(XamNuiIdentityAbort, kNone, kStub);

// Kinect ID enrollment of a signed-in user; nobody is ever enrolled, for which
// xam reports index -2.
dword_result_t XamUserNuiGetEnrollmentIndex_entry(dword_t user_index,
                                                  lpdword_t enrollment_index) {
  if (user_index >= XUserMaxUserCount || !enrollment_index) {
    return X_E_INVALIDARG;
  }
  *enrollment_index = 0xFFFFFFFE;
  return X_E_NO_SUCH_USER;
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
void XamNuiPlayerEngagementUpdate_entry(
    lpvoid_t skeleton_frame, dword_t engaged_tracking_id,
    pointer_t<X_NUI_PLAYER_ENGAGEMENT_UPDATE> engagement) {
  std::memset(engagement, 0, sizeof(X_NUI_PLAYER_ENGAGEMENT_UPDATE));
  engagement->engaged_index = 0xFFFFFFFF;
  engagement->engaged_user_index = 0xFE;
  engagement->engaged_enrollment_index = 0xFFFFFFFE;
  if (!skeleton_frame) {
    return;
  }
  constexpr float kTrackedBonus = 0.3f;
  constexpr float kMinimumDistance = 0.1f;
  uint8_t* frame = skeleton_frame.as<uint8_t*>();
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
DECLARE_XAM_EXPORT1(XamNuiPlayerEngagementUpdate, kNone, kImplemented);

}  // namespace xam
}  // namespace kernel
}  // namespace xe

DECLARE_XAM_EMPTY_REGISTER_EXPORTS(NUI);

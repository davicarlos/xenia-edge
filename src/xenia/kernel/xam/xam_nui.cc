/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2022 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

// xam's Kinect support, after xam.xex of system update 17559 (base
// 0x815F0000; addresses below are its functions). Titles bring their own NUI
// runtime: a skeleton tracker on the GPU, identity and tilt engines. xam
// holds the sensor's state, relays between the system and those engines, and
// runs the system HUD on the frames the title publishes (xam_nui_hud.cc).

#include <algorithm>
#include <cmath>
#include <cstring>

#include "xenia/base/cvar.h"
#include "xenia/base/filesystem.h"
#include "xenia/base/logging.h"
#include "xenia/cpu/processor.h"
#include "xenia/emulator.h"
#include "xenia/hid/kinect/usb_audio.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/kinect/kinect_device.h"
#include "xenia/kernel/nui.h"
#include "xenia/kernel/util/shim_utils.h"
#include "xenia/kernel/xam/profile_manager.h"
#include "xenia/kernel/xam/user_profile.h"
#include "xenia/kernel/xam/user_settings.h"
#include "xenia/kernel/xam/user_tracker.h"
#include "xenia/kernel/xam/xam_nui.h"
#include "xenia/kernel/xam/xam_private.h"
#include "xenia/kernel/xthread.h"
#include "xenia/xbox.h"

DECLARE_bool(use_50Hz_mode);
DECLARE_bool(kinect_chat_microphone);
DECLARE_bool(kinect_disabled);

namespace xe {
namespace kernel {
namespace xam {

// What xam answers for a missing tilt controller, sensor or user.
constexpr X_HRESULT kNuiNotConnected = X_E_DEVICE_NOT_CONNECTED;  // 0x8007048F
constexpr X_HRESULT kNuiNotBound = 0x80070525;
constexpr X_HRESULT kNuiNtSuccess = 0x10000000;  // HRESULT_FROM_NT(0)
// xam's title ID for the settings of the dashboard's own profile section.
constexpr uint32_t kDashboardTitleId = 0xFFFE07D1;

X_HRESULT xeXamNuiCallTitle(uint32_t callback, uint32_t message,
                            uint32_t* words, uint32_t word_count) {
  XThread* thread = XThread::GetCurrentThread();
  if (!callback || !thread) {
    return 0x8000FFFF;  // E_UNEXPECTED, as without a registered callback.
  }
  Memory* memory = kernel_state()->memory();
  uint32_t buffer = 0;
  if (word_count) {
    buffer = memory->SystemHeapAlloc(word_count * 4);
    if (!buffer) {
      return X_E_FAIL;
    }
    auto guest = memory->TranslateVirtual<xe::be<uint32_t>*>(buffer);
    for (uint32_t i = 0; i < word_count; ++i) {
      guest[i] = words[i];
    }
  }
  uint64_t args[] = {message, buffer};
  const auto result = X_HRESULT(kernel_state()->processor()->Execute(
      thread->thread_state(), callback, args, xe::countof(args)));
  if (word_count) {
    auto guest = memory->TranslateVirtual<xe::be<uint32_t>*>(buffer);
    for (uint32_t i = 0; i < word_count; ++i) {
      words[i] = guest[i];
    }
    memory->SystemHeapFree(buffer);
  }
  return result;
}

// --- The sensor --------------------------------------------------------------

struct X_NUI_DEVICE_STATUS {
  xe::be<uint32_t> status;
  // NTSTATUS of the camera and of the audio device; DEVICE_NOT_CONNECTED
  // when missing.
  xe::be<uint32_t> camera_status;
  xe::be<uint32_t> audio_status;
  xe::be<uint32_t> flags;
};
static_assert_size(X_NUI_DEVICE_STATUS, 0x10);

// The flags 0x817FC750 builds.
enum : uint32_t {
  kNuiStatusReady = 0x1,
  kNuiStatusMicrophones = 0x2,
  kNuiStatusError = 0x4,
  kNuiStatusOverheating = 0x8,
  kNuiStatusForcedOff = 0x20,
  kNuiStatusProblem = 0x40,
};

dword_result_t XamNuiGetDeviceStatus_entry(pointer_t<X_NUI_DEVICE_STATUS> status) {
  if (!status) {
    return X_E_INVALIDARG;
  }
  status.Zero();
  NUIController* nui = kernel_state()->nui();
  auto* device = nui->device();
  // A sensor switched off in the settings, or absent, reads as not connected
  // (0x817FC990).
  if (!nui->NuiInitialized() || !device->camera_present()) {
    status->camera_status = X_STATUS_DEVICE_NOT_CONNECTED;
    status->audio_status = X_STATUS_DEVICE_NOT_CONNECTED;
    return X_E_SUCCESS;
  }
  uint32_t flags = kNuiStatusReady;
  if (device->audio_present()) {
    // 4D53090F only starts its NUI audio with the microphones there.
    flags |= kNuiStatusMicrophones;
  } else {
    status->audio_status = X_STATUS_DEVICE_NOT_CONNECTED;
  }
  status->flags = flags;
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiGetDeviceStatus, kNone, kImplemented);

dword_result_t XamNuiIsDeviceReady_entry() {
  return kernel_state()->nui()->IsDeviceReady() ? 1 : 0;
}
DECLARE_XAM_EXPORT1(XamNuiIsDeviceReady, kNone, kImplemented);

// The Kinect Studio's hook (devkits only); without it xam fails every request.
dword_result_t XamXStudioRequest_entry(dword_t request, lpvoid_t data) {
  return X_E_FAIL;
}
DECLARE_XAM_EXPORT1(XamXStudioRequest, kNone, kImplemented);

dword_result_t XamIsNuiAutomationEnabled_entry() { return 0; }
DECLARE_XAM_EXPORT2(XamIsNuiAutomationEnabled, kNone, kImplemented,
                    kHighFrequency);

dword_result_t XamIsNatalPlaybackEnabled_entry() { return 0; }
DECLARE_XAM_EXPORT2(XamIsNatalPlaybackEnabled, kNone, kImplemented,
                    kHighFrequency);

dword_result_t XamIsNuiUIActive_entry() {
  return kernel_state()->xam_state()->IsUIActive() ? 1 : 0;
}
DECLARE_XAM_EXPORT1(XamIsNuiUIActive, kNone, kImplemented);

// 12 characters and a NUL; 4D53090F passes 0x11 bytes.
dword_result_t XamNuiGetDeviceSerialNumber_entry(lpvoid_t buffer, dword_t size) {
  if (!buffer || size < 13) {
    return X_E_INVALIDARG;
  }
  const std::string& serial = kernel_state()->nui()->device()->serial_number();
  if (serial.empty()) {
    return kNuiNotConnected;
  }
  char* out = buffer.as<char*>();
  std::memset(out, 0, size);
  std::memcpy(out, serial.data(), std::min<size_t>(serial.size(), 12));
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiGetDeviceSerialNumber, kNone, kImplemented);

// The troubleshooter's support code (0x817FA2E0): the sensor's mode and
// state, the kind and code of a problem (0: its firmware is the expected
// one), that firmware's version (fwversions.txt: 01.02.0810.00) and the serial
// number.
dword_result_t XamNuiGetSupportString_entry(lpvoid_t buffer, dword_t size) {
  if (!buffer || !size) {
    return X_E_INVALIDARG;
  }
  NUIController* nui = kernel_state()->nui();
  const bool present = nui->IsDeviceReady();
  std::string serial = present ? nui->device()->serial_number() : "";
  serial.resize(12, '0');
  const std::string text = fmt::format(
      "K:{:02d}:{:02d}:{:02d}:{:02d}:{:02d}.{:02d}.{:04d}.{:02d}:{:>12}",
      present ? 1 : 0, 0, 0, present ? 0 : 1, 1, 2, 810, 0, serial);
  const size_t length = std::min<size_t>(text.size(), size - 1);
  std::memcpy(buffer.as<char*>(), text.data(), length);
  buffer.as<char*>()[length] = '\0';
  return kNuiNtSuccess;
}
DECLARE_XAM_EXPORT1(XamNuiGetSupportString, kNone, kImplemented);

// Switches the sensor off or on (0x817FA170): bit 31 of the retail flags.
dword_result_t XamNuiSetForceDeviceOff_entry(dword_t off) {
  if (bool(off) == cvars::kinect_disabled) {
    return 0xD0000184;
  }
  auto* xconfig = kernel_state()->xconfig();
  uint32_t flags = xconfig->ReadSetting<uint32_t>(
      X_CONFIG_CATEGORY::XCONFIG_USER_CATEGORY, XCONFIG_USER_RETAIL_FLAGS);
  flags = off ? flags | X_RETAIL_FLAGS::KinectDisabled
              : flags & ~uint32_t(X_RETAIL_FLAGS::KinectDisabled);
  const uint32_t swapped = xe::byte_swap(flags);
  xconfig->WriteSetting(X_CONFIG_CATEGORY::XCONFIG_USER_CATEGORY,
                        XCONFIG_USER_RETAIL_FLAGS, &swapped);
  kernel_state()->BroadcastNotification(
      kXNotificationSystemNUIHardwareStatusChanged,
      kernel_state()->nui()->NuiInitialized());
  return X_STATUS_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiSetForceDeviceOff, kNone, kImplemented);

// The firmware updater's notices (0x817FA228 and 0x817FA270); a missing
// firmware file opens the troubleshooter.
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

// The microphone array calibration the console keeps in its
// MicArrayCalibrate.mec system package (0x817F8DE8), sent to the sensor.
dword_result_t XamNatalDeviceAudioCalibrate_entry() {
  return kernel_state()->nui()->device()->SendAudioCalibration() ? X_E_SUCCESS
                                                                 : 0x8AC80005;
}
DECLARE_XAM_EXPORT1(XamNatalDeviceAudioCalibrate, kNone, kImplemented);

// The voice chat microphone is the sensor's while bit 3 of
// XCONFIG_CONSOLE_NUI is clear (0x816F9E90 and 0x816F9ED8).
dword_result_t XamNuiIsChatMicEnabled_entry() {
  return cvars::kinect_chat_microphone ? 1 : 0;
}
DECLARE_XAM_EXPORT1(XamNuiIsChatMicEnabled, kNone, kImplemented);

void XamNuiEnableChatMic_entry(dword_t enable) {
  auto* xconfig = kernel_state()->xconfig();
  uint8_t data[0x1C];
  xconfig->ReadSetting(X_CONFIG_CATEGORY::XCONFIG_CONSOLE_CATEGORY,
                       XCONFIG_CONSOLE_NUI, data);
  uint32_t flags = xe::load_and_swap<uint32_t>(data);
  flags = enable ? flags & ~0x8u : flags | 0x8u;
  xe::store_and_swap<uint32_t>(data, flags);
  xconfig->WriteSetting(X_CONFIG_CATEGORY::XCONFIG_CONSOLE_CATEGORY,
                        XCONFIG_CONSOLE_NUI, data);
}
DECLARE_XAM_EXPORT1(XamNuiEnableChatMic, kNone, kImplemented);

// --- Depth calibration and camera information --------------------------------

// Kept in the storage root; the console keeps it in a system setting
// (0x816F9990 and 0x816F9A50). Both answer HRESULT_FROM_NT.
dword_result_t XamNuiGetDepthCalibration_entry(lpvoid_t calibration) {
  if (!calibration) {
    return X_E_INVALIDARG;
  }
  std::memset(calibration, 0, NUIController::kDepthCalibrationBytes);
  std::array<uint8_t, NUIController::kDepthCalibrationBytes> data;
  if (!kernel_state()->nui()->ReadDepthCalibration(data)) {
    return X_STATUS_NO_SUCH_FILE | X_FACILITY_NT_BIT;
  }
  std::memcpy(calibration, data.data(), data.size());
  return kNuiNtSuccess;
}
DECLARE_XAM_EXPORT1(XamNuiGetDepthCalibration, kNone, kImplemented);

dword_result_t XamNuiStoreDepthCalibration_entry(lpvoid_t calibration) {
  if (!calibration) {
    return X_E_INVALIDARG;
  }
  std::array<uint8_t, NUIController::kDepthCalibrationBytes> data;
  std::memcpy(data.data(), calibration, data.size());
  return kernel_state()->nui()->WriteDepthCalibration(data)
             ? kNuiNtSuccess
             : X_STATUS_UNSUCCESSFUL | X_FACILITY_NT_BIT;
}
DECLARE_XAM_EXPORT1(XamNuiStoreDepthCalibration, kNone, kImplemented);

// PsCam code 44 (0x816F9A48 and 0x817FCB40): the scale and offset the
// sensor's depth runs with.
dword_result_t XamNuiGetLoadedDepthCalibration_entry(lpdword_t calibration) {
  if (!calibration) {
    return X_E_INVALIDARG;
  }
  NUIController* nui = kernel_state()->nui();
  if (!nui->IsDeviceReady()) {
    return X_STATUS_DEVICE_NOT_CONNECTED | X_FACILITY_NT_BIT;
  }
  const auto words = nui->GetLoadedDepthCalibration();
  calibration[0] = words[0];
  calibration[1] = words[1];
  return kNuiNtSuccess;
}
DECLARE_XAM_EXPORT1(XamNuiGetLoadedDepthCalibration, kNone, kImplemented);

// PsCam code 46 (0x816F9B38): selector 0, a 4-byte value; selector 1, the
// color camera's automatic exposure table, entries of {scene brightness,
// frames per second, exposure in seconds}, the exposures multiples of the
// mains period so lights don't flicker. These are the tables titles fall back
// to (4D53090F at 0x821CAB48 and 0x821CABA0), for 60 Hz or, with
// use_50Hz_mode, 50 Hz mains.
struct X_NUI_EXPOSURE {
  xe::be<float> brightness;
  xe::be<float> frame_rate;
  xe::be<float> exposure;
};
static_assert_size(X_NUI_EXPOSURE, 0xC);

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
      // 34.19, as titles default to.
      *buffer.as<xe::be<uint32_t>*>() = 0x4208C083;
      *returned_size = 4;
      return X_E_SUCCESS;
    case 1: {
      const Exposure* table = cvars::use_50Hz_mode ? k50Hz : k60Hz;
      const uint32_t count = cvars::use_50Hz_mode ? uint32_t(std::size(k50Hz))
                                                  : uint32_t(std::size(k60Hz));
      if (size < count * sizeof(X_NUI_EXPOSURE)) {
        return X_E_INVALIDARG;
      }
      auto out = buffer.as<X_NUI_EXPOSURE*>();
      for (uint32_t i = 0; i < count; ++i) {
        out[i].brightness = table[i].brightness;
        out[i].frame_rate = table[i].frame_rate;
        out[i].exposure = table[i].exposure;
      }
      *returned_size = count * sizeof(X_NUI_EXPOSURE);
      return X_E_SUCCESS;
    }
    default:
      return X_E_INVALIDARG;
  }
}
DECLARE_XAM_EXPORT1(XamNuiGetTrueColorInfo, kNone, kSketchy);

// PsCam code 46 with selector 0x100 (0x816F9B90): fx, fy, cx and cy in
// pixels, in the order Dash.FieldCalibration builds its camera matrix from
// (0x9BD22C38). The Kinect's nominal color camera at 640x480.
dword_result_t XamNuiGetCameraIntrinsics_entry(lpvoid_t buffer, dword_t size,
                                               lpdword_t returned_size) {
  if (!buffer || !returned_size || size < 4 * sizeof(float)) {
    return X_E_INVALIDARG;
  }
  if (!kernel_state()->nui()->IsDeviceReady()) {
    return kNuiNotConnected;
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

// The sensor's fan (0x816F9BF0): byte 7 of property 0x838 (PsCam code 20),
// which isn't read here; 4D53090F only keeps it for its audio telemetry.
dword_result_t XamNuiGetFanRate_entry(lpvoid_t rate) {
  if (!rate) {
    return X_E_INVALIDARG;
  }
  if (!kernel_state()->nui()->IsDeviceReady()) {
    return X_STATUS_DEVICE_NOT_CONNECTED | X_FACILITY_NT_BIT;
  }
  *rate.as<uint8_t*>() = 0;
  return kNuiNtSuccess;
}
DECLARE_XAM_EXPORT1(XamNuiGetFanRate, kNone, kSketchy);

// --- Users and their Kinect ID enrollments -----------------------------------

dword_result_t XamUserNuiGetEnrollmentIndex_entry(dword_t user_index,
                                                  lpdword_t enrollment) {
  if (user_index >= NUIController::kUserCount || !enrollment) {
    return X_E_INVALIDARG;
  }
  const uint32_t index = kernel_state()->nui()->GetUserEnrollment(user_index);
  *enrollment = index;
  return index == NUIController::kNoEnrollment ? kNuiNotBound : X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamUserNuiGetEnrollmentIndex, kNone, kImplemented);

dword_result_t XamUserNuiGetUserIndex_entry(dword_t enrollment,
                                            lpdword_t user_index) {
  if (enrollment >= 8 || !user_index) {
    return X_E_INVALIDARG;
  }
  const uint32_t user = kernel_state()->nui()->FindEnrolledUser(enrollment);
  if (user >= NUIController::kUserCount) {
    return kNuiNotBound;
  }
  *user_index = user;
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamUserNuiGetUserIndex, kNone, kImplemented);

// The first signed-in profile without an enrollment.
dword_result_t XamUserNuiGetUserIndexForBind_entry(lpdword_t user_index) {
  if (!user_index) {
    return X_E_INVALIDARG;
  }
  for (uint32_t i = 0; i < NUIController::kUserCount; ++i) {
    if (kernel_state()->xam_state()->IsUserSignedIn(i) &&
        kernel_state()->nui()->GetUserEnrollment(i) ==
            NUIController::kNoEnrollment) {
      *user_index = i;
      return X_E_SUCCESS;
    }
  }
  return X_E_FAIL;
}
DECLARE_XAM_EXPORT1(XamUserNuiGetUserIndexForBind, kNone, kImplemented);

// The first user slot nobody is signed in to.
dword_result_t XamUserNuiGetUserIndexForSignin_entry(lpdword_t user_index) {
  if (!user_index) {
    return X_E_INVALIDARG;
  }
  for (uint32_t i = 0; i < NUIController::kUserCount; ++i) {
    if (!kernel_state()->xam_state()->IsUserSignedIn(i)) {
      *user_index = i;
      return X_E_SUCCESS;
    }
  }
  return X_E_FAIL;
}
DECLARE_XAM_EXPORT1(XamUserNuiGetUserIndexForSignin, kNone, kImplemented);

dword_result_t XamUserNuiBind_entry(dword_t enrollment, dword_t user_index) {
  if (enrollment >= 8 || user_index >= NUIController::kUserCount) {
    return X_E_INVALIDARG;
  }
  kernel_state()->nui()->SetUserEnrollment(user_index, enrollment);
  kernel_state()->BroadcastNotification(kXNotificationSystemNUIBindingChanged,
                                        user_index);
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamUserNuiBind, kNone, kImplemented);

dword_result_t XamUserNuiUnbind_entry(dword_t user_index) {
  if (user_index >= NUIController::kUserCount) {
    return X_E_INVALIDARG;
  }
  NUIController* nui = kernel_state()->nui();
  if (nui->GetUserEnrollment(user_index) == NUIController::kNoEnrollment) {
    return kNuiNotBound;
  }
  nui->SetUserEnrollment(user_index, NUIController::kNoEnrollment);
  kernel_state()->BroadcastNotification(kXNotificationSystemNUIBindingChanged,
                                        user_index);
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamUserNuiUnbind, kNone, kImplemented);

// Kinect sign-in is a dashboard profile setting (0x1004004E). Switching it
// off deletes the profile's Kinect ID data and tells the identity service
// (0x2A009).
dword_result_t XamUserNuiIsBiometricEnabled_entry(dword_t user_index) {
  if (user_index >= NUIController::kUserCount) {
    return 0;
  }
  UserProfile* user = kernel_state()->xam_state()->GetUserProfile(user_index);
  if (!user) {
    return 0;
  }
  const auto setting = kernel_state()->xam_state()->user_tracker()->GetSetting(
      user, kDashboardTitleId,
      uint32_t(UserSettingId::XPROFILE_NUI_BIOMETRIC_SIGNIN));
  if (!setting) {
    return 0;
  }
  const auto value = setting->get_host_data();
  if (const int32_t* enabled = std::get_if<int32_t>(&value)) {
    return *enabled != 0;
  }
  return 0;
}
DECLARE_XAM_EXPORT1(XamUserNuiIsBiometricEnabled, kNone, kImplemented);

X_RESULT xeXamWriteBiometricData(uint64_t xuid, uint32_t version,
                                 const uint8_t* data, uint32_t size);

dword_result_t XamUserNuiEnableBiometric_entry(dword_t user_index,
                                               dword_t enable) {
  if (user_index >= NUIController::kUserCount) {
    return X_E_INVALIDARG;
  }
  UserProfile* user = kernel_state()->xam_state()->GetUserProfile(user_index);
  if (!user) {
    return X_E_NO_SUCH_USER;
  }
  const UserSetting setting(UserSettingId::XPROFILE_NUI_BIOMETRIC_SIGNIN,
                            int32_t(enable ? 1 : 0));
  kernel_state()->xam_state()->user_tracker()->UpsertSetting(
      user->xuid(), kDashboardTitleId, &setting);
  if (!enable) {
    xeXamWriteBiometricData(user->xuid(), NUIController::kBiometricNone,
                            nullptr, 0);
  }
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamUserNuiEnableBiometric, kNone, kImplemented);

// A profile's Kinect ID data. On the console it is kept with the profile on
// Xbox Live (XMsg 0xFB 0xB0076 and 0xB0077, up to 0x32000 bytes); here in the
// profile's dashboard content (NuiBiometric.bin): the version, then the
// data. Titles
// cache it with xam (XamApp 0x2A007) and read it back from there first.
constexpr uint32_t kBiometricDataMax = 0x32000;

std::filesystem::path BiometricDataPath(uint64_t xuid) {
  return kernel_state()->xam_state()->profile_manager()->GetProfileContentPath(
             xuid, kDashboardTitleId) /
         "NuiBiometric.bin";
}

X_RESULT xeXamWriteBiometricData(uint64_t xuid, uint32_t version,
                                 const uint8_t* data, uint32_t size) {
  const auto path = BiometricDataPath(xuid);
  if (!size || version == NUIController::kBiometricNone) {
    std::error_code error;
    std::filesystem::remove(path, error);
    kernel_state()->nui()->SetCachedBiometricData(xuid, {});
    return X_ERROR_SUCCESS;
  }
  std::error_code error;
  std::filesystem::create_directories(path.parent_path(), error);
  FILE* file = xe::filesystem::OpenFile(path, "wb");
  if (!file) {
    return X_ERROR_FUNCTION_FAILED;
  }
  const uint32_t be_version = xe::byte_swap(version);
  const bool written = std::fwrite(&be_version, 4, 1, file) == 1 &&
                       std::fwrite(data, 1, size, file) == size;
  std::fclose(file);
  XELOGI("Kinect ID: saved {} bytes of data (version {:08X}) for {:016X}",
         size, version, xuid);
  return written ? X_ERROR_SUCCESS : X_ERROR_FUNCTION_FAILED;
}

dword_result_t XamReadBiometricData_entry(qword_t xuid, lpdword_t version,
                                          lpvoid_t buffer, lpdword_t size,
                                          pointer_t<XAM_OVERLAPPED> overlapped) {
  if (!version || !buffer || !size) {
    return X_ERROR_INVALID_PARAMETER;
  }
  X_RESULT result = X_ERROR_NOT_FOUND;
  if (FILE* file = xe::filesystem::OpenFile(BiometricDataPath(xuid), "rb")) {
    uint32_t be_version = 0;
    if (std::fread(&be_version, 4, 1, file) == 1) {
      const uint32_t capacity = std::min<uint32_t>(*size, kBiometricDataMax);
      const size_t read = std::fread(buffer, 1, capacity, file);
      *version = xe::byte_swap(be_version);
      *size = uint32_t(read);
      result = X_ERROR_SUCCESS;
    }
    std::fclose(file);
  }
  if (result != X_ERROR_SUCCESS) {
    *version = NUIController::kBiometricNone;
    *size = 0;
  }
  if (overlapped) {
    kernel_state()->CompleteOverlappedImmediate(overlapped, result);
    return X_ERROR_IO_PENDING;
  }
  return result;
}
DECLARE_XAM_EXPORT1(XamReadBiometricData, kNone, kImplemented);

dword_result_t XamWriteBiometricData_entry(qword_t xuid, lpdword_t version,
                                           lpvoid_t buffer, lpdword_t size,
                                           pointer_t<XAM_OVERLAPPED> overlapped) {
  if (!version || !buffer || !size) {
    return X_ERROR_INVALID_PARAMETER;
  }
  const uint32_t length = std::min<uint32_t>(*size, kBiometricDataMax);
  const X_RESULT result =
      xeXamWriteBiometricData(xuid, *version, buffer.as<uint8_t*>(), length);
  if (overlapped) {
    kernel_state()->CompleteOverlappedImmediate(overlapped, result);
    return X_ERROR_IO_PENDING;
  }
  return result;
}
DECLARE_XAM_EXPORT1(XamWriteBiometricData, kNone, kImplemented);

// --- Tilt --------------------------------------------------------------------

// UsbdGetNatalHardwareVersion (0x816FA088): 1 for model 1414's motor hub
// (DetroitDeviceRequest), 2 for model 1473, whose audio firmware drives the
// motor (McaDeviceRequest), 0 without a sensor. Without the motor itself, a
// recording, the tilt is simulated as a 1473's.
dword_result_t XamNuiCameraGetTiltControllerType_entry() {
  return kernel_state()->nui()->IsDeviceReady() ? 2 : 0;
}
DECLARE_XAM_EXPORT1(XamNuiCameraGetTiltControllerType, kNone, kImplemented);

static bool HasTiltController() {
  return XamNuiCameraGetTiltControllerType_entry() != 0;
}

// The status titles read: 'XtSs', 0x50 bytes, built from the motor's 0x48
// byte block (0x816FAA60). ElevationGetAngle takes the angle from the
// gravity's Y and Z (0x816FAF68).
constexpr uint32_t kTiltStatusSignature = 0x58745373;  // 'XtSs'

struct X_NUI_TILT_STATUS {
  xe::be<uint32_t> size;
  xe::be<uint32_t> signature;
  xe::be<uint32_t> unknown_08[6];
  // In the accelerometer's counts, about 819 for 1 g.
  xe::be<int32_t> gravity[3];
  // Bits 0 and 1: the motor is moving.
  xe::be<uint32_t> motor_state;
  xe::be<uint32_t> unknown_30[8];
};
static_assert_size(X_NUI_TILT_STATUS, 0x50);

dword_result_t XamNuiCameraTiltGetStatus_entry(
    pointer_t<X_NUI_TILT_STATUS> status) {
  if (!status || status->signature != kTiltStatusSignature ||
      status->size < sizeof(X_NUI_TILT_STATUS)) {
    return X_E_INVALIDARG;
  }
  if (!HasTiltController()) {
    return kNuiNotConnected;
  }
  status.Zero();
  status->size = sizeof(X_NUI_TILT_STATUS);
  status->signature = kTiltStatusSignature;
  NUIController* nui = kernel_state()->nui();
  hid::kinect::UsbAudio::State motor;
  if (nui->device()->ReadMotorState(motor)) {
    for (size_t i = 0; i < 3; ++i) {
      status->gravity[i] = motor.accelerometer[i];
    }
    status->motor_state = motor.flags & 3;
  } else {
    // A camera that doesn't move, at the angle last set.
    constexpr double kOneG = 819.0;
    const double angle = nui->GetElevationAngle() * M_PI / 180.0;
    status->gravity[1] = int32_t(std::lround(kOneG * std::cos(angle)));
    status->gravity[2] = int32_t(std::lround(kOneG * std::sin(angle)));
  }
  return kNuiNtSuccess;
}
DECLARE_XAM_EXPORT2(XamNuiCameraTiltGetStatus, kNone, kImplemented,
                    kHighFrequency);

dword_result_t XamNuiCameraElevationGetAngle_entry(lpdword_t angle,
                                                   lpdword_t motor_state) {
  if (!angle) {
    return X_E_INVALIDARG;
  }
  X_NUI_TILT_STATUS* status = kernel_memory()->TranslateVirtual<X_NUI_TILT_STATUS*>(
      kernel_memory()->SystemHeapAlloc(sizeof(X_NUI_TILT_STATUS)));
  const uint32_t status_ptr = kernel_memory()->HostToGuestVirtual(status);
  std::memset(status, 0, sizeof(*status));
  status->size = sizeof(X_NUI_TILT_STATUS);
  status->signature = kTiltStatusSignature;
  const X_HRESULT result = XamNuiCameraTiltGetStatus_entry(
      pointer_t<X_NUI_TILT_STATUS>(status_ptr));
  if (XSUCCEEDED(result)) {
    const double degrees =
        std::atan2(double(int32_t(status->gravity[2])),
                   double(int32_t(status->gravity[1]))) *
        57.2957795;
    *angle = uint32_t(int32_t(std::lround(degrees)));
    if (motor_state) {
      *motor_state = status->motor_state & 3;
    }
  }
  kernel_memory()->SystemHeapFree(status_ptr);
  return result;
}
DECLARE_XAM_EXPORT2(XamNuiCameraElevationGetAngle, kNone, kImplemented,
                    kHighFrequency);

dword_result_t XamNuiCameraElevationSetAngle_entry(int_t angle) {
  if (!HasTiltController()) {
    return kNuiNotConnected;
  }
  kernel_state()->nui()->SetElevationAngle(angle);
  return kNuiNtSuccess;
}
DECLARE_XAM_EXPORT1(XamNuiCameraElevationSetAngle, kNone, kImplemented);

// Stops the motor, then tells the title's tilt engine (0x2B009).
dword_result_t XamNuiCameraElevationStopMovement_entry() {
  if (!HasTiltController()) {
    return kNuiNotConnected;
  }
  NUIController* nui = kernel_state()->nui();
  nui->StopElevation();
  if (nui->tilt_engine()) {
    xeXamNuiCallTitle(nui->tilt_engine(), 0x2B009, nullptr, 0);
  }
  return kNuiNtSuccess;
}
DECLARE_XAM_EXPORT1(XamNuiCameraElevationStopMovement, kNone, kImplemented);

// The floor plane the title found (0x816FA618), by value: two floats in each
// register. 4D53090F sends it about 25 times a second.
dword_result_t XamNuiCameraRememberFloor_entry(qword_t normal_xy,
                                               qword_t normal_z_distance,
                                               dword_t reset) {
  if (!HasTiltController()) {
    return kNuiNotConnected;
  }
  auto high = [](uint64_t v) {
    return xe::memory::Reinterpret<float>(uint32_t(v >> 32));
  };
  auto low = [](uint64_t v) {
    return xe::memory::Reinterpret<float>(uint32_t(v));
  };
  kernel_state()->nui()->SetRememberedFloor(
      {high(normal_xy), low(normal_xy), high(normal_z_distance),
       low(normal_z_distance)});
  return kNuiNtSuccess;
}
DECLARE_XAM_EXPORT2(XamNuiCameraRememberFloor, kNone, kImplemented,
                    kHighFrequency);

// Model 1414's motor takes flags (0xF0510); model 1473's ignores them
// (0x816FB000).
dword_result_t XamNuiCameraSetFlags_entry(qword_t flags, dword_t mask) {
  if (!HasTiltController()) {
    return kNuiNotConnected;
  }
  NUIController* nui = kernel_state()->nui();
  nui->SetCameraFlags((nui->GetCameraFlags() & ~uint32_t(mask)) |
                      (uint32_t(flags) & uint32_t(mask)));
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraSetFlags, kNone, kImplemented);

// The system's requests to the title's tilt engine, which XamApp relays
// (0x2B005, 0x2B00A, 0x2B00B): the Kinect Tuner asks for them.
dword_result_t XamNuiCameraElevationAutoTilt_entry(double_t distance) {
  NUIController* nui = kernel_state()->nui();
  if (!HasTiltController()) {
    return kNuiNotConnected;
  }
  uint32_t words[] = {xe::memory::Reinterpret<uint32_t>(float(distance))};
  return xeXamNuiCallTitle(nui->tilt_engine(), 0x2B005, words, 1);
}
DECLARE_XAM_EXPORT1(XamNuiCameraElevationAutoTilt, kNone, kImplemented);

dword_result_t XamNuiCameraElevationReverseAutoTilt_entry(dword_t unk) {
  NUIController* nui = kernel_state()->nui();
  if (!HasTiltController()) {
    return kNuiNotConnected;
  }
  uint32_t words[] = {unk};
  return xeXamNuiCallTitle(nui->tilt_engine(), 0x2B00A, words, 1);
}
DECLARE_XAM_EXPORT1(XamNuiCameraElevationReverseAutoTilt, kNone, kImplemented);

// The Tuner checks whether the player's head is cut off with
// (2.1336, 1.3716, 0x20): the distance to stay within, the height to aim at,
// and 0x20 to only check, without moving.
dword_result_t XamNuiCameraAdjustTilt_entry(double_t distance, double_t height,
                                            dword_t flags) {
  NUIController* nui = kernel_state()->nui();
  if (!HasTiltController()) {
    return kNuiNotConnected;
  }
  uint32_t words[] = {flags,
                      xe::memory::Reinterpret<uint32_t>(float(distance)),
                      xe::memory::Reinterpret<uint32_t>(float(height))};
  return xeXamNuiCallTitle(nui->tilt_engine(), 0x2B00B, words, 3);
}
DECLARE_XAM_EXPORT1(XamNuiCameraAdjustTilt, kNone, kImplemented);

dword_result_t XamNuiCameraTiltSetCallback_entry(dword_t callback) {
  kernel_state()->nui()->set_tilt_engine(callback);
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraTiltSetCallback, kNone, kImplemented);

dword_result_t XamNuiCameraElevationSetCallback_entry(dword_t callback) {
  kernel_state()->nui()->set_tilt_status_listener(callback);
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraElevationSetCallback, kNone, kImplemented);

// The title's tilt engine reports 0x2B006 (0x10 bytes) or 0x2B00C (0xE0
// bytes) to the listener (0x816FA1A8); other messages come back as they are.
dword_result_t XamNuiCameraTiltReportStatus_entry(dword_t message,
                                                  lpvoid_t status) {
  if (message != 0x2B006 && message != 0x2B00C) {
    return message.value();
  }
  const uint32_t listener = kernel_state()->nui()->tilt_status_listener();
  XThread* thread = XThread::GetCurrentThread();
  if (listener && thread) {
    uint64_t args[] = {message, status.guest_address()};
    kernel_state()->processor()->Execute(thread->thread_state(), listener,
                                         args, xe::countof(args));
  }
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraTiltReportStatus, kNone, kImplemented);

}  // namespace xam
}  // namespace kernel
}  // namespace xe

DECLARE_XAM_EMPTY_REGISTER_EXPORTS(NUI);

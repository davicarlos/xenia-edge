/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

// Kinect ID, and XamApp's NUI messages.
//
// The face recognition is the title's: its identity engine registers a
// callback with xam (0x2C009), and xam's identity service hands it the
// system's requests (0x8180E750): enrollment info (0x2C013), unenroll
// (0x2C012), the face picture (0x2C011), identify (0x2C006-0x2C008), abort
// (0x2C00E), enroll for sign-in (0x2C010). Titles keep the Kinect ID data of
// each profile with xam (0x2A007, 0x2A008) and with the profile
// (XamRead/WriteBiometricData).

#include <chrono>
#include <cstring>

#include "xenia/base/cvar.h"
#include "xenia/base/logging.h"
#include "xenia/cpu/processor.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/nui.h"
#include "xenia/kernel/util/shim_utils.h"
#include "xenia/kernel/xam/xam_nui.h"
#include "xenia/kernel/xam/xam_private.h"
#include "xenia/kernel/xthread.h"
#include "xenia/xbox.h"

DEFINE_bool(kinect_log_hands, false,
            "Log every skeleton frame the title's runtime publishes: each "
            "skeleton's state and its hands and shoulder center, with the "
            "time since the last frame. Titles' hand cursors (4D53093B) only "
            "follow a fully tracked skeleton.",
            "HID");

namespace xe {
namespace kernel {
namespace xam {

dword_result_t XamShowNuiTroubleshooterUI_entry(dword_t user_index,
                                                dword_t tracking_id,
                                                dword_t flags);

namespace {

// The identity service's answer without a callback.
constexpr X_HRESULT kNoIdentityEngine = 0x8000FFFF;
// The answer 0x2C013 gives when the identity engine has nothing enrolled,
// which ends an identification.
constexpr X_HRESULT kNoEnrollments = 0x83010005;

X_HRESULT CallIdentityEngine(uint32_t message, uint32_t* words,
                             uint32_t word_count) {
  const uint32_t engine = kernel_state()->nui()->identity_engine();
  if (!engine) {
    return kNoIdentityEngine;
  }
  return xeXamNuiCallTitle(engine, message, words, word_count);
}

}  // namespace

// --- Exports -----------------------------------------------------------------

// 8 random bytes, made on first use (0x8180DAC8).
qword_result_t XamNuiIdentityGetSessionId_entry() {
  return kernel_state()->nui()->GetSessionId();
}
DECLARE_XAM_EXPORT1(XamNuiIdentityGetSessionId, kNone, kImplemented);

// 0x2C013: {0x10, enrollment} in, two words out (0x8180E048).
dword_result_t XamNuiIdentityGetEnrollmentInfo_entry(dword_t enrollment,
                                                     lpdword_t first,
                                                     lpdword_t second) {
  uint32_t words[4] = {0x10, enrollment, 0, 0};
  const X_HRESULT result = CallIdentityEngine(0x2C013, words, 4);
  if (XSUCCEEDED(result)) {
    if (words[0] != 0x10) {
      return kNoIdentityEngine;
    }
    if (first) {
      *first = words[2];
    }
    if (second) {
      *second = words[3];
    }
  }
  return result;
}
DECLARE_XAM_EXPORT1(XamNuiIdentityGetEnrollmentInfo, kNone, kImplemented);

// 0x2C012: {8, enrollment} (0x8180DFE0).
void XamNuiIdentityUnenroll_entry(dword_t enrollment) {
  uint32_t words[2] = {8, enrollment};
  CallIdentityEngine(0x2C012, words, 2);
}
DECLARE_XAM_EXPORT1(XamNuiIdentityUnenroll, kNone, kImplemented);

// The face picture of the last identification (0x8180DF08): 0x2C011 gives
// {0x18, width, height, format, unused, address}, and xam points a texture
// header at it (XGSetTextureHeaderEx, one level, then XGOffsetResourceAddress).
dword_result_t XamNuiIdentityGetColorTexture_entry(lpvoid_t texture) {
  if (!texture) {
    return X_E_INVALIDARG;
  }
  uint32_t words[6] = {0x18, 0, 0, 0, 0, 0};
  const X_HRESULT result = CallIdentityEngine(0x2C011, words, 6);
  if (XFAILED(result)) {
    return result;
  }
  if (words[0] != 0x18) {
    return kNoIdentityEngine;
  }
  const uint32_t width = words[1], height = words[2], format = words[3],
                 address = words[5];
  // D3DTexture: Common (a texture), its reference count, the fences, the
  // fetch constant from +0x1C (dword 0: a 2D texture with the format's signs
  // and a pitch of whole 32-texel tiles; dword 1: format, endianness and
  // base address; dword 2: size; dword 3: swizzle and number format).
  auto header = texture.as<xe::be<uint32_t>*>();
  std::memset(header, 0, 13 * 4);
  header[0] = 3 | 0x200000;
  header[1] = 1;
  header[4] = 0xFFFF0000;
  header[5] = 0xFFFF0000;
  const uint32_t pitch_tiles = (width + 31) / 32;
  header[7] = 2 | ((format >> 9) & 0xFF) << 2 | pitch_tiles << 22 |
              (format & 0x100) << 23;
  header[8] = (format & 0x3F) | (format & 0xC0) | (address & 0xFFFFF000);
  header[9] = (width - 1) | (height - 1) << 13;
  header[10] = (format >> 17) & 0x1FFF;
  header[12] = 1 << 9;
  return result;
}
DECLARE_XAM_EXPORT1(XamNuiIdentityGetColorTexture, kNone, kSketchy);

// Who the player with the tracking ID is (0x8180DA50). xam has the identity
// engine match the face (0x2C007) and finishes the overlapped with the
// result; here it's the user bound to the enrollment the skeleton carried in
// the last frame the HUD saw, or 0xFE for nobody. nuihud waits for
// 0x800703E5.
dword_result_t XamNuiIdentityIdentifyWithBiometric_entry(
    qword_t tracking_id, pointer_t<XAM_OVERLAPPED> overlapped,
    lpdword_t user_index, lpdword_t unk) {
  if (!kernel_state()->nui()->IsHudEnabled()) {
    return X_E_FAIL;
  }
  const uint32_t enrollment = xeXamNuiHudEnrollmentOf(uint32_t(tracking_id));
  NUIController* nui = kernel_state()->nui();
  uint32_t user = 0xFE;
  const uint32_t bound = nui->FindEnrolledUser(enrollment);
  if (bound < NUIController::kUserCount) {
    user = bound;
  }
  if (user_index) {
    *user_index = user;
  }
  if (!overlapped) {
    return X_E_SUCCESS;
  }
  kernel_state()->CompleteOverlappedImmediate(overlapped, X_ERROR_SUCCESS);
  return X_HRESULT_FROM_WIN32(X_ERROR_IO_PENDING);
}
DECLARE_XAM_EXPORT1(XamNuiIdentityIdentifyWithBiometric, kNone, kSketchy);

// 0x2C008, an asynchronous task of xam's identity service whose result isn't
// understood (0x8180D9E8): no quality warnings.
dword_result_t XamNuiIdentityGetQualityFlags_entry(
    dword_t unk, pointer_t<XAM_OVERLAPPED> overlapped) {
  if (!kernel_state()->nui()->IsHudEnabled()) {
    return X_E_FAIL;
  }
  if (!overlapped) {
    return X_E_SUCCESS;
  }
  kernel_state()->CompleteOverlappedImmediate(overlapped, X_ERROR_SUCCESS);
  return X_HRESULT_FROM_WIN32(X_ERROR_IO_PENDING);
}
DECLARE_XAM_EXPORT1(XamNuiIdentityGetQualityFlags, kNone, kSketchy);

// Sends two words to the sensor (PsCam code 40, 0x816F9B30); what they set
// isn't known, and nothing here uses them.
dword_result_t XamNuiIdentityGetQualityFlagsMessage_entry(lpdword_t words) {
  if (!words) {
    return X_E_INVALIDARG;
  }
  return 0x10000000;
}
DECLARE_XAM_EXPORT1(XamNuiIdentityGetQualityFlagsMessage, kNone, kSketchy);

// 0x2C00E (0x8180D910).
dword_result_t XamNuiIdentityAbort_entry(dword_t request) {
  if (!kernel_state()->nui()->IsHudEnabled()) {
    return X_E_FAIL;
  }
  uint32_t words[] = {request};
  const X_HRESULT result = CallIdentityEngine(0x2C00E, words, 1);
  return result == kNoIdentityEngine ? X_E_SUCCESS : result;
}
DECLARE_XAM_EXPORT1(XamNuiIdentityAbort, kNone, kImplemented);

// 0x2C010: {user, p2, p4}, finishing the overlapped (0x8180D970).
dword_result_t XamNuiIdentityEnrollForSignIn_entry(
    dword_t user_index, dword_t unk2, pointer_t<XAM_OVERLAPPED> overlapped,
    dword_t unk4) {
  if (!kernel_state()->nui()->IsHudEnabled()) {
    return X_E_FAIL;
  }
  uint32_t words[] = {user_index, unk2, unk4};
  const X_HRESULT result = CallIdentityEngine(0x2C010, words, 3);
  if (!overlapped) {
    return result;
  }
  kernel_state()->CompleteOverlappedImmediate(
      overlapped, XSUCCEEDED(result) ? X_ERROR_SUCCESS : X_ERROR_FUNCTION_FAILED);
  return X_HRESULT_FROM_WIN32(X_ERROR_IO_PENDING);
}
DECLARE_XAM_EXPORT1(XamNuiIdentityEnrollForSignIn, kNone, kImplemented);

// Whose the system HUD and Guide are (0x81692440 and its setter), 0xFF for
// none.
dword_result_t XamHudGetUserIndex_entry() {
  return kernel_state()->nui()->GetHudUserIndex();
}
DECLARE_XAM_EXPORT1(XamHudGetUserIndex, kNone, kImplemented);

void XamHudSetUserIndex_entry(dword_t user_index) {
  kernel_state()->nui()->SetHudUserIndex(user_index);
}
DECLARE_XAM_EXPORT1(XamHudSetUserIndex, kNone, kImplemented);

// --- XamApp messages ---------------------------------------------------------

namespace {

Memory* memory() { return kernel_state()->memory(); }

template <typename T>
T* Guest(uint32_t address) {
  return memory()->TranslateVirtual<T*>(address);
}

// kinect_log_hands: a line a frame. Joint states are 0 not tracked, 1
// inferred, 2 tracked; the shoulder center is what 4D53093B's cursor measures
// the hands from (0x826ADB70).
void LogHands(const NuiSkeletonSnapshot& snapshot) {
  constexpr uint32_t kShoulderCenter = 2, kHandLeft = 7, kHandRight = 11;
  static auto last = std::chrono::steady_clock::now();
  const auto now = std::chrono::steady_clock::now();
  const double interval_ms =
      std::chrono::duration<double, std::milli>(now - last).count();
  last = now;
  bool anyone = false;
  for (uint32_t i = 0; i < NuiSkeletonSnapshot::kSkeletonCount; ++i) {
    const auto& s = snapshot.skeletons[i];
    if (s.state == NuiSkeletonSnapshot::TrackingState::kNotTracked) {
      continue;
    }
    anyone = true;
    const auto& l = s.joints[kHandLeft];
    const auto& r = s.joints[kHandRight];
    const auto& c = s.joints[kShoulderCenter];
    XELOGI(
        "Kinect hands: frame {} +{:.0f} ms skeleton {} state {} id {:08X} "
        "left {} ({:.2f} {:.2f} {:.2f}) right {} ({:.2f} {:.2f} {:.2f}) "
        "shoulders {} ({:.2f} {:.2f} {:.2f})",
        snapshot.frame_number, interval_ms, i, uint32_t(s.state),
        s.tracking_id, s.joint_states[kHandLeft], l[0], l[1], l[2],
        s.joint_states[kHandRight], r[0], r[1], r[2],
        s.joint_states[kShoulderCenter], c[0], c[1], c[2]);
  }
  if (!anyone) {
    XELOGI("Kinect hands: frame {} +{:.0f} ms nobody", snapshot.frame_number,
           interval_ms);
  }
}

// 0x2B0xx, xam's dispatcher 0x816F9CB8 (switch on message − 0x2B001).
X_HRESULT DispatchTitleMessage(uint32_t message, uint32_t buffer_ptr,
                               uint8_t* buffer) {
  NUIController* nui = kernel_state()->nui();
  switch (message) {
    case 0x2B001:
      // The HUD's input delay (0x816AC460); nothing waits on it here.
      return X_E_SUCCESS;
    case 0x2B002: {
      // A skeleton frame of the title's runtime: the frame observer (the
      // Kinect Tuner, 0x816F9FC0), then the system HUD. Sent with
      // XMsgInProcessCall and a length of 0; 0xAE0 bytes by contract.
      if (!buffer ||
          xe::load_and_swap<uint64_t>(buffer) != NUIController::kRuntimeVersion) {
        return X_E_FAIL;
      }
      NuiSkeletonSnapshot snapshot;
      static uint64_t frame_number = 0;
      snapshot.frame_number = ++frame_number;
      for (uint32_t i = 0; i < 4; ++i) {
        snapshot.floor_plane[i] =
            xe::load_and_swap<float>(buffer + kNuiFrameFloorPlane + i * 4);
      }
      for (uint32_t i = 0; i < kNuiSkeletonCount; ++i) {
        const uint8_t* source =
            buffer + kNuiFrameSkeletons + i * kNuiSkeletonSize;
        auto& skeleton = snapshot.skeletons[i];
        skeleton.state = NuiSkeletonSnapshot::TrackingState(
            xe::load_and_swap<uint32_t>(source + kNuiSkeletonState));
        skeleton.tracking_id =
            xe::load_and_swap<uint32_t>(source + kNuiSkeletonTrackingId);
        for (uint32_t c = 0; c < 3; ++c) {
          skeleton.position[c] = xe::load_and_swap<float>(
              source + kNuiSkeletonPosition + c * 4);
        }
        for (uint32_t j = 0; j < NuiSkeletonSnapshot::kJointCount; ++j) {
          for (uint32_t c = 0; c < 3; ++c) {
            skeleton.joints[j][c] = xe::load_and_swap<float>(
                source + kNuiSkeletonJoints + j * 16 + c * 4);
          }
          skeleton.joint_states[j] = xe::load_and_swap<uint32_t>(
              source + kNuiSkeletonJointStates + j * 4);
        }
      }
      nui->SetSkeletonSnapshot(snapshot);
      if (cvars::kinect_log_hands) {
        LogHands(snapshot);
      }
      // Now and then, whether the tracker sees anyone.
      if (snapshot.frame_number % 60 == 0) {
        uint32_t tracked = 0, position_only = 0;
        for (const auto& skeleton : snapshot.skeletons) {
          tracked += skeleton.state ==
                     NuiSkeletonSnapshot::TrackingState::kTracked;
          position_only += skeleton.state ==
                           NuiSkeletonSnapshot::TrackingState::kPositionOnly;
        }
        XELOGI("Kinect: skeleton frame {}: {} tracked, {} position only",
               snapshot.frame_number, tracked, position_only);
      }
      if (const uint32_t observer = nui->frame_observer()) {
        if (XThread* thread = XThread::GetCurrentThread()) {
          uint64_t args[] = {buffer_ptr};
          kernel_state()->processor()->Execute(thread->thread_state(),
                                               observer, args,
                                               xe::countof(args));
        }
      }
      return xeXamNuiHudInterpretFrame(buffer);
    }
    case 0x2B003: {
      // The title's runtime starts: {xam version, XTL version, flags}.
      if (!buffer) {
        return X_E_INVALIDARG;
      }
      const uint64_t xam_version = xe::load_and_swap<uint64_t>(buffer);
      const uint64_t xtl_version = xe::load_and_swap<uint64_t>(buffer + 8);
      const uint32_t flags = xe::load_and_swap<uint32_t>(buffer + 16);
      X_HRESULT result = X_E_FAIL;
      if (xam_version == NUIController::kRuntimeVersion) {
        nui->InitializeTitle(xam_version, xtl_version, flags);
        // Bit 31 leaves the HUD off, which xam answers with E_FAIL.
        if (!(flags & 0x80000000)) {
          result = X_E_SUCCESS;
        }
      }
      // Without bit 30, a sensor that isn't ready opens the troubleshooter
      // (0x200000) for a console set up for Kinect.
      if (!(flags & 0x40000000) && nui->NuiInitialized() &&
          !nui->IsDeviceReady()) {
        XamShowNuiTroubleshooterUI_entry(0xFF, 0, 0x200000);
      }
      return result;
    }
    case 0x2B004:
      nui->ShutdownTitle();
      return X_E_SUCCESS;
    case 0x2B005:
    case 0x2B009:
    case 0x2B00A:
    case 0x2B00B: {
      // The system's tilt requests, to the title's tilt engine (0x816FA970).
      const uint32_t engine = nui->tilt_engine();
      if (!engine) {
        return kNoIdentityEngine;
      }
      if (XThread* thread = XThread::GetCurrentThread()) {
        uint64_t args[] = {message, buffer_ptr};
        return X_HRESULT(kernel_state()->processor()->Execute(
            thread->thread_state(), engine, args, xe::countof(args)));
      }
      return X_E_FAIL;
    }
    case 0x2B006:
    case 0x2B00C: {
      // The tilt engine's status, to the listener (0x816FA988).
      const uint32_t listener = nui->tilt_status_listener();
      XThread* thread = XThread::GetCurrentThread();
      if (listener && thread) {
        uint64_t args[] = {message, buffer_ptr};
        kernel_state()->processor()->Execute(thread->thread_state(), listener,
                                             args, xe::countof(args));
      }
      return X_E_SUCCESS;
    }
    case 0x2B007:
      if (!buffer) {
        return X_E_INVALIDARG;
      }
      nui->set_tilt_engine(xe::load_and_swap<uint32_t>(buffer));
      return X_E_SUCCESS;
    case 0x2B00D:
      if (!buffer) {
        return X_E_INVALIDARG;
      }
      nui->set_tilt_status_listener(xe::load_and_swap<uint32_t>(buffer));
      return X_E_SUCCESS;
    case 0x2B00E:
      if (!buffer) {
        return X_E_INVALIDARG;
      }
      nui->set_frame_observer(xe::load_and_swap<uint32_t>(buffer));
      return X_E_SUCCESS;
    default:
      return kNoIdentityEngine;
  }
}

}  // namespace

bool xeXamNuiDispatchMessage(uint32_t message, uint32_t buffer_ptr,
                             uint32_t buffer_length, X_HRESULT& result) {
  uint8_t* buffer = buffer_ptr ? Guest<uint8_t>(buffer_ptr) : nullptr;
  NUIController* nui = kernel_state()->nui();
  if (message >= 0x2B001 && message <= 0x2B00E) {
    result = DispatchTitleMessage(message, buffer_ptr, buffer);
    return true;
  }
  switch (message) {
    case 0x2A004:
    case 0x2A005:
    case 0x2A006:
      // Identity requests (24 bytes) the runtime sends while its identity
      // engine starts, with no overlapped; it only needs them to succeed.
      result = X_E_SUCCESS;
      return true;
    case 0x2A007: {
      // Caches a profile's Kinect ID data: {xuid, version (8 bytes), data,
      // size}. Version kBiometricNone and size 1 for a profile without any.
      if (!buffer || buffer_length < 0x18) {
        result = X_E_INVALIDARG;
        return true;
      }
      const uint64_t xuid = xe::load_and_swap<uint64_t>(buffer);
      NUIController::BiometricData data;
      data.version = xe::load_and_swap<uint32_t>(buffer + 12);
      const uint32_t data_ptr = xe::load_and_swap<uint32_t>(buffer + 16);
      const uint32_t size = xe::load_and_swap<uint32_t>(buffer + 20);
      if (data_ptr && size && data.version != NUIController::kBiometricNone) {
        const uint8_t* source = Guest<uint8_t>(data_ptr);
        data.data.assign(source, source + size);
      }
      nui->SetCachedBiometricData(xuid, std::move(data));
      result = X_E_SUCCESS;
      return true;
    }
    case 0x2A008: {
      // Reads the cache: {xuid, &version (an 8-byte version), data, &size}.
      // Version 0 means xam doesn't have it yet, which has the title read the
      // profile's copy; without a data pointer, only the version.
      if (!buffer || buffer_length < 0x18) {
        result = X_E_INVALIDARG;
        return true;
      }
      const uint64_t xuid = xe::load_and_swap<uint64_t>(buffer);
      const uint32_t version_ptr = xe::load_and_swap<uint32_t>(buffer + 12);
      const uint32_t data_ptr = xe::load_and_swap<uint32_t>(buffer + 16);
      const uint32_t size_ptr = xe::load_and_swap<uint32_t>(buffer + 20);
      NUIController::BiometricData data = nui->GetCachedBiometricData(xuid);
      if (data.version == NUIController::kBiometricNone && data.data.empty() &&
          !nui->HasCachedBiometricData(xuid)) {
        data.version = 0;
      }
      if (size_ptr) {
        auto size_out = Guest<xe::be<uint32_t>>(size_ptr);
        if (data_ptr && data.data.size() > *size_out) {
          result = X_E_INVALIDARG;
          return true;
        }
        *size_out = uint32_t(data.data.size());
      }
      if (data_ptr && !data.data.empty()) {
        std::memcpy(Guest<uint8_t>(data_ptr), data.data.data(),
                    data.data.size());
      }
      if (version_ptr) {
        *Guest<xe::be<uint64_t>>(version_ptr) = data.version;
      }
      result = X_E_SUCCESS;
      return true;
    }
    case 0x2A009:
      // Biometric sign-in was switched off for a profile.
      result = X_E_SUCCESS;
      return true;
    case 0x2C001:
    case 0x2C003:
    case 0x2C004:
    case 0x2C005:
      // Identity service setup from 4D5308C9's runtime.
      result = X_E_SUCCESS;
      return true;
    case 0x2C009:
      // The title's identity engine registers its callback.
      if (!buffer) {
        result = X_E_INVALIDARG;
        return true;
      }
      nui->set_identity_engine(xe::load_and_swap<uint32_t>(buffer));
      XELOGI("Kinect: identity engine {:08X}", nui->identity_engine());
      result = X_E_SUCCESS;
      return true;
    case 0x2C00C: {
      // {operation, type (8 bytes), input, input size, output, &output size}.
      // Operation 2 reads an identity parameter by name, as text; with no
      // answer titles use their own defaults. Operation 1 converts Kinect ID
      // data to version 0x10001, which titles' data already is.
      if (!buffer) {
        result = X_E_INVALIDARG;
        return true;
      }
      const uint32_t operation = xe::load_and_swap<uint32_t>(buffer);
      const uint32_t input_ptr = xe::load_and_swap<uint32_t>(buffer + 16);
      const uint32_t input_size = xe::load_and_swap<uint32_t>(buffer + 20);
      const uint32_t output_ptr = xe::load_and_swap<uint32_t>(buffer + 24);
      const uint32_t output_size_ptr = xe::load_and_swap<uint32_t>(buffer + 28);
      if (operation == 1) {
        if (!input_ptr || !output_size_ptr) {
          result = X_E_INVALIDARG;
          return true;
        }
        auto output_size = Guest<xe::be<uint32_t>>(output_size_ptr);
        if (output_ptr) {
          if (input_size > *output_size) {
            result = X_E_INVALIDARG;
            return true;
          }
          std::memmove(Guest<uint8_t>(output_ptr), Guest<uint8_t>(input_ptr),
                       input_size);
        }
        *output_size = input_size;
      } else if (output_size_ptr) {
        *Guest<xe::be<uint32_t>>(output_size_ptr) = 0;
      }
      result = X_E_SUCCESS;
      return true;
    }
    case 0x2C00D: {
      // Resolves whose face it is among up to 8 enrollments (4D5308C9):
      // {scores, 0, 8 bytes, &result, size}. The answer is the enrollment, or
      // 0xFFFFFFFE for nobody, which it is without enrollments.
      if (!buffer) {
        result = X_E_INVALIDARG;
        return true;
      }
      const uint32_t result_ptr = xe::load_and_swap<uint32_t>(buffer + 16);
      if (result_ptr) {
        *Guest<xe::be<uint32_t>>(result_ptr) = 0xFFFFFFFE;
      }
      result = X_E_SUCCESS;
      return true;
    }
    default:
      return false;
  }
}

}  // namespace xam
}  // namespace kernel
}  // namespace xe

DECLARE_XAM_EMPTY_REGISTER_EXPORTS(NUIIdentity);

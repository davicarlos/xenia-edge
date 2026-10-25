/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2022 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include <cfloat>
#include <vector>

#include "xenia/base/memory.h"
#include "xenia/hid/kinect/usb_audio.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/kinect/kinect_audio.h"
#include "xenia/kernel/kinect/kinect_device.h"
#include "xenia/kernel/nui.h"
#include "xenia/kernel/util/shim_utils.h"
#include "xenia/kernel/xam/xam_private.h"
#include "xenia/xbox.h"

namespace xe {
namespace kernel {
namespace xam {

dword_result_t XamVoiceIsActiveProcess_entry() {
  // Returning 0 here will short-circuit a bunch of voice stuff.
  return 0;
}
DECLARE_XAM_EXPORT1(XamVoiceIsActiveProcess, kNone, kStub);

dword_result_t XamVoiceCreate_entry(dword_t user_index,
                                    dword_t max_attached_packets,  // 0xF
                                    lpdword_t out_voice_ptr) {
  // Null out the ptr.
  out_voice_ptr.Zero();
  return X_ERROR_ACCESS_DENIED;
}
DECLARE_XAM_EXPORT1(XamVoiceCreate, kNone, kStub);

dword_result_t XamVoiceClose_entry(lpunknown_t voice_ptr) { return 0; }
DECLARE_XAM_EXPORT1(XamVoiceClose, kNone, kStub);

dword_result_t XamVoiceHeadsetPresent_entry(lpunknown_t voice_ptr) { return 0; }
DECLARE_XAM_EXPORT1(XamVoiceHeadsetPresent, kNone, kStub);

dword_result_t XamVoiceSubmitPacket_entry(lpdword_t unk1, dword_t unk2,
                                          lpdword_t unk3) {
  // also may return 0xD000009D
  return 0x800700AA;
}
DECLARE_XAM_EXPORT1(XamVoiceSubmitPacket, kNone, kStub);

// 2 without the Kinect's microphone array (0x816FBBE8).
dword_result_t XamVoiceGetMicArrayStatus_entry() {
  return kernel_state()->nui()->audio()->microphones_available() ? 1 : 2;
}
DECLARE_XAM_EXPORT1(XamVoiceGetMicArrayStatus, kNone, kImplemented);

// Queues a request for the next 16 ms of the microphones, set by the event
// once filled; see KinectAudio for the request.
dword_result_t XamVoiceGetMicArrayAudioEx_entry(dword_t request,
                                                dword_t event) {
  return kernel_state()->nui()->audio()->ReadMicrophones(request, event);
}
DECLARE_XAM_EXPORT1(XamVoiceGetMicArrayAudioEx, kNone, kImplemented);

// Windows that would overrun are dropped on the host and missing ones come
// as silence, so the title never sees an underrun (McaDeviceRequest 0x20,
// 0x816FBD50).
dword_result_t XamVoiceGetMicArrayUnderrunStatus_entry(lpdword_t flags,
                                                       lpdword_t count) {
  if (flags) {
    *flags = 0;
  }
  if (count) {
    *count = 0;
  }
  return 0x10000000;
}
DECLARE_XAM_EXPORT1(XamVoiceGetMicArrayUnderrunStatus, kNone, kImplemented);

// Points the array's beam, in radians from straight ahead (the sensor forms
// beams within 50 degrees either way), or back to the sensor's choice with
// FLT_MAX: CEMD message 5 to its audio processing (0x816FBD60).
dword_result_t XamVoiceSetMicArrayBeamAngle_entry(double_t angle) {
  const float radians = float(angle);
  constexpr float kMaxBeamAngle = 0.873f;
  if (radians != FLT_MAX &&
      (radians < -kMaxBeamAngle || radians > kMaxBeamAngle)) {
    return X_E_INVALIDARG;
  }
  std::vector<uint8_t> message(16, 0);
  xe::store<uint32_t>(message.data(),
                      hid::kinect::UsbAudio::kAudioMessageMagic);
  xe::store<uint32_t>(message.data() + 8,
                      hid::kinect::UsbAudio::kAudioMessageBeamAngle);
  xe::store<float>(message.data() + 12, radians);
  return kernel_state()->nui()->device()->SendAudioMessage(message)
             ? X_E_SUCCESS
             : X_E_FAIL;
}
DECLARE_XAM_EXPORT1(XamVoiceSetMicArrayBeamAngle, kNone, kImplemented);

// Who the title considers idle at the array; xam keeps the mask for its own
// voice processing (0x816FB9C0), and the microphones read the same.
dword_result_t XamVoiceSetMicArrayIdleUsers_entry(dword_t idle_users,
                                                  dword_t signal) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamVoiceSetMicArrayIdleUsers, kNone, kImplemented);

}  // namespace xam
}  // namespace kernel
}  // namespace xe

DECLARE_XAM_EMPTY_REGISTER_EXPORTS(Voice);

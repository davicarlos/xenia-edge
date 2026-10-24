/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2022 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/nui.h"
#include "xenia/kernel/nui_audio.h"
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

dword_result_t XamVoiceGetMicArrayStatus_entry() {
  // 2 is a connected array, as titles check it; 0 none.
  return kernel_state()->nui()->audio()->microphones_available() ? 2 : 0;
}
DECLARE_XAM_EXPORT1(XamVoiceGetMicArrayStatus, kNone, kImplemented);

dword_result_t XamVoiceGetMicArrayAudioEx_entry(dword_t request_ptr,
                                                dword_t event_ptr) {
  // Queues a request for the next 16 ms of the Kinect microphones; see
  // NuiAudio for the request layout.
  return kernel_state()->nui()->audio()->ReadMicrophones(request_ptr,
                                                         event_ptr);
}
DECLARE_XAM_EXPORT1(XamVoiceGetMicArrayAudioEx, kNone, kSketchy);

// Which users the title considers idle at the microphone array. xam keeps the
// mask for its own voice processing and optionally signals its worker; the
// microphones are read the same either way here.
dword_result_t XamVoiceSetMicArrayIdleUsers_entry(dword_t idle_users,
                                                  dword_t signal) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamVoiceSetMicArrayIdleUsers, kNone, kImplemented);

dword_result_t XamVoiceGetMicArrayUnderrunStatus_entry(lpdword_t flags_ptr,
                                                       lpdword_t count_ptr) {
  // Windows that would have overrun are dropped on the host, and missing ones
  // are filled with silence, so the title never sees an underrun.
  if (flags_ptr) {
    *flags_ptr = 0;
  }
  if (count_ptr) {
    *count_ptr = 0;
  }
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamVoiceGetMicArrayUnderrunStatus, kNone, kSketchy);

}  // namespace xam
}  // namespace kernel
}  // namespace xe

DECLARE_XAM_EMPTY_REGISTER_EXPORTS(Voice);

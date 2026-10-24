/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_KERNEL_NUI_AUDIO_H_
#define XENIA_KERNEL_NUI_AUDIO_H_

#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>

#include "xenia/kernel/util/object_table.h"
#include "xenia/kernel/xthread.h"
#include "xenia/xbox.h"

namespace xe {
namespace hid {
namespace kinect {
class UsbAudio;
}  // namespace kinect
}  // namespace hid

namespace kernel {

// The Kinect microphone array as a title's NUI audio reads it. The title asks
// for 16 ms windows with XamVoiceGetMicArrayAudioEx and checks for them from
// its echo-cancellation (MEC) client, which the render driver calls every
// 256-sample frame (5.33 ms, one "tic"); a window spans 3 tics and carries the
// tic it was captured at. Here a host thread keeps that tic, completes a
// window request every 3 tics with the sensor's next window (or silence if it
// hasn't arrived) and then calls the MEC client.
class NuiAudio {
 public:
  NuiAudio();
  ~NuiAudio();

  // The audio device to read; nullptr without one. Not owned.
  void SetDevice(hid::kinect::UsbAudio* device);
  bool microphones_available();
  // Whether the title is reading the microphones, and the peak of the last
  // window it got, from 0 to 1, for the Kinect Tuner.
  bool microphones_in_use() const { return microphones_in_use_; }
  float microphone_peak() const { return microphone_peak_; }

  // XamVoiceGetMicArrayAudioEx: queues a window request, completed by
  // setting `event_ptr` (an X_KEVENT).
  X_STATUS ReadMicrophones(uint32_t request_ptr, uint32_t event_ptr);

  // XAudioRegisterRenderDriverMECClient and its counterparts.
  X_STATUS RegisterMecClient(uint32_t callback, uint32_t callback_arg,
                             uint32_t* handle);
  void UnregisterMecClient(uint32_t handle);
  uint16_t render_tic() const { return render_tic_; }

 private:
  struct Request {
    uint32_t request_ptr;
    uint32_t event_ptr;
  };

  void EnsureThread();
  void ThreadMain();
  void Tick();
  void CompleteRequest(const Request& request, uint16_t tic);

  std::mutex mutex_;
  hid::kinect::UsbAudio* device_ = nullptr;
  bool microphones_started_ = false;
  uint64_t next_window_ = 0;
  std::deque<Request> requests_;
  uint32_t mec_callback_ = 0;
  uint32_t mec_callback_arg_ = 0;

  std::atomic<uint16_t> render_tic_ = 0;
  std::atomic<bool> microphones_in_use_ = false;
  std::atomic<float> microphone_peak_ = 0.0f;
  object_ref<XHostThread> thread_;
  std::atomic<bool> running_ = false;
};

}  // namespace kernel
}  // namespace xe

#endif  // XENIA_KERNEL_NUI_AUDIO_H_

/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_KERNEL_KINECT_KINECT_AUDIO_H_
#define XENIA_KERNEL_KINECT_KINECT_AUDIO_H_

#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>

#include "xenia/kernel/util/object_table.h"
#include "xenia/kernel/xthread.h"
#include "xenia/xbox.h"

namespace xe {
namespace kernel {
namespace kinect {

class KinectDevice;

// The microphone array as a title's NUI runtime reads it. The title asks for
// 16 ms windows with XamVoiceGetMicArrayAudioEx and looks for them from its
// echo cancellation (MEC) client, which the audio render driver calls every
// 256-sample frame of 5.33 ms (a tic); a window spans 3 tics and carries the
// tic it was captured at. A host thread keeps the tic, completes a window
// request every 3 tics with the sensor's next window (silence if none has
// come) and calls the MEC client.
class KinectAudio {
 public:
  explicit KinectAudio(KinectDevice* device);
  ~KinectAudio();

  bool microphones_available() const;
  // For the Kinect Tuner: whether the title reads the microphones, and the
  // peak of its last window from 0 to 1.
  bool microphones_in_use() const { return microphones_in_use_; }
  float microphone_peak() const { return microphone_peak_; }

  // XamVoiceGetMicArrayAudioEx: queues a window request, completed by
  // setting the X_KEVENT at event_ptr.
  X_STATUS ReadMicrophones(uint32_t request_ptr, uint32_t event_ptr);

  // XAudioRegisterRenderDriverMECClient and XAudioGetRenderDriverTic.
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

  KinectDevice* device_;
  mutable std::mutex mutex_;
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

}  // namespace kinect
}  // namespace kernel
}  // namespace xe

#endif  // XENIA_KERNEL_KINECT_KINECT_AUDIO_H_

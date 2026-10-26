/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/kernel/kinect/kinect_audio.h"

#include <algorithm>
#include <chrono>
#include <climits>
#include <cstdlib>
#include <cstring>

#include "xenia/base/byte_order.h"
#include "xenia/base/clock.h"
#include "xenia/base/cvar.h"
#include "xenia/base/logging.h"
#include "xenia/base/threading.h"
#include "xenia/cpu/processor.h"
#include "xenia/hid/kinect/usb_audio.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/kinect/kinect_device.h"
#include "xenia/kernel/xboxkrnl/xboxkrnl_threading.h"
#include "xenia/memory.h"

DEFINE_int32(kinect_microphone_gain, 8,
             "Multiply the Kinect microphone samples titles get by this: they "
             "come off USB quieter than titles expect. At 1, Nike+ Kinect "
             "Training hears \"Xbox\" but not the command after it; at 8 both.",
             "HID");

namespace xe {
namespace kernel {
namespace kinect {

namespace {

// A window request (0x816FBCF8), big-endian: the buffers for the processed
// mix and the four microphones, their sizes, and from +0x40 in a larger one
// the tic the window was captured at and the time.
constexpr uint32_t kRequestBuffers = 0x04;
constexpr uint32_t kRequestSizes = 0x18;
constexpr uint32_t kRequestTic = 0x40;
constexpr uint32_t kRequestTimeHigh = 0x44;
constexpr uint32_t kRequestTimeLow = 0x48;
constexpr uint32_t kRequestChannels = 5;

constexpr uint32_t kTicsPerWindow = 3;
constexpr auto kTicPeriod = std::chrono::nanoseconds(5333333);
constexpr uint32_t kMecClientHandle = 0x4D454301;

}  // namespace

KinectAudio::KinectAudio(KinectDevice* device) : device_(device) {}

KinectAudio::~KinectAudio() {
  running_ = false;
  if (thread_) {
    thread_->Wait(0, 0, 0, nullptr);
    thread_.reset();
  }
  if (microphones_started_) {
    device_->StopMicrophones();
  }
}

bool KinectAudio::microphones_available() const {
  return device_->audio_present();
}

X_STATUS KinectAudio::ReadMicrophones(uint32_t request_ptr,
                                      uint32_t event_ptr) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!device_->audio_present()) {
      return X_STATUS_DEVICE_NOT_CONNECTED;
    }
    if (!microphones_started_) {
      if (!device_->StartMicrophones(nullptr)) {
        return X_STATUS_DEVICE_NOT_CONNECTED;
      }
      microphones_started_ = true;
      microphones_in_use_ = true;
      next_window_ = 0;
      XELOGI("Kinect: the title is reading the microphones");
    }
    requests_.push_back({request_ptr, event_ptr});
  }
  EnsureThread();
  return X_STATUS_SUCCESS;
}

X_STATUS KinectAudio::RegisterMecClient(uint32_t callback,
                                        uint32_t callback_arg,
                                        uint32_t* handle) {
  if (!device_->audio_present()) {
    return X_STATUS_DEVICE_NOT_CONNECTED;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    mec_callback_ = callback;
    mec_callback_arg_ = callback_arg;
  }
  XELOGI("Kinect: echo cancellation client {:08X}", callback);
  *handle = kMecClientHandle;
  EnsureThread();
  return X_STATUS_SUCCESS;
}

void KinectAudio::UnregisterMecClient(uint32_t handle) {
  std::lock_guard<std::mutex> lock(mutex_);
  mec_callback_ = 0;
  mec_callback_arg_ = 0;
}

void KinectAudio::EnsureThread() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (thread_) {
    return;
  }
  KernelState* kernel_state = KernelState::shared();
  running_ = true;
  thread_ = object_ref<XHostThread>(new XHostThread(
      kernel_state, 64 * 1024, 0,
      [this]() {
        ThreadMain();
        return 0;
      },
      kernel_state->GetSystemProcess()));
  thread_->set_can_debugger_suspend(true);
  thread_->set_name("Kinect Audio");
  thread_->Create();
}

// Keeps the tic on a fixed schedule of 5.33 ms, as the console's audio
// render driver does.
void KinectAudio::ThreadMain() {
  auto deadline = std::chrono::steady_clock::now();
  while (running_) {
    deadline += kTicPeriod;
    const auto now = std::chrono::steady_clock::now();
    if (deadline > now) {
      xe::threading::NanoSleepPrecise(
          std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - now)
              .count());
    } else if (now - deadline > kTicPeriod * 8) {
      // Far behind (stopped in a debugger): don't race to catch up.
      deadline = now;
    }
    Tick();
  }
}

// One tic: every third completes the oldest window request, then the echo
// cancellation client runs, on this thread as on the console's driver thread.
void KinectAudio::Tick() {
  const uint16_t tic = render_tic_ + 1;
  render_tic_ = tic;
  uint32_t callback = 0, callback_arg = 0;
  Request request = {};
  bool complete = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (tic % kTicsPerWindow == 0 && !requests_.empty()) {
      request = requests_.front();
      requests_.pop_front();
      complete = true;
    }
    callback = mec_callback_;
    callback_arg = mec_callback_arg_;
  }
  if (complete) {
    CompleteRequest(request, tic);
  }
  if (callback) {
    uint64_t args[] = {callback_arg};
    KernelState::shared()->processor()->Execute(thread_->thread_state(),
                                                callback, args, 1);
  }
}

void KinectAudio::CompleteRequest(const Request& request, uint16_t tic) {
  Memory* memory = KernelState::shared()->memory();
  uint8_t* fields = memory->TranslateVirtual<uint8_t*>(request.request_ptr);
  hid::kinect::UsbAudio::MicrophoneWindow window;
  bool have_window;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    have_window = device_->ReadMicrophoneWindow(next_window_, window);
  }
  // A request is due every third tic, whether the sensor sent a window or
  // not; without one it gets silence.
  if (!have_window) {
    std::memset(&window, 0, sizeof(window));
  }
  int32_t peak = 0;
  for (int16_t sample : window.processed) {
    peak = std::max(peak, std::abs(int32_t(sample)));
  }
  microphone_peak_ = float(peak) / 32768.0f;
  for (uint32_t channel = 0; channel < kRequestChannels; ++channel) {
    const uint32_t buffer =
        xe::load_and_swap<uint32_t>(fields + kRequestBuffers + channel * 4);
    const uint32_t size =
        xe::load_and_swap<uint32_t>(fields + kRequestSizes + channel * 4);
    if (!buffer) {
      continue;
    }
    uint8_t* dest = memory->TranslateVirtual<uint8_t*>(buffer);
    if (channel == 0) {
      // The processed mix, 16-bit.
      const uint32_t count =
          std::min<uint32_t>(size / sizeof(int16_t),
                             hid::kinect::UsbAudio::kSamplesPerWindow);
      for (uint32_t i = 0; i < count; ++i) {
        xe::store_and_swap<int16_t>(dest + i * sizeof(int16_t),
                                    window.processed[i]);
      }
    } else {
      // A microphone, 32-bit.
      const uint32_t count =
          std::min<uint32_t>(size / sizeof(int32_t),
                             hid::kinect::UsbAudio::kSamplesPerWindow);
      for (uint32_t i = 0; i < count; ++i) {
        const int64_t sample = int64_t(window.microphones[channel - 1][i]) *
                               cvars::kinect_microphone_gain;
        xe::store_and_swap<int32_t>(
            dest + i * sizeof(int32_t),
            int32_t(std::clamp<int64_t>(sample, INT32_MIN, INT32_MAX)));
      }
    }
  }
  xe::store_and_swap<uint16_t>(fields + kRequestTic, tic);
  const uint64_t time = Clock::QueryGuestSystemTime();
  xe::store_and_swap<uint32_t>(fields + kRequestTimeHigh,
                               uint32_t(time >> 32));
  xe::store_and_swap<uint32_t>(fields + kRequestTimeLow, uint32_t(time));
  xboxkrnl::xeKeSetEvent(
      memory->TranslateVirtual<X_KEVENT*>(request.event_ptr), 1, 0);
}

}  // namespace kinect
}  // namespace kernel
}  // namespace xe

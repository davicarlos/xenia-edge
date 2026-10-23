/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/kernel/nui_audio.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>

#include "xenia/base/byte_order.h"
#include "xenia/base/clock.h"
#include "xenia/base/logging.h"
#include "xenia/base/threading.h"
#include "xenia/cpu/processor.h"
#include "xenia/hid/kinect/usb_audio.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/xboxkrnl/xboxkrnl_threading.h"
#include "xenia/memory.h"

namespace xe {
namespace kernel {

namespace {

// A window request as the NUI runtime of Nike+ Kinect Training lays it out
// (big-endian): the buffers for the processed mix and the four microphones,
// their sizes, and the tic the window was captured at.
constexpr uint32_t kRequestBuffers = 0x04;
constexpr uint32_t kRequestSizes = 0x18;
constexpr uint32_t kRequestTic = 0x40;
constexpr uint32_t kRequestTimeHigh = 0x44;
constexpr uint32_t kRequestTimeLow = 0x48;
constexpr uint32_t kRequestChannels = 5;

constexpr uint32_t kTicsPerWindow = 3;
constexpr auto kTicPeriod = std::chrono::nanoseconds(5333333);
constexpr uint32_t kThreadStackSize = 64 * 1024;
constexpr uint32_t kMecClientHandle = 0x4D454301;

}  // namespace

NuiAudio::NuiAudio() = default;

NuiAudio::~NuiAudio() {
  running_ = false;
  if (thread_) {
    thread_->Wait(0, 0, 0, nullptr);
    thread_.reset();
  }
}

void NuiAudio::SetDevice(hid::kinect::UsbAudio* device) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (device_ && microphones_started_) {
    device_->StopMicrophones();
  }
  device_ = device;
  microphones_started_ = false;
  microphones_in_use_ = false;
}

bool NuiAudio::microphones_available() {
  std::lock_guard<std::mutex> lock(mutex_);
  return device_ != nullptr;
}

X_STATUS NuiAudio::ReadMicrophones(uint32_t request_ptr, uint32_t event_ptr) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!device_) {
      return X_STATUS_DEVICE_NOT_CONNECTED;
    }
    if (!microphones_started_) {
      if (!device_->StartMicrophones(nullptr)) {
        return X_STATUS_DEVICE_NOT_CONNECTED;
      }
      microphones_started_ = true;
      microphones_in_use_ = true;
      next_window_ = 0;
      XELOGI("NUI: the title is reading the Kinect microphones");
    }
    requests_.push_back({request_ptr, event_ptr});
  }
  EnsureThread();
  return X_STATUS_SUCCESS;
}

X_STATUS NuiAudio::RegisterMecClient(uint32_t callback, uint32_t callback_arg,
                                     uint32_t* handle) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!device_) {
      return X_STATUS_DEVICE_NOT_CONNECTED;
    }
    mec_callback_ = callback;
    mec_callback_arg_ = callback_arg;
  }
  XELOGI("NUI: echo cancellation client {:08X} registered", callback);
  *handle = kMecClientHandle;
  EnsureThread();
  return X_STATUS_SUCCESS;
}

void NuiAudio::UnregisterMecClient(uint32_t handle) {
  std::lock_guard<std::mutex> lock(mutex_);
  mec_callback_ = 0;
  mec_callback_arg_ = 0;
}

void NuiAudio::EnsureThread() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (thread_) {
    return;
  }
  KernelState* kernel_state = KernelState::shared();
  running_ = true;
  thread_ = object_ref<XHostThread>(new XHostThread(
      kernel_state, kThreadStackSize, 0,
      [this]() {
        ThreadMain();
        return 0;
      },
      kernel_state->GetSystemProcess()));
  thread_->set_can_debugger_suspend(true);
  thread_->set_name("NUI Audio");
  thread_->Create();
}

void NuiAudio::ThreadMain() {
  auto deadline = std::chrono::steady_clock::now();
  while (running_) {
    deadline += kTicPeriod;
    const auto now = std::chrono::steady_clock::now();
    if (deadline > now) {
      xe::threading::NanoSleepPrecise(
          std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - now)
              .count());
    } else if (now - deadline > kTicPeriod * 8) {
      // Too far behind (the debugger stopped us); don't race to catch up.
      deadline = now;
    }
    Tick();
  }
}

void NuiAudio::Tick() {
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

void NuiAudio::CompleteRequest(const Request& request, uint16_t tic) {
  Memory* memory = KernelState::shared()->memory();
  uint8_t* fields = memory->TranslateVirtual<uint8_t*>(request.request_ptr);
  hid::kinect::UsbAudio::MicrophoneWindow window;
  bool have_window = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    have_window =
        device_ && device_->ReadMicrophoneWindow(next_window_, window);
  }
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
      const uint32_t count = std::min<uint32_t>(
          size / sizeof(int16_t),
          hid::kinect::UsbAudio::kSamplesPerWindow);
      for (uint32_t i = 0; i < count; ++i) {
        xe::store_and_swap<int16_t>(dest + i * sizeof(int16_t),
                                    window.processed[i]);
      }
    } else {
      const uint32_t count = std::min<uint32_t>(
          size / sizeof(int32_t),
          hid::kinect::UsbAudio::kSamplesPerWindow);
      for (uint32_t i = 0; i < count; ++i) {
        xe::store_and_swap<int32_t>(dest + i * sizeof(int32_t),
                                    window.microphones[channel - 1][i]);
      }
    }
  }
  xe::store_and_swap<uint16_t>(fields + kRequestTic, tic);
  const uint64_t time = Clock::QueryGuestSystemTime();
  xe::store_and_swap<uint32_t>(fields + kRequestTimeHigh, uint32_t(time >> 32));
  xe::store_and_swap<uint32_t>(fields + kRequestTimeLow, uint32_t(time));
  xboxkrnl::xeKeSetEvent(memory->TranslateVirtual<X_KEVENT*>(request.event_ptr),
                         1, 0);
}

}  // namespace kernel
}  // namespace xe

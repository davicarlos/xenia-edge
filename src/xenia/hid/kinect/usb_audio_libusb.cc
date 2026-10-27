/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

// Kinect audio firmware upload over libusb. The bootloader protocol follows
// libfreenect (src/loader.c, Apache-2.0).

#include "xenia/hid/kinect/usb_audio.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <vector>

#include "xenia/base/logging.h"
#include "xenia/base/threading.h"

#include "third_party/libusb/libusb/libusb.h"

namespace xe {
namespace hid {
namespace kinect {

namespace {

constexpr uint16_t kVendorId = 0x045E;
constexpr uint16_t kAudioProductId = 0x02AD;
constexpr unsigned char kCommandEndpoint = 0x01;
constexpr unsigned char kReplyEndpoint = 0x81;
constexpr unsigned int kBulkTimeoutMs = 1000;
// The device drops off the bus and comes back running the firmware.
constexpr auto kReenumerateTimeout = std::chrono::seconds(10);
constexpr auto kReenumeratePollInterval = std::chrono::milliseconds(250);

constexpr uint32_t kCommandMagic = 0x06022009;
constexpr uint32_t kReplyMagic = 0x0A6FE000;
constexpr uint32_t kFirmwareMagic = 0xCA77F00D;
constexpr uint32_t kCommandWrite = 0x03;
constexpr uint32_t kCommandExecute = 0x04;
// Firmware is written in pages, each sent in bulk packets.
constexpr uint32_t kPageBytes = 0x4000;
constexpr uint32_t kPacketBytes = 512;

// All fields are little-endian on the wire.
#pragma pack(push, 1)
struct BootloaderCommand {
  uint32_t magic;
  uint32_t tag;
  uint32_t bytes;
  uint32_t command;
  uint32_t address;
  uint32_t unknown;
};
struct BootloaderReply {
  uint32_t magic;
  uint32_t tag;
  uint32_t status;
};
struct FirmwareHeader {
  uint32_t magic;
  uint16_t version_minor;
  uint16_t version_major;
  uint16_t version_release;
  uint16_t version_patch;
  uint32_t base_address;
  uint32_t size;
  uint32_t entry_address;
};
#pragma pack(pop)

// Opens the audio device, returning its interface count in `interfaces`.
libusb_device_handle* OpenAudioDevice(libusb_context* context,
                                      int& interfaces) {
  libusb_device** devices;
  const ssize_t count = libusb_get_device_list(context, &devices);
  if (count < 0) {
    return nullptr;
  }
  libusb_device_handle* handle = nullptr;
  for (ssize_t i = 0; i < count && !handle; ++i) {
    libusb_device_descriptor descriptor;
    if (libusb_get_device_descriptor(devices[i], &descriptor) < 0 ||
        descriptor.idVendor != kVendorId ||
        descriptor.idProduct != kAudioProductId) {
      continue;
    }
    libusb_config_descriptor* config;
    if (libusb_get_active_config_descriptor(devices[i], &config) < 0) {
      continue;
    }
    interfaces = config->bNumInterfaces;
    libusb_free_config_descriptor(config);
    const int result = libusb_open(devices[i], &handle);
    if (result < 0) {
      XELOGW("Kinect: cannot open the audio device ({})",
             libusb_error_name(result));
      handle = nullptr;
    }
  }
  libusb_free_device_list(devices, 1);
  return handle;
}

// The audio device's ROM bootloader, which is all it runs after power-up: it
// takes commands on endpoint 0x01 (a header, then the data in 512-byte
// packets) and answers each on 0x81.
class Bootloader {
 public:
  explicit Bootloader(libusb_device_handle* handle) : handle_(handle) {}

  bool Send(uint32_t command, uint32_t address, const uint8_t* data,
            uint32_t bytes) {
    BootloaderCommand header = {};
    header.magic = kCommandMagic;
    header.tag = tag_;
    header.bytes = bytes;
    header.command = command;
    header.address = address;
    if (!Bulk(kCommandEndpoint, reinterpret_cast<uint8_t*>(&header),
              sizeof(header))) {
      return false;
    }
    for (uint32_t sent = 0; sent < bytes; sent += kPacketBytes) {
      if (!Bulk(kCommandEndpoint, const_cast<uint8_t*>(data + sent),
                std::min(kPacketBytes, bytes - sent))) {
        return false;
      }
    }
    return ReadReply();
  }

 private:
  bool Bulk(unsigned char endpoint, uint8_t* data, uint32_t bytes) {
    int transferred = 0;
    const int result =
        libusb_bulk_transfer(handle_, endpoint, data, static_cast<int>(bytes),
                             &transferred, kBulkTimeoutMs);
    if (result < 0 || uint32_t(transferred) != bytes) {
      XELOGE("Kinect: audio bootloader transfer failed ({}, {} of {} bytes)",
             libusb_error_name(result), transferred, bytes);
      return false;
    }
    return true;
  }

  bool ReadReply() {
    uint8_t buffer[kPacketBytes];
    int transferred = 0;
    const int result =
        libusb_bulk_transfer(handle_, kReplyEndpoint, buffer, sizeof(buffer),
                             &transferred, kBulkTimeoutMs);
    BootloaderReply reply;
    if (result < 0 || transferred != sizeof(reply)) {
      XELOGE("Kinect: no reply from the audio bootloader ({}, {} bytes)",
             libusb_error_name(result), transferred);
      return false;
    }
    std::memcpy(&reply, buffer, sizeof(reply));
    if (reply.magic != kReplyMagic || reply.tag != tag_) {
      XELOGE("Kinect: bad reply from the audio bootloader ({:08X}, tag {})",
             reply.magic, reply.tag);
      return false;
    }
    if (reply.status) {
      XELOGW("Kinect: audio bootloader status {:08X}", reply.status);
    }
    ++tag_;
    return true;
  }

  libusb_device_handle* handle_;
  uint32_t tag_ = 1;
};

// Writes the firmware into the device's memory and starts it (bootloader
// commands 3 and 4). The firmware is lost when the sensor loses power.
bool Upload(libusb_device_handle* handle, const std::vector<uint8_t>& image) {
  FirmwareHeader header;
  if (image.size() < sizeof(header)) {
    XELOGE("Kinect: the audio firmware file is too small");
    return false;
  }
  std::memcpy(&header, image.data(), sizeof(header));
  if (header.magic != kFirmwareMagic || header.size > image.size()) {
    XELOGE("Kinect: not an audio firmware image (magic {:08X}, size {:X})",
           header.magic, header.size);
    return false;
  }
  XELOGI("Kinect: uploading audio firmware {}.{}.{}.{} ({} bytes)",
         header.version_major, header.version_minor, header.version_release,
         header.version_patch, header.size);

  if (libusb_claim_interface(handle, 0) < 0) {
    XELOGE("Kinect: cannot claim the audio bootloader interface");
    return false;
  }
  Bootloader bootloader(handle);
  // The image is written from its start, header included, at base_address.
  bool ok = true;
  for (uint32_t offset = 0; ok && offset < header.size; offset += kPageBytes) {
    ok = bootloader.Send(kCommandWrite, header.base_address + offset,
                         image.data() + offset,
                         std::min(kPageBytes, header.size - offset));
  }
  ok = ok && bootloader.Send(kCommandExecute, header.entry_address, nullptr, 0);
  libusb_release_interface(handle, 0);
  return ok;
}

// Firmware commands are {magic, tag, arg1, command, arg2}; the status request
// leaves out arg2.
#pragma pack(push, 1)
struct FirmwareCommand {
  uint32_t magic;
  uint32_t tag;
  uint32_t arg1;
  uint32_t command;
  uint32_t arg2;
};
#pragma pack(pop)

constexpr uint32_t kCommandLed = 0x10;
constexpr uint32_t kCommandReadStatus = 0x8032;
constexpr uint32_t kCommandSetTilt = 0x803B;
// Messages to the audio processing (see UsbAudio::SendAudioMessage).
constexpr uint32_t kCommandMessageStart = 0x133;
constexpr uint32_t kCommandMessageBlock = 0x134;
constexpr uint32_t kCommandMessageEnd = 0x135;
constexpr uint32_t kMessageBlockBytes = 0x4000;
// Status block of the 0x8032 command, in 32-bit little-endian words.
constexpr uint32_t kStatusBytes = 104;
constexpr size_t kStatusFlagsWord = 1;
constexpr size_t kStatusAccelerometerWord = 4;
// Words 9 to 11 and 12 to 14 are the accelerometer through two stages of
// filtering.
constexpr size_t kStatusFilteredWord = 9;
constexpr size_t kStatusGravityWord = 12;
constexpr size_t kStatusTiltAngleWord = 7;
constexpr unsigned int kMotorTimeoutMs = 250;
// Microphone stream, from libfreenect's src/audio.c.
constexpr unsigned char kMicrophoneEndpoint = 0x82;
constexpr unsigned char kSpeakerEndpoint = 0x02;
constexpr uint32_t kMicrophonePacketBytes = 524;
constexpr uint32_t kMicrophoneHeaderBytes = 12;
constexpr uint32_t kMicrophonePacketMagic = 0x80000080;
constexpr uint32_t kSpeakerPacketBytes = 76;
constexpr int kMicrophoneTransfers = 8;
constexpr int kMicrophonePacketsPerTransfer = 16;
// Channels 1-9 of a window.
constexpr uint32_t kAllMicrophoneChannels = 0x1FE;
constexpr size_t kQueuedWindows = 32;
constexpr long kEventTimeoutUs = 50000;
constexpr auto kMicrophoneStopTimeout = std::chrono::seconds(2);

class UsbAudioLibusb;
void RegisterOpenAudio(UsbAudioLibusb* motor);
void UnregisterOpenAudio(UsbAudioLibusb* motor);

class UsbAudioLibusb : public UsbAudio {
 public:
  ~UsbAudioLibusb() override {
    UnregisterOpenAudio(this);
    StopMicrophones();
    RestoreIdleLed();
    if (handle_) {
      libusb_release_interface(handle_, 0);
      libusb_close(handle_);
    }
    if (context_) {
      libusb_exit(context_);
    }
  }

  bool Initialize() {
    if (libusb_init(&context_) < 0) {
      context_ = nullptr;
      return false;
    }
    int interfaces = 0;
    handle_ = OpenAudioDevice(context_, interfaces);
    if (!handle_) {
      return false;
    }
    // The bootloader has one interface; the firmware adds the audio ones.
    if (interfaces < 2) {
      XELOGW("Kinect: the audio firmware isn't running; no motor or LED");
      return false;
    }
    libusb_set_auto_detach_kernel_driver(handle_, 1);
    if (libusb_claim_interface(handle_, 0) < 0) {
      XELOGE("Kinect: cannot claim the motor interface");
      return false;
    }
    claimed_ = true;
    RegisterOpenAudio(this);
    return true;
  }

  bool SetTiltAngle(int32_t degrees) override {
    std::lock_guard<std::mutex> lock(mutex_);
    return Send(kCommandSetTilt, 0, uint32_t(degrees),
                sizeof(FirmwareCommand)) &&
           ReadReply();
  }

  // The firmware's 0x803A command fails while the motor moves; aiming it at
  // the angle it's at stops it there instead.
  bool StopMovement() override {
    State state;
    return ReadState(state) && SetTiltAngle(state.tilt_angle);
  }

  bool SetLed(Led led) override {
    std::lock_guard<std::mutex> lock(mutex_);
    return Send(kCommandLed, 0, uint32_t(led), sizeof(FirmwareCommand)) &&
           ReadReply();
  }

  // Back to the blinking green the sensor shows when nothing uses it. Skipped
  // if another thread is in the middle of a command, as at a quick exit.
  void RestoreIdleLed() {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (claimed_ && lock.owns_lock()) {
      Send(kCommandLed, 0, uint32_t(Led::kBlinkGreen),
           sizeof(FirmwareCommand)) &&
          ReadReply();
    }
  }

  bool ReadState(State& state) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!Send(kCommandReadStatus, kStatusBytes, 0,
              offsetof(FirmwareCommand, arg2))) {
      return false;
    }
    uint32_t words[kStatusBytes / 4] = {};
    int transferred = 0;
    const int result = libusb_bulk_transfer(
        handle_, kReplyEndpoint, reinterpret_cast<uint8_t*>(words),
        sizeof(words), &transferred, kMotorTimeoutMs);
    if (result < 0 || transferred != int(sizeof(words))) {
      return false;
    }
    if (!ReadReply()) {
      return false;
    }
    for (size_t i = 0; i < 3; ++i) {
      state.accelerometer[i] =
          int16_t(words[kStatusAccelerometerWord + i] & 0xFFFF);
      state.filtered[i] = int16_t(words[kStatusFilteredWord + i] & 0xFFFF);
      state.gravity[i] = int16_t(words[kStatusGravityWord + i] & 0xFFFF);
    }
    state.tilt_angle = int8_t(words[kStatusTiltAngleWord] & 0xFF);
    state.flags = words[kStatusFlagsWord];
    return true;
  }

  // The firmware takes messages like it takes the calibration libfreenect
  // uploads: 0x133 with the size, 0x134 for each block at its offset, then
  // 0x135, after which its audio loop handles the message.
  bool SendAudioMessage(const std::vector<uint8_t>& message) override {
    std::lock_guard<std::mutex> lock(mutex_);
    const uint32_t size = uint32_t(message.size());
    if (!Send(kCommandMessageStart, 0, size, sizeof(FirmwareCommand)) ||
        !ReadReply()) {
      return false;
    }
    for (uint32_t offset = 0; offset < size; offset += kMessageBlockBytes) {
      const uint32_t block = std::min(kMessageBlockBytes, size - offset);
      if (!Send(kCommandMessageBlock, block, offset,
                sizeof(FirmwareCommand))) {
        return false;
      }
      for (uint32_t sent = 0; sent < block; sent += kPacketBytes) {
        const uint32_t bytes = std::min(kPacketBytes, block - sent);
        int transferred = 0;
        if (libusb_bulk_transfer(
                handle_, kCommandEndpoint,
                const_cast<uint8_t*>(message.data() + offset + sent),
                int(bytes), &transferred, kMotorTimeoutMs) < 0 ||
            uint32_t(transferred) != bytes) {
          return false;
        }
      }
      if (!ReadReply()) {
        return false;
      }
    }
    return Send(kCommandMessageEnd, 0, size, sizeof(FirmwareCommand)) &&
           ReadReply();
  }

 private:
  bool Send(uint32_t command, uint32_t arg1, uint32_t arg2, size_t bytes) {
    FirmwareCommand request = {kCommandMagic, tag_++, arg1, command, arg2};
    int transferred = 0;
    const int result = libusb_bulk_transfer(
        handle_, kCommandEndpoint, reinterpret_cast<uint8_t*>(&request),
        int(bytes), &transferred, kMotorTimeoutMs);
    return result >= 0 && size_t(transferred) == bytes;
  }

  bool ReadReply() {
    BootloaderReply reply;
    uint8_t buffer[kPacketBytes];
    int transferred = 0;
    const int result =
        libusb_bulk_transfer(handle_, kReplyEndpoint, buffer, sizeof(buffer),
                             &transferred, kMotorTimeoutMs);
    if (result < 0 || transferred != sizeof(reply)) {
      return false;
    }
    std::memcpy(&reply, buffer, sizeof(reply));
    return reply.magic == kReplyMagic && reply.status == 0;
  }

  // Microphone stream: one transfer list per direction.
  struct IsoStream {
    std::vector<libusb_transfer*> transfers;
    std::vector<std::unique_ptr<uint8_t[]>> buffers;
  };

  bool StartIso(unsigned char endpoint, uint32_t packet_bytes,
                IsoStream& stream) {
    for (int i = 0; i < kMicrophoneTransfers; ++i) {
      libusb_transfer* transfer =
          libusb_alloc_transfer(kMicrophonePacketsPerTransfer);
      auto buffer = std::make_unique<uint8_t[]>(packet_bytes *
                                                kMicrophonePacketsPerTransfer);
      libusb_fill_iso_transfer(transfer, handle_, endpoint, buffer.get(),
                               packet_bytes * kMicrophonePacketsPerTransfer,
                               kMicrophonePacketsPerTransfer, OnIsoTransfer,
                               this, 0);
      libusb_set_iso_packet_lengths(transfer, packet_bytes);
      if (endpoint == kSpeakerEndpoint) {
        FillSpeakerPackets(transfer);
      }
      stream.transfers.push_back(transfer);
      stream.buffers.push_back(std::move(buffer));
      ++in_flight_;
      if (libusb_submit_transfer(transfer) < 0) {
        --in_flight_;
        return false;
      }
    }
    return true;
  }

  static void LIBUSB_CALL OnIsoTransfer(libusb_transfer* transfer) {
    auto self = static_cast<UsbAudioLibusb*>(transfer->user_data);
    const bool is_input = transfer->endpoint == kMicrophoneEndpoint;
    if (is_input && transfer->status == LIBUSB_TRANSFER_COMPLETED) {
      const uint8_t* buffer = transfer->buffer;
      for (int i = 0; i < transfer->num_iso_packets; ++i) {
        const libusb_iso_packet_descriptor& packet =
            transfer->iso_packet_desc[i];
        if (packet.status == LIBUSB_TRANSFER_COMPLETED) {
          self->AddMicrophonePacket(buffer, packet.actual_length);
        }
        buffer += packet.length;
      }
    }
    if (self->microphones_stopping_ ||
        transfer->status == LIBUSB_TRANSFER_NO_DEVICE) {
      --self->in_flight_;
      return;
    }
    if (!is_input) {
      self->FillSpeakerPackets(transfer);
    }
    if (libusb_submit_transfer(transfer) < 0) {
      --self->in_flight_;
    }
  }

  // The sensor expects a speaker stream alongside the microphones, for its
  // echo cancellation; this sends silence with the window and sequence
  // headers libfreenect found it needs.
  void FillSpeakerPackets(libusb_transfer* transfer) {
    uint8_t* packet = transfer->buffer;
    for (int i = 0; i < transfer->num_iso_packets; ++i) {
      std::memset(packet, 0, kSpeakerPacketBytes);
      packet[0] = uint8_t(out_window_);
      packet[1] = uint8_t(out_window_ >> 8);
      packet[2] = out_sequence_;
      uint8_t marker = 0x01;
      if (out_counter_in_window_ < 4) {
        marker = uint8_t(
            ((out_timestamp_ >> (out_counter_in_window_ * 4)) & 0xF) << 4 |
            0x05);
      } else if (out_window_parity_ == 0 && out_counter_in_window_ < 8) {
        marker = uint8_t(
            (((out_timestamp_ + 23) >> ((out_counter_in_window_ - 4) * 4)) &
             0xF)
                << 4 |
            0x05);
      }
      packet[3] = marker;
      ++out_sequence_;
      ++out_counter_in_window_;
      out_timestamp_ += out_window_parity_ == 1 ? 6 : 5;
      if (out_sequence_ == 0x80 || out_sequence_ == 0x2B ||
          out_sequence_ == 0x56) {
        if (out_sequence_ == 0x80) {
          out_sequence_ = 0;
        }
        out_counter_in_window_ = 0;
        ++out_window_;
        if (++out_window_parity_ == 3) {
          out_window_parity_ = 0;
        }
      }
      packet += transfer->iso_packet_desc[i].length;
    }
  }

  // A 524-byte packet carries one channel's 512 bytes of a window: channel 1
  // the processed mix (256 16-bit samples), channels 2-9 the halves of the
  // four microphones (128 32-bit samples each). Little-endian.
  void AddMicrophonePacket(const uint8_t* data, uint32_t length) {
    if (length != kMicrophonePacketBytes) {
      return;
    }
    uint32_t magic;
    uint16_t channel, window;
    std::memcpy(&magic, data, 4);
    std::memcpy(&channel, data + 4, 2);
    std::memcpy(&window, data + 8, 2);
    if (magic != kMicrophonePacketMagic || channel < 1 || channel > 9) {
      return;
    }
    std::lock_guard<std::mutex> lock(windows_mutex_);
    if (window != assembling_.window) {
      FinishWindow();
      assembling_.window = window;
      assembling_channels_ = 0;
    }
    const uint8_t* samples = data + kMicrophoneHeaderBytes;
    if (channel == 1) {
      std::memcpy(assembling_.processed, samples,
                  sizeof(assembling_.processed));
    } else {
      const uint32_t microphone = (channel - 2) / 2;
      const uint32_t half = (channel - 2) % 2;
      std::memcpy(assembling_.microphones[microphone] +
                      half * (kSamplesPerWindow / 2),
                  samples, kSamplesPerWindow / 2 * sizeof(int32_t));
    }
    assembling_channels_ |= 1u << (channel - 1);
  }

  // Queues the window being assembled if every channel arrived.
  void FinishWindow() {
    if ((assembling_channels_ & kAllMicrophoneChannels) !=
        kAllMicrophoneChannels) {
      return;
    }
    // The sensor sends its processed mix only now and then without a
    // speaker signal to cancel; the microphones' average stands in, scaled
    // from the 32-bit microphone samples to 16 bits.
    if (!(assembling_channels_ & 1)) {
      for (uint32_t i = 0; i < kSamplesPerWindow; ++i) {
        int64_t sum = 0;
        for (uint32_t m = 0; m < kMicrophoneCount; ++m) {
          sum += assembling_.microphones[m][i];
        }
        assembling_.processed[i] =
            int16_t(std::clamp<int64_t>((sum / kMicrophoneCount) >> 16,
                                        INT16_MIN, INT16_MAX));
      }
    }
    // The sensor may first send windows left from an earlier session and
    // then restart its numbering; older windows are dropped then.
    if (windows_written_ > queue_start_ &&
        int16_t(assembling_.window -
                windows_[(windows_written_ - 1) % kQueuedWindows].window) <=
            0) {
      queue_start_ = windows_written_;
    }
    windows_[windows_written_ % kQueuedWindows] = assembling_;
    ++windows_written_;
    if (on_window_) {
      on_window_();
    }
  }

  // libusb completes the isochronous transfers from here.
  void EventLoop() {
    timeval timeout = {0, kEventTimeoutUs};
    while (events_running_) {
      libusb_handle_events_timeout(context_, &timeout);
    }
  }

 public:
  // With the speaker stream the sensor expects beside them (see
  // FillSpeakerPackets).
  bool StartMicrophones(std::function<void()> on_window) override {
    if (microphones_started_) {
      return true;
    }
    on_window_ = std::move(on_window);
    microphones_stopping_ = false;
    events_running_ = true;
    event_thread_ =
        xe::threading::Thread::Create({}, [this]() { EventLoop(); });
    if (!event_thread_) {
      events_running_ = false;
      return false;
    }
    event_thread_->set_name("Kinect Audio USB Events");
    microphones_started_ = true;
    if (!StartIso(kMicrophoneEndpoint, kMicrophonePacketBytes, input_) ||
        !StartIso(kSpeakerEndpoint, kSpeakerPacketBytes, output_)) {
      XELOGE("Kinect: cannot start the microphone stream");
      StopMicrophones();
      return false;
    }
    XELOGI("Kinect: microphones streaming");
    return true;
  }

  void StopMicrophones() override {
    if (!microphones_started_) {
      return;
    }
    microphones_stopping_ = true;
    for (IsoStream* stream : {&input_, &output_}) {
      for (libusb_transfer* transfer : stream->transfers) {
        libusb_cancel_transfer(transfer);
      }
    }
    const auto deadline =
        std::chrono::steady_clock::now() + kMicrophoneStopTimeout;
    while (in_flight_ > 0 && std::chrono::steady_clock::now() < deadline) {
      xe::threading::Sleep(std::chrono::milliseconds(10));
    }
    events_running_ = false;
    if (event_thread_) {
      xe::threading::Wait(event_thread_.get(), false);
      event_thread_.reset();
    }
    for (IsoStream* stream : {&input_, &output_}) {
      if (in_flight_ > 0) {
        // Freeing a transfer that libusb still owns is unsafe, so leak them.
        for (auto& buffer : stream->buffers) {
          static_cast<void>(buffer.release());
        }
      } else {
        for (libusb_transfer* transfer : stream->transfers) {
          libusb_free_transfer(transfer);
        }
      }
      stream->transfers.clear();
      stream->buffers.clear();
    }
    microphones_started_ = false;
  }

  // The last kQueuedWindows windows are kept; a reader that fell further
  // behind skips to the oldest.
  bool ReadMicrophoneWindow(uint64_t& next,
                            MicrophoneWindow& window) override {
    std::lock_guard<std::mutex> lock(windows_mutex_);
    const uint64_t oldest =
        std::max(queue_start_, windows_written_ > kQueuedWindows
                                   ? windows_written_ - kQueuedWindows
                                   : uint64_t(0));
    next = std::max(next, oldest);
    if (next >= windows_written_) {
      return false;
    }
    window = windows_[next % kQueuedWindows];
    ++next;
    return true;
  }

 private:
  libusb_context* context_ = nullptr;
  libusb_device_handle* handle_ = nullptr;
  bool claimed_ = false;
  std::mutex mutex_;
  uint32_t tag_ = 1;

  IsoStream input_;
  IsoStream output_;
  std::function<void()> on_window_;
  std::unique_ptr<xe::threading::Thread> event_thread_;
  std::atomic<bool> events_running_ = false;
  std::atomic<bool> microphones_stopping_ = false;
  std::atomic<int> in_flight_ = 0;
  bool microphones_started_ = false;
  std::mutex windows_mutex_;
  MicrophoneWindow assembling_ = {};
  uint32_t assembling_channels_ = 0;
  MicrophoneWindow windows_[kQueuedWindows] = {};
  uint64_t windows_written_ = 0;
  uint64_t queue_start_ = 0;
  // Speaker stream headers, touched only on the USB thread.
  uint16_t out_window_ = 0;
  uint8_t out_sequence_ = 0;
  uint32_t out_counter_in_window_ = 0;
  uint32_t out_timestamp_ = 0;
  uint32_t out_window_parity_ = 0;
};

// The emulator exits via std::quick_exit, which skips destructors; without
// this the LED stays as the title left it.
std::mutex g_open_audio_mutex;
std::vector<UsbAudioLibusb*> g_open_audio;

void RestoreOpenAudioLeds() {
  std::lock_guard<std::mutex> lock(g_open_audio_mutex);
  for (UsbAudioLibusb* motor : g_open_audio) {
    motor->RestoreIdleLed();
  }
}

void RegisterOpenAudio(UsbAudioLibusb* motor) {
  static std::once_flag handler_registered;
  std::call_once(handler_registered,
                 []() { std::at_quick_exit(RestoreOpenAudioLeds); });
  std::lock_guard<std::mutex> lock(g_open_audio_mutex);
  g_open_audio.push_back(motor);
}

void UnregisterOpenAudio(UsbAudioLibusb* motor) {
  std::lock_guard<std::mutex> lock(g_open_audio_mutex);
  g_open_audio.erase(
      std::remove(g_open_audio.begin(), g_open_audio.end(), motor),
      g_open_audio.end());
}

}  // namespace

std::unique_ptr<UsbAudio> UsbAudio::Open() {
  auto motor = std::make_unique<UsbAudioLibusb>();
  if (!motor->Initialize()) {
    return nullptr;
  }
  return motor;
}

bool LoadAudioFirmware(const std::vector<uint8_t>& image) {
  libusb_context* context;
  if (libusb_init(&context) < 0) {
    return false;
  }
  int interfaces = 0;
  libusb_device_handle* handle = OpenAudioDevice(context, interfaces);
  if (!handle) {
    XELOGW("Kinect: audio device not found");
    libusb_exit(context);
    return false;
  }
  // The bootloader has one interface; the firmware adds the audio ones.
  if (interfaces >= 2) {
    XELOGI("Kinect: audio firmware already running");
    libusb_close(handle);
    libusb_exit(context);
    return true;
  }

  const bool uploaded = Upload(handle, image);
  libusb_close(handle);
  bool running = false;
  if (uploaded) {
    const auto deadline =
        std::chrono::steady_clock::now() + kReenumerateTimeout;
    while (!running && std::chrono::steady_clock::now() < deadline) {
      xe::threading::Sleep(kReenumeratePollInterval);
      interfaces = 0;
      if (libusb_device_handle* reopened =
              OpenAudioDevice(context, interfaces)) {
        libusb_close(reopened);
        running = interfaces >= 2;
      }
    }
    if (running) {
      XELOGI("Kinect: audio firmware running");
    } else {
      XELOGE("Kinect: the audio device did not come back with the firmware");
    }
  }
  libusb_exit(context);
  return running;
}

}  // namespace kinect
}  // namespace hid
}  // namespace xe

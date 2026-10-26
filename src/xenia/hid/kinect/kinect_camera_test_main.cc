/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

// Streams from a Kinect v1 camera on the host for a few seconds and reports
// the frame rate, to tell a USB problem from one in the emulated kernel.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "third_party/fmt/include/fmt/format.h"
#include "xenia/base/console_app_main.h"
#include "xenia/base/cvar.h"
#include "xenia/base/filesystem.h"
#include "xenia/base/string.h"
#include "xenia/base/threading.h"
#include "xenia/hid/kinect/usb_audio.h"
#include "xenia/hid/kinect/usb_camera.h"

DEFINE_int32(seconds, 3, "How long to stream before exiting.", "Kinect");
DEFINE_bool(use_usbdk, true,
            "Access the camera through UsbDk rather than WinUSB. Windows only.",
            "Kinect");
DEFINE_path(record_dir, "",
            "Record the stream into this directory (calibration and every "
            "frame) for kinect_replay_path to play back.",
            "Kinect");
DEFINE_path(depth_pgm, "",
            "With record_dir: instead of streaming, write a one-frame "
            "recording from this 16-bit PGM of raw disparities, as "
            "dump_prefix writes them, with the "
            "connected camera's calibration.",
            "Kinect");
DEFINE_path(audio_firmware, "",
            "Upload this audio firmware (audios.bin) to the sensor, as "
            "kinect_audio_firmware_path does, and exit.",
            "Kinect");
DEFINE_bool(motor_test, false,
            "Instead of streaming, set the LED to led, tilt the motor to tilt "
            "degrees and print the motor state for seconds. Needs the audio "
            "firmware running.",
            "Kinect");
DEFINE_path(audio_wav, "",
            "Instead of streaming the camera, record the microphones for "
            "seconds into <path>_processed.wav (the sensor's mix) and "
            "<path>_microphones.wav (the four microphones). Needs the audio "
            "firmware running.",
            "Kinect");
DEFINE_int32(tilt, 0, "With motor_test: tilt angle in degrees.", "Kinect");
DEFINE_double(beam_angle, 0.0,
              "With motor_test: also point the microphone array's beam this "
              "many radians from straight ahead.",
              "Kinect");
DEFINE_int32(stop_after_ms, -1,
             "With motor_test: stop the movement this long after starting it.",
             "Kinect");
DEFINE_int32(led, 3,
             "With motor_test: 1 off, 2 blinking green, 3 green, 4 red.",
             "Kinect");
DEFINE_string(dump_prefix, "",
              "Write the last frames as <prefix>_depth.pgm (16-bit, raw 11-bit "
              "disparity) and <prefix>_color.pgm (8-bit Bayer).",
              "Kinect");

namespace xe {
namespace hid {
namespace kinect {

namespace {

bool WriteWav(const std::filesystem::path& path, uint16_t channels,
              uint16_t bits, const std::vector<uint8_t>& samples) {
  FILE* file = xe::filesystem::OpenFile(path, "wb");
  if (!file) {
    return false;
  }
  constexpr uint32_t kRate = 16000;
  const uint32_t block = channels * bits / 8;
  const uint32_t data_bytes = uint32_t(samples.size());
  auto put32 = [file](uint32_t v) { std::fwrite(&v, 4, 1, file); };
  auto put16 = [file](uint16_t v) { std::fwrite(&v, 2, 1, file); };
  std::fwrite("RIFF", 1, 4, file);
  put32(36 + data_bytes);
  std::fwrite("WAVEfmt ", 1, 8, file);
  put32(16);
  put16(1);
  put16(channels);
  put32(kRate);
  put32(kRate * block);
  put16(uint16_t(block));
  put16(bits);
  std::fwrite("data", 1, 4, file);
  put32(data_bytes);
  std::fwrite(samples.data(), 1, samples.size(), file);
  std::fclose(file);
  return true;
}

constexpr uint32_t kWidth = 640;
constexpr uint32_t kHeight = 480;
constexpr auto kPollInterval = std::chrono::milliseconds(5);

// Unpacks big-endian 11-bit samples, as xboxkrnl_pscam.cc does.
std::vector<uint16_t> UnpackDepth(const std::vector<uint8_t>& packed) {
  std::vector<uint16_t> out(kWidth * kHeight);
  uint32_t bits = 0;
  int bit_count = 0;
  size_t in = 0;
  for (uint16_t& pixel : out) {
    while (bit_count < 11) {
      bits = (bits << 8) | packed[in++];
      bit_count += 8;
    }
    bit_count -= 11;
    pixel = static_cast<uint16_t>((bits >> bit_count) & 0x7FF);
  }
  return out;
}

// Packs 16-bit samples into big-endian 11-bit ones, as the camera sends them.
std::vector<uint8_t> PackDepth(const std::vector<uint16_t>& depth) {
  std::vector<uint8_t> packed;
  packed.reserve(UsbCamera::kDepthFrameBytes);
  uint32_t bits = 0;
  int bit_count = 0;
  for (uint16_t pixel : depth) {
    bits = (bits << 11) | (pixel & 0x7FF);
    bit_count += 11;
    while (bit_count >= 8) {
      bit_count -= 8;
      packed.push_back(uint8_t(bits >> bit_count));
    }
  }
  return packed;
}

// Reads a 640x480 16-bit big-endian PGM.
bool ReadDepthPgm(const std::filesystem::path& path,
                  std::vector<uint16_t>& out) {
  FILE* file = xe::filesystem::OpenFile(path, "rb");
  if (!file) {
    return false;
  }
  char magic[3] = {};
  uint32_t width = 0, height = 0, max_value = 0;
  bool ok = fscanf(file, "%2s %u %u %u", magic, &width, &height,
                   &max_value) == 4 &&
            std::string(magic) == "P5" && width == kWidth &&
            height == kHeight && max_value > 255;
  fgetc(file);
  std::vector<uint8_t> bytes(kWidth * kHeight * 2);
  ok = ok && fread(bytes.data(), 1, bytes.size(), file) == bytes.size();
  fclose(file);
  if (!ok) {
    return false;
  }
  out.resize(kWidth * kHeight);
  for (size_t i = 0; i < out.size(); ++i) {
    out[i] = uint16_t((bytes[i * 2] << 8) | bytes[i * 2 + 1]);
  }
  return true;
}

bool WritePgm(const std::string& path, const void* pixels, size_t bytes,
              uint32_t max_value) {
  FILE* file = std::fopen(path.c_str(), "wb");
  if (!file) {
    return false;
  }
  const std::string header =
      fmt::format("P5\n{} {}\n{}\n", kWidth, kHeight, max_value);
  bool ok = std::fwrite(header.data(), 1, header.size(), file) ==
                header.size() &&
            std::fwrite(pixels, 1, bytes, file) == bytes;
  std::fclose(file);
  return ok;
}

}  // namespace

int kinect_camera_test_main(const std::vector<std::string>& args) {
  static_cast<void>(args);
  if (!cvars::audio_firmware.empty()) {
    std::vector<uint8_t> image;
    if (FILE* file = xe::filesystem::OpenFile(cvars::audio_firmware, "rb")) {
      uint8_t buffer[0x4000];
      size_t read;
      while ((read = std::fread(buffer, 1, sizeof(buffer), file)) > 0) {
        image.insert(image.end(), buffer, buffer + read);
      }
      std::fclose(file);
    }
    const bool running = LoadAudioFirmware(image);
    fmt::print("Audio firmware {}.\n", running ? "running" : "not running");
    return running ? 0 : 1;
  }
  if (!cvars::audio_wav.empty()) {
    auto audio = UsbAudio::Open();
    if (!audio || !audio->StartMicrophones(nullptr)) {
      fmt::print(stderr, "The Kinect microphones could not be started.\n");
      return 1;
    }
    std::vector<uint8_t> processed, microphones;
    uint64_t next = 0;
    uint16_t last = 0;
    bool have_last = false;
    uint32_t windows = 0, gaps = 0;
    const auto end = std::chrono::steady_clock::now() +
                     std::chrono::seconds(cvars::seconds);
    while (std::chrono::steady_clock::now() < end) {
      UsbAudio::MicrophoneWindow window;
      if (!audio->ReadMicrophoneWindow(next, window)) {
        xe::threading::Sleep(std::chrono::milliseconds(4));
        continue;
      }
      if (have_last && uint16_t(window.window - last) != 3) {
        ++gaps;
      }
      last = window.window;
      have_last = true;
      ++windows;
      auto* p = reinterpret_cast<const uint8_t*>(window.processed);
      processed.insert(processed.end(), p, p + sizeof(window.processed));
      for (uint32_t i = 0; i < UsbAudio::kSamplesPerWindow; ++i) {
        for (uint32_t m = 0; m < UsbAudio::kMicrophoneCount; ++m) {
          auto* q =
              reinterpret_cast<const uint8_t*>(&window.microphones[m][i]);
          microphones.insert(microphones.end(), q, q + 4);
        }
      }
    }
    audio->StopMicrophones();
    const std::string prefix = xe::path_to_utf8(cvars::audio_wav);
    WriteWav(prefix + "_processed.wav", 1, 16, processed);
    WriteWav(prefix + "_microphones.wav", UsbAudio::kMicrophoneCount, 32,
             microphones);
    fmt::print("{} windows ({:.1f} s), {} gaps\n", windows,
               windows * 0.016, gaps);
    return 0;
  }
  if (cvars::motor_test) {
    auto motor = UsbAudio::Open();
    if (!motor) {
      fmt::print(stderr, "No Kinect motor could be opened.\n");
      return 1;
    }
    fmt::print("LED {}: {}\n", cvars::led,
               motor->SetLed(UsbAudio::Led(cvars::led)) ? "ok" : "failed");
    fmt::print("Tilt {}: {}\n", cvars::tilt,
               motor->SetTiltAngle(cvars::tilt) ? "ok" : "failed");
    {
      const float angle = float(cvars::beam_angle);
      const uint32_t words[3] = {UsbAudio::kAudioMessageMagic, 0,
                                 UsbAudio::kAudioMessageBeamAngle};
      std::vector<uint8_t> message(sizeof(words) + sizeof(angle));
      std::memcpy(message.data(), words, sizeof(words));
      std::memcpy(message.data() + sizeof(words), &angle, sizeof(angle));
      fmt::print("Beam angle {}: {}\n", angle,
                 motor->SendAudioMessage(message) ? "ok" : "failed");
    }
    const auto start = std::chrono::steady_clock::now();
    const auto end = start + std::chrono::seconds(cvars::seconds);
    bool stopped = cvars::stop_after_ms < 0;
    while (std::chrono::steady_clock::now() < end) {
      if (!stopped && std::chrono::steady_clock::now() - start >=
                          std::chrono::milliseconds(cvars::stop_after_ms)) {
        stopped = true;
        fmt::print("Stop: {}\n", motor->StopMovement() ? "ok" : "failed");
      }
      UsbAudio::State state;
      if (motor->ReadState(state)) {
        fmt::print("angle {:4} accel {:5} {:5} {:5} flags {:08X}\n",
                   state.tilt_angle, state.accelerometer[0],
                   state.accelerometer[1], state.accelerometer[2],
                   state.flags);
      } else {
        fmt::print("state read failed\n");
      }
      xe::threading::Sleep(std::chrono::milliseconds(100));
    }
    return 0;
  }
  std::atomic<uint32_t> callbacks = 0;
  auto camera = UsbCamera::Open(cvars::use_usbdk, [&]() { ++callbacks; });
  if (!camera) {
    fmt::print(stderr,
               "No Kinect camera could be opened; the log says why "
               "(--log_file=stdout shows it here).\n");
    return 1;
  }
  const UsbCameraCalibration& calibration = camera->calibration();
  fmt::print(
      "Camera open. Calibration: emitter distance {:.4f}, rcmos distance "
      "{:.4f}, reference distance {:.4f}, reference pixel size {:.6f}, "
      "const shift {}\n",
      calibration.dcmos_emitter_distance, calibration.dcmos_rcmos_distance,
      calibration.reference_distance, calibration.reference_pixel_size,
      calibration.const_shift);

  if (!cvars::record_dir.empty()) {
    if (!WriteRecordingCalibration(cvars::record_dir, calibration)) {
      fmt::print(stderr, "Cannot write to {}\n", xe::path_to_utf8(cvars::record_dir));
      return 1;
    }
    if (!cvars::depth_pgm.empty()) {
      std::vector<uint16_t> pgm_depth;
      if (!ReadDepthPgm(cvars::depth_pgm, pgm_depth)) {
        fmt::print(stderr, "{} isn't a 640x480 16-bit PGM\n",
                   xe::path_to_utf8(cvars::depth_pgm));
        return 1;
      }
      // Pixels without depth are 2047 from the camera.
      for (uint16_t& pixel : pgm_depth) {
        if (!pixel) {
          pixel = 2047;
        }
      }
      const bool ok = WriteRecordingFrame(cvars::record_dir, 0,
                                          PackDepth(pgm_depth),
                                          std::vector<uint8_t>(kWidth * kHeight));
      fmt::print("{}: one-frame recording {}\n",
                 xe::path_to_utf8(cvars::record_dir),
                 ok ? "written" : "not written");
      return ok ? 0 : 1;
    }
  }
  uint32_t recorded_frames = 0;

  std::vector<uint8_t> depth;
  std::vector<uint8_t> color;
  std::vector<uint8_t> last_depth;
  std::vector<uint8_t> last_color;
  uint32_t depth_sequence = 0;
  uint32_t color_sequence = 0;
  uint32_t depth_frames = 0;
  uint32_t color_frames = 0;
  const auto start = std::chrono::steady_clock::now();
  const auto deadline = start + std::chrono::seconds(cvars::seconds);
  while (std::chrono::steady_clock::now() < deadline) {
    if (uint32_t sequence = camera->CopyDepthFrame(depth_sequence, depth)) {
      depth_sequence = sequence;
      ++depth_frames;
      last_depth.swap(depth);
      if (!cvars::record_dir.empty()) {
        WriteRecordingFrame(cvars::record_dir, recorded_frames++, last_depth,
                            last_color);
      }
    }
    if (uint32_t sequence = camera->CopyColorFrame(color_sequence, color)) {
      color_sequence = sequence;
      ++color_frames;
      last_color.swap(color);
    }
    xe::threading::Sleep(kPollInterval);
  }
  const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
  fmt::print(
      "{:.1f} s: {} depth frames ({:.1f} fps), {} color frames ({:.1f} fps), "
      "{} frame callbacks\n",
      elapsed, depth_frames, depth_frames / elapsed, color_frames,
      color_frames / elapsed, callbacks.load());
  if (!cvars::record_dir.empty()) {
    fmt::print("Recorded {} frames into {}\n", recorded_frames,
               xe::path_to_utf8(cvars::record_dir));
  }

  if (!cvars::dump_prefix.empty()) {
    if (!last_depth.empty()) {
      std::vector<uint16_t> unpacked = UnpackDepth(last_depth);
      // PGM 16-bit samples are big-endian.
      for (uint16_t& pixel : unpacked) {
        pixel = static_cast<uint16_t>((pixel << 8) | (pixel >> 8));
      }
      const std::string path = cvars::dump_prefix + "_depth.pgm";
      fmt::print("{}: {}\n", path,
                 WritePgm(path, unpacked.data(), unpacked.size() * 2, 2047)
                     ? "written"
                     : "cannot write");
    }
    if (!last_color.empty()) {
      const std::string path = cvars::dump_prefix + "_color.pgm";
      fmt::print("{}: {}\n", path,
                 WritePgm(path, last_color.data(), last_color.size(), 255)
                     ? "written"
                     : "cannot write");
    }
  }

  // Closing the camera stops its streams and returns it to its driver.
  camera.reset();
  return depth_frames && color_frames ? 0 : 2;
}

}  // namespace kinect
}  // namespace hid
}  // namespace xe

XE_DEFINE_CONSOLE_APP("xenia-kinect-camera-test",
                      xe::hid::kinect::kinect_camera_test_main, "");

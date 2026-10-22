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
#include <filesystem>
#include <string>
#include <vector>

#include "third_party/fmt/include/fmt/format.h"
#include "xenia/base/console_app_main.h"
#include "xenia/base/cvar.h"
#include "xenia/base/filesystem.h"
#include "xenia/base/string.h"
#include "xenia/base/threading.h"
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
            "kinect_dump_depth_path or dump_prefix write them, with the "
            "connected camera's calibration.",
            "Kinect");
DEFINE_string(dump_prefix, "",
              "Write the last frames as <prefix>_depth.pgm (16-bit, raw 11-bit "
              "disparity) and <prefix>_color.pgm (8-bit Bayer).",
              "Kinect");

namespace xe {
namespace hid {
namespace kinect {

namespace {

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

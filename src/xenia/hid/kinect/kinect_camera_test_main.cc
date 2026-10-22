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
#include <string>
#include <vector>

#include "third_party/fmt/include/fmt/format.h"
#include "xenia/base/console_app_main.h"
#include "xenia/base/cvar.h"
#include "xenia/base/threading.h"
#include "xenia/hid/kinect/usb_camera.h"

DEFINE_int32(seconds, 3, "How long to stream before exiting.", "Kinect");
DEFINE_bool(use_usbdk, true,
            "Access the camera through UsbDk rather than WinUSB. Windows only.",
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

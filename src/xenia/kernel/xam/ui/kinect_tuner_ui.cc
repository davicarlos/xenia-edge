/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/kernel/xam/ui/kinect_tuner_ui.h"

#include <algorithm>
#include <cmath>

#include "third_party/imgui/imgui.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/nui.h"
#include "xenia/kernel/xboxkrnl/xboxkrnl_private.h"
#include "xenia/ui/imgui_drawer.h"

namespace xe {
namespace kernel {
namespace xam {
namespace ui {

namespace {

constexpr uint32_t kFrameWidth = 640;
constexpr uint32_t kFrameHeight = 480;
constexpr size_t kPackedDepthBytes = kFrameWidth * kFrameHeight * 11 / 8;
constexpr size_t kBayerBytes = kFrameWidth * kFrameHeight;
constexpr uint16_t kNoDepthRaw = 2047;
constexpr auto kRefreshInterval = std::chrono::milliseconds(66);
constexpr size_t kRetiredTextureCount = 8;
// The distances the skeleton tracker works at, in millimetres.
constexpr float kNearLimitMm = 1200.0f;
constexpr float kFarLimitMm = 3500.0f;

// Approximate distance of a raw 11-bit disparity, in millimetres, after
// Stéphane Magnenat's fit used with libfreenect.
float RawToMillimetres(uint16_t raw) {
  return 1000.0f * 0.1236f * std::tan(float(raw) / 2842.5f + 1.1863f);
}

// Unpacks big-endian 11-bit samples.
void UnpackDepth(const std::vector<uint8_t>& packed,
                 std::vector<uint16_t>& out) {
  out.resize(kFrameWidth * kFrameHeight);
  uint32_t bits = 0;
  int bit_count = 0;
  size_t in = 0;
  for (uint16_t& pixel : out) {
    while (bit_count < 11) {
      bits = (bits << 8) | packed[in++];
      bit_count += 8;
    }
    bit_count -= 11;
    pixel = uint16_t((bits >> bit_count) & 0x7FF);
  }
}

}  // namespace

KinectTunerUI::KinectTunerUI(xe::ui::ImGuiDrawer* imgui_drawer,
                             xe::hid::InputSystem* input_system)
    : XamGamepadDialog(imgui_drawer, input_system),
      elevation_(kernel_state()->nui()->GetElevationAngle()) {}

void KinectTunerUI::Refresh() {
  camera_present_ = xboxkrnl::CopyKinectCameraPreview(
      depth_sequence_, packed_depth_, color_sequence_, bayer_);
  if (!camera_present_) {
    return;
  }

  if (packed_depth_.size() == kPackedDepthBytes) {
    static thread_local std::vector<uint16_t> depth;
    UnpackDepth(packed_depth_, depth);
    depth_rgba_.resize(kPreviewWidth * kPreviewHeight * 4);
    uint32_t near_count = 0, good_count = 0, far_count = 0;
    for (uint32_t y = 0; y < kPreviewHeight; ++y) {
      for (uint32_t x = 0; x < kPreviewWidth; ++x) {
        const uint16_t raw = depth[(y * 2) * kFrameWidth + x * 2];
        uint8_t* texel = &depth_rgba_[(y * kPreviewWidth + x) * 4];
        texel[3] = 255;
        if (raw == 0 || raw >= kNoDepthRaw) {
          texel[0] = texel[1] = texel[2] = 0;
          continue;
        }
        const float mm = RawToMillimetres(raw);
        // Brighter is closer within each band.
        if (mm < kNearLimitMm) {
          ++near_count;
          const float t = std::clamp(mm / kNearLimitMm, 0.0f, 1.0f);
          texel[0] = uint8_t(255 - t * 100);
          texel[1] = 40;
          texel[2] = 40;
        } else if (mm <= kFarLimitMm) {
          ++good_count;
          const float t = (mm - kNearLimitMm) / (kFarLimitMm - kNearLimitMm);
          texel[0] = 30;
          texel[1] = uint8_t(255 - t * 170);
          texel[2] = 60;
        } else {
          ++far_count;
          const float t = std::clamp((mm - kFarLimitMm) / 4000.0f, 0.0f, 1.0f);
          texel[0] = 30;
          texel[1] = 50;
          texel[2] = uint8_t(200 - t * 140);
        }
      }
    }
    const float total = float(kPreviewWidth * kPreviewHeight);
    near_share_ = near_count / total;
    good_share_ = good_count / total;
    far_share_ = far_count / total;
    if (depth_texture_) {
      retired_textures_.push_back(std::move(depth_texture_));
    }
    depth_texture_ = imgui_drawer()->CreateRgbaTexture(
        kPreviewWidth, kPreviewHeight, depth_rgba_.data());
  }

  if (bayer_.size() == kBayerBytes) {
    // Each 2x2 Bayer cell (G R / B G) becomes one texel.
    color_rgba_.resize(kPreviewWidth * kPreviewHeight * 4);
    for (uint32_t y = 0; y < kPreviewHeight; ++y) {
      const uint8_t* row0 = &bayer_[(y * 2) * kFrameWidth];
      const uint8_t* row1 = row0 + kFrameWidth;
      for (uint32_t x = 0; x < kPreviewWidth; ++x) {
        uint8_t* texel = &color_rgba_[(y * kPreviewWidth + x) * 4];
        texel[0] = row0[x * 2 + 1];
        texel[1] = uint8_t((uint32_t(row0[x * 2]) + row1[x * 2 + 1]) / 2);
        texel[2] = row1[x * 2];
        texel[3] = 255;
      }
    }
    if (color_texture_) {
      retired_textures_.push_back(std::move(color_texture_));
    }
    color_texture_ = imgui_drawer()->CreateRgbaTexture(
        kPreviewWidth, kPreviewHeight, color_rgba_.data());
  }

  while (retired_textures_.size() > kRetiredTextureCount) {
    retired_textures_.pop_front();
  }
}

void KinectTunerUI::OnDraw(ImGuiIO& io) {
  const auto now = std::chrono::steady_clock::now();
  if (now - last_refresh_ >= kRefreshInterval) {
    last_refresh_ = now;
    Refresh();
  }

  if (!has_opened_) {
    ImGui::SetNextWindowPos(
        ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f),
        ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    has_opened_ = true;
  }
  bool open = true;
  if (!ImGui::Begin("Kinect Tuner##kinect_tuner", &open,
                    ImGuiWindowFlags_AlwaysAutoResize |
                        ImGuiWindowFlags_NoCollapse)) {
    ImGui::End();
    return;
  }

  if (!camera_present_) {
    ImGui::TextWrapped(
        "No Kinect camera is in use. Enable allow_kinect_camera and connect "
        "a Kinect for Xbox 360.");
  } else {
    ImGui::Text("Depth");
    ImGui::SameLine(kPreviewWidth + 16.0f);
    ImGui::Text("Color");
    if (depth_texture_) {
      ImGui::Image(reinterpret_cast<ImTextureID>(depth_texture_.get()),
                   ImVec2(float(kPreviewWidth), float(kPreviewHeight)));
    } else {
      ImGui::Dummy(ImVec2(float(kPreviewWidth), float(kPreviewHeight)));
    }
    ImGui::SameLine();
    if (color_texture_) {
      ImGui::Image(reinterpret_cast<ImTextureID>(color_texture_.get()),
                   ImVec2(float(kPreviewWidth), float(kPreviewHeight)));
    } else {
      ImGui::Dummy(ImVec2(float(kPreviewWidth), float(kPreviewHeight)));
    }

    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f),
                       "Closer than 1.2 m: %3.0f%%", near_share_ * 100.0f);
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.5f, 1.0f),
                       "  1.2 - 3.5 m: %3.0f%%", good_share_ * 100.0f);
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.5f, 0.6f, 1.0f, 1.0f),
                       "  Farther: %3.0f%%", far_share_ * 100.0f);
    ImGui::TextWrapped(
        "Stand 1.8 to 3 m from the sensor, facing it, with your whole body "
        "including your feet in the depth image. Green is the range the "
        "skeleton tracker uses; red is too close.");
  }

  ImGui::Separator();
  ImGui::Text("Camera elevation reported to the game");
  if (ImGui::SliderInt("degrees", &elevation_, NUIController::kElevationMinimum,
                       NUIController::kElevationMaximum)) {
    kernel_state()->nui()->SetElevationAngle(elevation_);
  }
  ImGui::TextDisabled(
      "The sensor doesn't tilt: model 1473 Kinects drive their motor through "
      "firmware Xenia doesn't have. Tilt the sensor by hand to match.");

  ImGui::Spacing();
  if (ImGui::Button("Close") || !open ||
      ImGui::IsKeyPressed(ImGuiKey_Escape) || ShouldCloseFromGamepad()) {
    ImGui::End();
    Close();
    return;
  }
  ImGui::End();
}

}  // namespace ui
}  // namespace xam
}  // namespace kernel
}  // namespace xe

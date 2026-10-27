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
#include <array>
#include <cmath>
#include <cstdio>
#include <utility>

#include "third_party/imgui/imgui.h"
#include "xenia/config.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/nui.h"
#include "xenia/hid/kinect/usb_audio.h"
#include "xenia/kernel/kinect/kinect_audio.h"
#include "xenia/kernel/kinect/kinect_device.h"
#include "xenia/kernel/xboxkrnl/xboxkrnl_private.h"
#include "xenia/ui/imgui_drawer.h"

DECLARE_bool(kinect_elevation_calibrated);
DECLARE_int32(kinect_elevation);

namespace xe {
namespace kernel {
namespace xam {
namespace ui {

namespace {

// What the console's Kinect Tuner saves: the tilt in XCONFIG_CONSOLE_NUI,
// with its calibrated flag (0x2), which titles go straight to.
void SaveCalibratedElevation(int32_t degrees) {
  auto* xconfig = kernel_state()->xconfig();
  uint8_t data[0x1C];
  xconfig->ReadSetting(X_CONFIG_CATEGORY::XCONFIG_CONSOLE_CATEGORY,
                       XCONFIG_CONSOLE_NUI, data);
  xe::store_and_swap<uint32_t>(data, xe::load_and_swap<uint32_t>(data) | 0x2);
  xe::store_and_swap<int32_t>(data + 8, degrees);
  xconfig->WriteSetting(X_CONFIG_CATEGORY::XCONFIG_CONSOLE_CATEGORY,
                        XCONFIG_CONSOLE_NUI, data);
}

}  // namespace

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

// NUI_SKELETON_POSITION_INDEX pairs joined by bones.
constexpr std::pair<uint8_t, uint8_t> kBones[] = {
    {0, 1},   {1, 2},   {2, 3},                       // Spine and head.
    {2, 4},   {4, 5},   {5, 6},   {6, 7},             // Left arm.
    {2, 8},   {8, 9},   {9, 10},  {10, 11},           // Right arm.
    {0, 12},  {12, 13}, {13, 14}, {14, 15},           // Left leg.
    {0, 16},  {16, 17}, {17, 18}, {18, 19},           // Right leg.
};
// NUI_CAMERA_SKELETON_TO_DEPTH_IMAGE_MULTIPLIER_320x240: pixels of a 320x240
// depth image per metre of x or y at a depth of 1 metre.
constexpr float kSkeletonToDepth320 = 285.63f;
constexpr ImU32 kSkeletonColors[NuiSkeletonSnapshot::kSkeletonCount] = {
    IM_COL32(255, 230, 60, 255),  IM_COL32(80, 220, 255, 255),
    IM_COL32(255, 120, 220, 255), IM_COL32(140, 255, 120, 255),
    IM_COL32(255, 160, 80, 255),  IM_COL32(200, 160, 255, 255)};

}  // namespace

void KinectTunerUI::DrawSkeletons(ImVec2 origin) {
  const NuiSkeletonSnapshot snapshot =
      kernel_state()->nui()->GetSkeletonSnapshot();
  skeleton_frame_number_ = snapshot.frame_number;
  skeletons_tracked_ = 0;
  skeletons_position_only_ = 0;
  if (!snapshot.frame_number) {
    return;
  }
  // The preview is the sensor's image shown as a mirror, which skeleton x
  // (to the sensor's left) maps onto directly.
  auto project = [&](const std::array<float, 3>& p, ImVec2& out) {
    if (p[2] <= 0.1f) {
      return false;
    }
    const float u = kSkeletonToDepth320 * p[0] / p[2];
    out = ImVec2(origin.x + float(kPreviewWidth) * 0.5f + u,
                 origin.y + float(kPreviewHeight) * 0.5f -
                     kSkeletonToDepth320 * p[1] / p[2]);
    return true;
  };
  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  draw_list->PushClipRect(
      origin, ImVec2(origin.x + kPreviewWidth, origin.y + kPreviewHeight),
      true);
  for (uint32_t i = 0; i < NuiSkeletonSnapshot::kSkeletonCount; ++i) {
    const NuiSkeletonSnapshot::Skeleton& skeleton = snapshot.skeletons[i];
    const ImU32 color = kSkeletonColors[i];
    const ImU32 inferred_color = (color & 0x00FFFFFF) | 0x80000000;
    ImVec2 point;
    if (skeleton.state ==
        NuiSkeletonSnapshot::TrackingState::kPositionOnly) {
      ++skeletons_position_only_;
      if (project(skeleton.position, point)) {
        draw_list->AddCircle(point, 10.0f, color, 16, 2.0f);
      }
      continue;
    }
    if (skeleton.state != NuiSkeletonSnapshot::TrackingState::kTracked) {
      continue;
    }
    ++skeletons_tracked_;
    std::array<ImVec2, NuiSkeletonSnapshot::kJointCount> points;
    std::array<bool, NuiSkeletonSnapshot::kJointCount> visible;
    for (uint32_t j = 0; j < NuiSkeletonSnapshot::kJointCount; ++j) {
      visible[j] =
          skeleton.joint_states[j] && project(skeleton.joints[j], points[j]);
    }
    for (const auto& [a, b] : kBones) {
      if (!visible[a] || !visible[b]) {
        continue;
      }
      // A bone is only as sure as its least sure end.
      const bool both_tracked =
          skeleton.joint_states[a] == 2 && skeleton.joint_states[b] == 2;
      draw_list->AddLine(points[a], points[b],
                         both_tracked ? color : inferred_color, 3.0f);
    }
    for (uint32_t j = 0; j < NuiSkeletonSnapshot::kJointCount; ++j) {
      if (visible[j]) {
        draw_list->AddCircleFilled(
            points[j], j == 3 ? 6.0f : 3.5f,
            skeleton.joint_states[j] == 2 ? color : inferred_color);
      }
    }
  }
  draw_list->PopClipRect();
}

KinectTunerUI::KinectTunerUI(xe::ui::ImGuiDrawer* imgui_drawer,
                             xe::hid::InputSystem* input_system)
    : XamGamepadDialog(imgui_drawer, input_system),
      elevation_(kernel_state()->nui()->GetElevationAngle()) {}

// Rebuilds the two previews from the newest frames, at half resolution: the
// depth colored by distance band, the color from the raw Bayer image.
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
        // Shown as a mirror, like the skeletons drawn over it.
        const uint16_t raw =
            depth[(y * 2) * kFrameWidth + (kFrameWidth - 2 - x * 2)];
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
        // Mirrored like the depth preview; cells stay aligned to the pattern.
        const uint32_t cell = kFrameWidth - 2 - x * 2;
        texel[0] = row0[cell + 1];
        texel[1] = uint8_t((uint32_t(row0[cell]) + row1[cell + 1]) / 2);
        texel[2] = row1[cell];
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
  if (close_requested_) {
    Close();
    return;
  }

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
    DrawSkeletons(ImGui::GetItemRectMin());
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
    if (!skeleton_frame_number_) {
      ImGui::TextDisabled("The game hasn't reported any skeleton yet.");
    } else {
      ImGui::Text("Players tracked by the game: %u (%u by position only)",
                  skeletons_tracked_, skeletons_position_only_);
    }
    ImGui::TextWrapped(
        "Stand 1.8 to 3 m from the sensor, facing it, with your whole body "
        "including your feet in the depth image. Green is the range the "
        "skeleton tracker uses; red is too close.");
  }

  ImGui::Separator();
  hid::kinect::UsbAudio::State tilt;
  if (kernel_state()->nui()->device()->ReadMotorState(tilt)) {
    // Gravity from the accelerometer gives the sensor's real elevation.
    const float angle = std::atan2(float(tilt.gravity[2]),
                                   float(tilt.gravity[1])) *
                        180.0f / 3.14159265f;
    ImGui::Text("Sensor elevation: %+.0f degrees%s", angle,
                tilt.flags & 3 ? " (moving)" : "");
    ImGui::TextDisabled("Accelerometer: x %d  y %d  z %d",
                        tilt.accelerometer[0], tilt.accelerometer[1],
                        tilt.accelerometer[2]);
    if (ImGui::SliderInt("Tilt to", &elevation_,
                         NUIController::kElevationMinimum,
                         NUIController::kElevationMaximum, "%d degrees")) {
      kernel_state()->nui()->SetElevationAngle(elevation_);
    }
    // Titles tilt straight to the elevation saved here from then on, as on a
    // console calibrated in its Kinect Tuner. Saved on release, not per drag.
    if (ImGui::IsItemDeactivatedAfterEdit()) {
      SaveCalibratedElevation(elevation_);
    }
    if (cvars::kinect_elevation_calibrated) {
      ImGui::TextDisabled(
          "Titles tilt the sensor to %+d degrees when they start.",
          cvars::kinect_elevation);
    } else {
      ImGui::TextDisabled(
          "Titles look for the floor themselves until a tilt is saved here.");
    }
  } else {
    ImGui::Text("Camera elevation reported to the game");
    if (ImGui::SliderInt("degrees", &elevation_,
                         NUIController::kElevationMinimum,
                         NUIController::kElevationMaximum)) {
      kernel_state()->nui()->SetElevationAngle(elevation_);
    }
    ImGui::TextDisabled(
        "The sensor doesn't tilt: model 1473 Kinects drive their motor "
        "through Microsoft's audio firmware; set kinect_audio_firmware_path "
        "to use it. Tilt the sensor by hand to match meanwhile.");
  }

  ImGui::Separator();
  kinect::KinectAudio* audio = kernel_state()->nui()->audio();
  if (!audio->microphones_available()) {
    ImGui::TextDisabled("No Kinect microphones (they need the audio firmware "
                        "too).");
  } else if (!audio->microphones_in_use()) {
    ImGui::TextDisabled("The game isn't using the Kinect microphones yet.");
  } else {
    // Shown in dB so speech stands out from a quiet room.
    const float peak = std::max(audio->microphone_peak(), 1e-5f);
    const float decibels = 20.0f * std::log10(peak);
    char label[32];
    std::snprintf(label, sizeof(label), "%.0f dB", decibels);
    ImGui::Text("Microphones");
    ImGui::SameLine();
    ImGui::ProgressBar(std::clamp((decibels + 60.0f) / 60.0f, 0.0f, 1.0f),
                       ImVec2(float(kPreviewWidth), 0.0f), label);
  }

  ImGui::Spacing();
  if (ImGui::Button("Close") || !open ||
      ImGui::IsKeyPressed(ImGuiKey_Escape) || ShouldCloseFromGamepad()) {
    // This frame has drawn the previews already.
    close_requested_ = true;
  }
  ImGui::End();
}

}  // namespace ui
}  // namespace xam
}  // namespace kernel
}  // namespace xe

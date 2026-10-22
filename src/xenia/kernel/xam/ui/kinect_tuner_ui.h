/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_KERNEL_XAM_UI_KINECT_TUNER_UI_H_
#define XENIA_KERNEL_XAM_UI_KINECT_TUNER_UI_H_

#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <vector>

#include "xenia/kernel/xam/xam_ui.h"
#include "xenia/ui/immediate_drawer.h"

namespace xe {
namespace kernel {
namespace xam {
namespace ui {

// The Kinect Tuner that XamShowNuiTroubleshooterUI opens: shows what the
// sensor sees, says whether the player stands at a distance the skeleton
// tracker can use, and sets the camera elevation the title is told about.
// The Kinect for Xbox 360 model 1473 drives its motor through the audio
// device, which needs Microsoft's audio firmware, so the elevation is the
// simulated motor's and the sensor itself doesn't tilt.
class KinectTunerUI final : public XamGamepadDialog {
 public:
  KinectTunerUI(xe::ui::ImGuiDrawer* imgui_drawer,
                xe::hid::InputSystem* input_system);

 protected:
  void OnDraw(ImGuiIO& io) override;

 private:
  static constexpr uint32_t kPreviewWidth = 320;
  static constexpr uint32_t kPreviewHeight = 240;

  void Refresh();

  bool has_opened_ = false;
  bool camera_present_ = false;
  std::chrono::steady_clock::time_point last_refresh_;
  uint32_t depth_sequence_ = 0;
  uint32_t color_sequence_ = 0;
  std::vector<uint8_t> packed_depth_;
  std::vector<uint8_t> bayer_;
  std::vector<uint8_t> depth_rgba_;
  std::vector<uint8_t> color_rgba_;
  std::unique_ptr<xe::ui::ImmediateTexture> depth_texture_;
  std::unique_ptr<xe::ui::ImmediateTexture> color_texture_;
  // Replaced textures, kept until frames that may still draw them are done.
  std::deque<std::unique_ptr<xe::ui::ImmediateTexture>> retired_textures_;
  // Shares of the image too close, in the tracker's range and too far.
  float near_share_ = 0.0f;
  float good_share_ = 0.0f;
  float far_share_ = 0.0f;
  int elevation_ = 0;
};

}  // namespace ui
}  // namespace xam
}  // namespace kernel
}  // namespace xe

#endif  // XENIA_KERNEL_XAM_UI_KINECT_TUNER_UI_H_

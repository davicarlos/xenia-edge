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

#include "third_party/imgui/imgui.h"
#include "xenia/kernel/xam/xam_ui.h"
#include "xenia/ui/immediate_drawer.h"

namespace xe {
namespace kernel {
namespace xam {
namespace ui {

// The Kinect Tuner that XamShowNuiTroubleshooterUI opens: shows what the
// sensor sees, says whether the player stands at a distance the skeleton
// tracker can use, tilts the sensor (or, without its motor, sets the
// elevation the title is told about) and shows the microphone level the title
// gets.
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
  // Draws the game's skeletons over the depth preview at origin.
  void DrawSkeletons(ImVec2 origin);

  bool has_opened_ = false;
  // Closing destroys the preview textures, so it waits for a frame that
  // hasn't drawn them - the GPU would read them after they're gone otherwise.
  bool close_requested_ = false;
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
  uint64_t skeleton_frame_number_ = 0;
  uint32_t skeletons_tracked_ = 0;
  uint32_t skeletons_position_only_ = 0;
};

}  // namespace ui
}  // namespace xam
}  // namespace kernel
}  // namespace xe

#endif  // XENIA_KERNEL_XAM_UI_KINECT_TUNER_UI_H_

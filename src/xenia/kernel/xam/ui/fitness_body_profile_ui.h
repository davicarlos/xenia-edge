/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_KERNEL_XAM_UI_FITNESS_BODY_PROFILE_UI_H_
#define XENIA_KERNEL_XAM_UI_FITNESS_BODY_PROFILE_UI_H_

#include <string>

#include "xenia/kernel/xam/xam_fitness.h"
#include "xenia/kernel/xam/xam_ui.h"

namespace xe {
namespace kernel {
namespace xam {
namespace ui {

// The system's fitness body profile screen (XamShowFitnessBodyProfileUI),
// where a player enters what titles estimate their calories from.
class FitnessBodyProfileUI final : public XamGamepadDialog {
 public:
  FitnessBodyProfileUI(xe::ui::ImGuiDrawer* imgui_drawer,
                       xe::hid::InputSystem* input_system, uint64_t xuid,
                       const std::string& gamertag);

 private:
  void OnDraw(ImGuiIO& io) override;

  bool has_opened_ = false;
  uint64_t xuid_;
  std::string title_;
  int female_ = 0;
  int height_centimeters_ = 175;
  float weight_kilograms_ = 75.0f;
  int birth_year_ = 1998;
};

}  // namespace ui
}  // namespace xam
}  // namespace kernel
}  // namespace xe

#endif

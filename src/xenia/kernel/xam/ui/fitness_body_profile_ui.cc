/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/kernel/xam/ui/fitness_body_profile_ui.h"

#include <algorithm>
#include <cmath>

#include "third_party/fmt/include/fmt/format.h"

namespace xe {
namespace kernel {
namespace xam {
namespace ui {

FitnessBodyProfileUI::FitnessBodyProfileUI(xe::ui::ImGuiDrawer* imgui_drawer,
                                           xe::hid::InputSystem* input_system,
                                           uint64_t xuid,
                                           const std::string& gamertag)
    : XamGamepadDialog(imgui_drawer, input_system),
      xuid_(xuid),
      title_(fmt::format("Body Profile - {}###FitnessBodyProfile", gamertag)) {
  if (const auto profile = LoadFitnessBodyProfile(xuid)) {
    female_ = profile->female ? 1 : 0;
    height_centimeters_ = int(std::lround(profile->height_meters * 100.0f));
    weight_kilograms_ = profile->weight_kilograms;
    birth_year_ = profile->birth_year;
  }
}

void FitnessBodyProfileUI::OnDraw(ImGuiIO& io) {
  if (!has_opened_) {
    ImGui::OpenPopup(title_.c_str());
    has_opened_ = true;
  }
  const ImVec2 center(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f);
  ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
  if (!ImGui::BeginPopupModal(title_.c_str(), nullptr,
                              ImGuiWindowFlags_AlwaysAutoResize)) {
    Close();
    return;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_Escape) || ShouldCloseFromGamepad()) {
    ImGui::CloseCurrentPopup();
    Close();
    ImGui::EndPopup();
    return;
  }
  ImGui::TextUnformatted(
      "Kinect titles estimate the calories you burn from these.");
  ImGui::Spacing();
  ImGui::RadioButton("Male", &female_, 0);
  ImGui::SameLine();
  ImGui::RadioButton("Female", &female_, 1);
  ImGui::SetNextItemWidth(160.0f);
  ImGui::InputInt("Height (cm)", &height_centimeters_);
  ImGui::SetNextItemWidth(160.0f);
  ImGui::InputFloat("Weight (kg)", &weight_kilograms_, 0.5f, 5.0f, "%.1f");
  ImGui::SetNextItemWidth(160.0f);
  ImGui::InputInt("Year of birth", &birth_year_);
  height_centimeters_ = std::clamp(height_centimeters_, 50, 250);
  weight_kilograms_ = std::clamp(weight_kilograms_, 20.0f, 300.0f);
  birth_year_ = std::clamp(birth_year_, 1900, 2030);
  ImGui::Spacing();
  if (ImGui::Button("Save")) {
    FitnessBodyProfile profile;
    profile.female = female_ != 0;
    profile.height_meters = height_centimeters_ / 100.0f;
    profile.weight_kilograms = weight_kilograms_;
    profile.birth_year = uint16_t(birth_year_);
    SaveFitnessBodyProfile(xuid_, profile);
    ImGui::CloseCurrentPopup();
    Close();
  }
  ImGui::SameLine();
  if (ImGui::Button("Cancel")) {
    ImGui::CloseCurrentPopup();
    Close();
  }
  ImGui::EndPopup();
}

}  // namespace ui
}  // namespace xam
}  // namespace kernel
}  // namespace xe

/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_KERNEL_XAM_XAM_FITNESS_H_
#define XENIA_KERNEL_XAM_XAM_FITNESS_H_

#include <cstdint>
#include <optional>

namespace xe {
namespace kernel {
namespace xam {

// A profile's fitness body profile, which the console keeps with its Xbox
// Live fitness data and titles read with XamFitnessGetCurrentBodyProfileRecord
// to estimate the calories a player burns (4D53090F, 0x82F55B80).
struct FitnessBodyProfile {
  bool female = false;
  float height_meters = 1.75f;
  float weight_kilograms = 75.0f;
  uint16_t birth_year = 1998;
};

// Kept in the storage root, one file per profile.
std::optional<FitnessBodyProfile> LoadFitnessBodyProfile(uint64_t xuid);
bool SaveFitnessBodyProfile(uint64_t xuid, const FitnessBodyProfile& profile);

}  // namespace xam
}  // namespace kernel
}  // namespace xe

#endif  // XENIA_KERNEL_XAM_XAM_FITNESS_H_

/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2022 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_KERNEL_XBOXKRNL_XBOXKRNL_PRIVATE_H_
#define XENIA_KERNEL_XBOXKRNL_XBOXKRNL_PRIVATE_H_

#include <vector>

#include "xenia/cpu/export_resolver.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/xboxkrnl/xboxkrnl_ordinals.h"

namespace xe {
namespace kernel {
namespace xboxkrnl {

xe::cpu::Export* RegisterExport_xboxkrnl(xe::cpu::Export* export_entry);

// Registration functions, one per file.
#define XE_MODULE_EXPORT_GROUP(m, n)                                  \
  void Register##n##Exports(xe::cpu::ExportResolver* export_resolver, \
                            KernelState* kernel_state);
#include "xboxkrnl_module_export_groups.inc"
#undef XE_MODULE_EXPORT_GROUP

// Stops the Kinect camera worker before the kernel goes away.
void ShutdownCameraService();
// For the Kinect Tuner: the camera's newest frames, if newer than the
// sequence numbers given, which are updated. False without a camera.
bool CopyKinectCameraPreview(uint32_t& depth_sequence,
                             std::vector<uint8_t>& packed_depth,
                             uint32_t& color_sequence,
                             std::vector<uint8_t>& bayer);

}  // namespace xboxkrnl
}  // namespace kernel
}  // namespace xe

#endif  // XENIA_KERNEL_XBOXKRNL_XBOXKRNL_PRIVATE_H_

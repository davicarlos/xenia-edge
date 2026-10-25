/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2019 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_KERNEL_XAM_XAM_PRIVATE_H_
#define XENIA_KERNEL_XAM_XAM_PRIVATE_H_

#include "xenia/base/byte_order.h"
#include "xenia/cpu/export_resolver.h"
#include "xenia/xbox.h"
#include "xenia/kernel/xam/xam_ordinals.h"

namespace xe {
namespace kernel {
namespace xam {

xe::cpu::Export* RegisterExport_xam(xe::cpu::Export* export_entry);

// XGraphics' 2D texture header (XGSetTextureHeaderEx): fills |texture|, 13
// big-endian dwords, when given, and returns the size of the levels.
uint32_t xeXGSetTextureHeaderEx(uint32_t width, uint32_t height,
                                uint32_t levels, uint32_t usage,
                                uint32_t d3d_format, int32_t exp_bias,
                                uint32_t flags, uint32_t base_offset,
                                uint32_t mip_offset, uint32_t pitch_bytes,
                                xe::be<uint32_t>* texture,
                                uint32_t* base_size_out,
                                uint32_t* mip_size_out);
void xeXGOffsetResourceAddress(xe::be<uint32_t>* resource, uint32_t offset);

// The NUI HUD's handling of a skeleton frame (XamNuiHudInterpretFrame): the
// tracked players, position warnings, engagement and the guide gesture.
X_RESULT xeXamNuiHudInterpretFrame(uint8_t* frame);

// Registration functions, one per file.
#define XE_MODULE_EXPORT_GROUP(m, n)                                  \
  void Register##n##Exports(xe::cpu::ExportResolver* export_resolver, \
                            KernelState* kernel_state);
#include "xam_module_export_groups.inc"
#undef XE_MODULE_EXPORT_GROUP

}  // namespace xam
}  // namespace kernel
}  // namespace xe

#endif  // XENIA_KERNEL_XAM_XAM_PRIVATE_H_

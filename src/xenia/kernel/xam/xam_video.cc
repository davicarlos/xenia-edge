/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2022 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include <algorithm>

#include "xenia/kernel/util/shim_utils.h"
#include "xenia/kernel/xam/xam_private.h"
#include "xenia/kernel/xboxkrnl/xboxkrnl_video.h"

DEFINE_bool(enable_3d_mode, false,
            "Enables usage of stereoscopic mode in titles.", "Video");

namespace xe {
namespace kernel {
namespace xam {

enum X_VIDEO_CAPABILITIES : uint32_t {
  X_VIDEO_ENABLE_STEREOSCOPIC_3D_MODE = 0x1,
  X_VIDEO_ENABLE_STEREOSCOPIC_720P_60HZ_MODE = 0x10
};

void XGetVideoMode_entry(pointer_t<X_VIDEO_MODE> video_mode) {
  // TODO(benvanik): actually check to see if these are the same.
  xboxkrnl::VdQueryVideoMode(video_mode, false);
}
DECLARE_XAM_EXPORT1(XGetVideoMode, kVideo, kSketchy);

dword_result_t XGetVideoCapabilities_entry() {
  uint32_t capabilities = 0;

  if (cvars::enable_3d_mode) {
    capabilities =
        static_cast<uint32_t>(
            X_VIDEO_CAPABILITIES::X_VIDEO_ENABLE_STEREOSCOPIC_3D_MODE) |
        static_cast<uint32_t>(
            X_VIDEO_CAPABILITIES::X_VIDEO_ENABLE_STEREOSCOPIC_720P_60HZ_MODE);
  }

  return capabilities;
}
DECLARE_XAM_EXPORT1(XGetVideoCapabilities, kVideo, kStub);


// XGraphics texture headers, from xam's copy of the library (0x817F7C38,
// 0x817F79C0, 0x81838CB0, 0x81837BB8 and 0x81837A98), for 2D textures: the
// D3D resource fields and the GPU texture fetch constant, with the layout of
// the levels in memory.
namespace {

// Bits per texel, and block width and height, of each GPU texture format
// (0x8160C121 and 0x817F6FD0).
constexpr uint8_t kTextureFormatBits[64] = {
    1,  1,  8,  16, 16, 16, 32, 32, 8,  8,  16, 16, 16, 32, 32, 16,
    32, 32, 4,  8,  8,  64, 32, 32, 16, 32, 64, 16, 32, 64, 16, 32,
    64, 32, 64, 128, 32, 64, 128, 8, 16, 16, 32, 8,  8,  16, 16, 16,
    32, 8,  32, 4,  8,  8,  32, 32, 32, 96, 4,  4,  4,  4,  32, 32};

void GetTextureFormatBlock(uint32_t format, uint32_t& width,
                           uint32_t& height) {
  width = height = 1;
  if (format == 0x2C || format == 0x27) {
    width = 4;
  } else if (format == 0x28 || (format >= 0x0B && format <= 0x0C)) {
    width = 2;
  } else if ((format >= 0x12 && format <= 0x14) || format == 0x31 ||
             (format >= 0x33 && format <= 0x35) ||
             (format >= 0x3A && format <= 0x3D)) {
    width = height = 4;
  }
}

uint32_t AlignUp(uint32_t value, uint32_t alignment) {
  return (value + alignment - 1) & ~(alignment - 1);
}

uint32_t Log2Ceil(uint32_t value) {
  uint32_t log = 0;
  while ((1u << log) < value) {
    ++log;
  }
  return log;
}

// One level: pads the pitch and height to the GPU's tiles and returns its
// size, 4 KB aligned (0x81837A98).
uint32_t LayoutTextureLevel(uint32_t& pitch, uint32_t& height, uint32_t bits,
                            uint32_t format, bool tiled) {
  uint32_t block_width, block_height;
  GetTextureFormatBlock(format, block_width, block_height);
  uint32_t texels = 32;
  if (!tiled) {
    const uint32_t block_bytes = block_width * block_height * bits / 8;
    if (block_bytes && 256 / block_bytes >= 32) {
      texels = 256 / block_bytes;
    }
  }
  pitch = AlignUp(pitch, block_width * texels);
  height = AlignUp(height, block_height * 32);
  return AlignUp(pitch * bits / 8 * height, 0x1000);
}

}  // namespace

uint32_t xeXGSetTextureHeaderEx(uint32_t width, uint32_t height,
                                uint32_t levels, uint32_t usage,
                                uint32_t d3d_format, int32_t exp_bias,
                                uint32_t flags, uint32_t base_offset,
                                uint32_t mip_offset, uint32_t pitch_bytes,
                                xe::be<uint32_t>* texture,
                                uint32_t* base_size_out,
                                uint32_t* mip_size_out) {
  constexpr uint32_t kNonPacked = 1;
  const uint32_t format = d3d_format & 0x3F;
  const bool tiled = (d3d_format >> 8) & 1;
  const bool packed = !(flags & kNonPacked);
  const uint32_t bits = kTextureFormatBits[format];
  width = std::max(width, 1u);
  height = std::max(height, 1u);
  if (!levels) {
    // Down to 1x1: one more than the largest dimension's highest bit.
    levels = 1;
    for (uint32_t size = std::max(width, height); size > 1; size >>= 1) {
      ++levels;
    }
  }
  // Level 0. A small packed texture is laid out at its power of two size.
  uint32_t pitch = 1u << Log2Ceil(width);
  uint32_t padded_height = 1u << Log2Ceil(height);
  if ((pitch > 16 && padded_height > 16) || !packed) {
    pitch = pitch_bytes ? pitch_bytes * 8 / bits : width;
    padded_height = height;
  }
  uint32_t block_width, block_height;
  GetTextureFormatBlock(format, block_width, block_height);
  uint32_t base_size;
  if (!tiled && !packed && levels == 1) {
    uint32_t level_pitch = pitch;
    uint32_t level_height = padded_height;
    LayoutTextureLevel(level_pitch, level_height, bits, format, tiled);
    pitch = level_pitch;
    base_size = AlignUp(height, block_height) * (pitch * bits / 8);
  } else {
    uint32_t level_height = padded_height;
    LayoutTextureLevel(pitch, level_height, bits, format, tiled);
    base_size = pitch * bits / 8 * level_height;
    if (tiled) {
      base_size = AlignUp(base_size, 0x1000);
    }
  }
  // The other levels, until the packed tail.
  uint32_t mip_size = 0;
  uint32_t log_width = Log2Ceil(width), log_height = Log2Ceil(height);
  for (uint32_t level = 1; level < levels; ++level) {
    if (packed && (log_width <= 4 || log_height <= 4)) {
      break;
    }
    log_width -= log_width ? 1 : 0;
    log_height -= log_height ? 1 : 0;
    uint32_t level_pitch = 1u << log_width;
    uint32_t level_height = 1u << log_height;
    mip_size += LayoutTextureLevel(level_pitch, level_height, bits, format,
                                   tiled);
  }
  // The other levels right after the base start on a page.
  if (mip_size && mip_offset == 0xFFFFFFFF) {
    base_size = AlignUp(base_size, 0x1000);
  }
  if (texture) {
    uint32_t header[13] = {};
    // D3DResource: a texture, with usage flags, one reference and no fences.
    header[0] = 3 | ((usage & 4) ? 0x200000 : 0) |
                ((usage & 0x200) ? 0x400000 : 0);
    header[1] = 1;
    header[5] = 0xFFFF0000;
    header[6] = 0xFFFF0000;
    // The fetch constant: type, component signs, pitch in 32 texels, tiling.
    header[7] = 2 | ((d3d_format >> 9) & 3) << 2 | ((d3d_format >> 11) & 3) << 4 |
                ((d3d_format >> 13) & 3) << 6 | ((d3d_format >> 15) & 3) << 8 |
                ((pitch >> 5) & 0x1FF) << 22 | uint32_t(tiled) << 31;
    // Format, endianness and base address.
    header[8] = format | ((d3d_format >> 6) & 3) << 6 |
                (base_offset & 0xFFFFF000);
    // Size, 2D.
    header[9] = ((width - 1) & 0x1FFF) | ((height - 1) & 0x1FFF) << 13;
    // Number format and swizzle, exponent bias.
    header[10] = ((d3d_format >> 17) & 0x1FFF) |
                 (uint32_t(exp_bias) & 0x3F) << 13;
    // Last level.
    header[11] = ((levels - 1) & 0xF) << 6;
    // 2D, packed mips, and the other levels' address.
    uint32_t mip_address = 0;
    if (mip_size) {
      mip_address =
          mip_offset == 0xFFFFFFFF ? base_size + base_offset : mip_offset;
    }
    header[12] = 1 << 9 | uint32_t(packed) << 11 | (mip_address & 0xFFFFF000);
    for (uint32_t i = 0; i < 13; ++i) {
      texture[i] = header[i];
    }
  }
  if (base_size_out) {
    *base_size_out = base_size;
  }
  if (mip_size_out) {
    *mip_size_out = mip_size;
  }
  return base_size + mip_size;
}

dword_result_t XGSetTextureHeaderEx_entry(
    dword_t width, dword_t height, dword_t levels, dword_t usage,
    dword_t format, dword_t exp_bias, dword_t flags, dword_t base_offset,
    dword_t mip_offset, dword_t pitch, lpvoid_t texture, lpdword_t base_size,
    lpdword_t mip_size) {
  uint32_t base = 0, mip = 0;
  const uint32_t total = xeXGSetTextureHeaderEx(
      width, height, levels, usage, format, int32_t(exp_bias.value()), flags,
      base_offset, mip_offset, pitch,
      texture ? texture.as<xe::be<uint32_t>*>() : nullptr, &base, &mip);
  if (base_size) {
    *base_size = base;
  }
  if (mip_size) {
    *mip_size = mip;
  }
  return total;
}
DECLARE_XAM_EXPORT1(XGSetTextureHeaderEx, kVideo, kSketchy);

// Moves a resource header's addresses by an offset (0x817F7D18): a texture's
// base and, when it has them, its other levels; a buffer's data.
void xeXGOffsetResourceAddress(xe::be<uint32_t>* resource, uint32_t offset) {
  switch (resource[0] & 0xF) {
    case 3: {
      resource[8] = (resource[8] & 0xFFF) |
                    ((resource[8] & 0xFFFFF000) + offset) & 0xFFFFF000;
      const uint32_t levels = ((resource[11] >> 6) & 0xF) + 1;
      if (levels > 1 && (resource[12] & 0xFFFFF000)) {
        resource[12] = (resource[12] & 0xFFF) |
                       ((resource[12] & 0xFFFFF000) + offset) & 0xFFFFF000;
      }
      break;
    }
    case 1:
      resource[6] = (resource[6] & 3) |
                    ((resource[6] & 0xFFFFFFFC) + offset) & 0xFFFFFFFC;
      break;
    case 2:
    case 7:
    case 8:
      resource[6] = resource[6] + offset;
      break;
    case 6:
      resource[8] = resource[8] + offset;
      break;
  }
}

void XGOffsetResourceAddress_entry(lpvoid_t resource, dword_t offset) {
  if (resource) {
    xeXGOffsetResourceAddress(resource.as<xe::be<uint32_t>*>(), offset);
  }
}
DECLARE_XAM_EXPORT1(XGOffsetResourceAddress, kVideo, kSketchy);

}  // namespace xam
}  // namespace kernel
}  // namespace xe

DECLARE_XAM_EMPTY_REGISTER_EXPORTS(Video);

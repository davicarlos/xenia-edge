/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_KERNEL_XAM_XAM_NUI_H_
#define XENIA_KERNEL_XAM_XAM_NUI_H_

#include <cstdint>

#include "xenia/xbox.h"

namespace xe {
namespace kernel {
namespace xam {

// NUI_SKELETON_FRAME as titles publish it (XamApp 0x2B002), big-endian,
// 0xAE0 bytes: the runtime version (5), the floor plane at +0x10 and the
// normal to gravity at +0x20, six skeletons of 0x1C0 bytes from +0x30, flags
// at +0xAB8, and at +0xACC and +0xAD0 the tracking IDs the system asks the
// tracker to follow.
constexpr uint32_t kNuiFrameSize = 0xAE0;
constexpr uint32_t kNuiFrameFloorPlane = 0x10;
constexpr uint32_t kNuiFrameNormalToGravity = 0x20;
constexpr uint32_t kNuiFrameSkeletons = 0x30;
constexpr uint32_t kNuiFrameFlags = 0xAB8;
constexpr uint32_t kNuiFrameImage = 0xAC4;
constexpr uint32_t kNuiFrameTrackedIds = 0xACC;
constexpr uint32_t kNuiSkeletonCount = 6;
constexpr uint32_t kNuiSkeletonSize = 0x1C0;
// Within a skeleton: the tracking state (0 none, 1 position only, 2 tracked),
// tracking ID, enrollment index, user index, position, 20 joints of 16 bytes
// and their tracking states.
constexpr uint32_t kNuiSkeletonState = 0x00;
constexpr uint32_t kNuiSkeletonTrackingId = 0x04;
constexpr uint32_t kNuiSkeletonEnrollment = 0x08;
constexpr uint32_t kNuiSkeletonUserIndex = 0x0C;
constexpr uint32_t kNuiSkeletonPosition = 0x10;
constexpr uint32_t kNuiSkeletonJoints = 0x20;
constexpr uint32_t kNuiSkeletonJointStates = 0x160;
constexpr uint32_t kNuiStateTracked = 2;
constexpr uint32_t kNuiStatePositionOnly = 1;

// Calls a callback a title registered with xam (the tilt engine, the identity
// engine) on the calling guest thread: callback(message, buffer), the buffer
// holding the words given, which come back as the callback left them.
X_HRESULT xeXamNuiCallTitle(uint32_t callback, uint32_t message,
                            uint32_t* words, uint32_t word_count);

// The system HUD's work on a frame a title published (XamNuiHudInterpretFrame,
// 0x816ACC68).
X_HRESULT xeXamNuiHudInterpretFrame(uint8_t* frame);

// The enrollment index the skeleton with the tracking ID had in the last
// frame the HUD saw, or 0xFFFFFFFE.
uint32_t xeXamNuiHudEnrollmentOf(uint32_t tracking_id);

// XamApp's NUI messages (0x2A0xx, 0x2B0xx, 0x2C0xx). False for a message that
// isn't one of them.
bool xeXamNuiDispatchMessage(uint32_t message, uint32_t buffer_ptr,
                             uint32_t buffer_length, X_HRESULT& result);

}  // namespace xam
}  // namespace kernel
}  // namespace xe

#endif  // XENIA_KERNEL_XAM_XAM_NUI_H_

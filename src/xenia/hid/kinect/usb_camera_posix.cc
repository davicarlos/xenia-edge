/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/hid/kinect/usb_audio.h"
#include "xenia/hid/kinect/usb_camera.h"

namespace xe {
namespace hid {
namespace kinect {

// Platforms where third_party doesn't build libusb (macOS, Android) report no
// camera; usb_camera_libusb.cc covers Windows and Linux.
std::unique_ptr<UsbCamera> UsbCamera::Open(bool use_usbdk,
                                           std::function<void()> on_frame) {
  return nullptr;
}

bool LoadAudioFirmware(const std::vector<uint8_t>& image) {
  return false;
}

std::unique_ptr<UsbAudio> UsbAudio::Open() { return nullptr; }

std::string ReadSerialNumber() { return {}; }

}  // namespace kinect
}  // namespace hid
}  // namespace xe

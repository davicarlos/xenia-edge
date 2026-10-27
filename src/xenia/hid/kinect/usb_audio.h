/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_HID_KINECT_USB_AUDIO_H_
#define XENIA_HID_KINECT_USB_AUDIO_H_

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace xe {
namespace hid {
namespace kinect {

// The Kinect audio device once its firmware is running (see
// LoadAudioFirmware): the microphone array and, on model 1473, the tilt
// motor, LED and accelerometer. Motor and LED commands follow the firmware's
// handlers (0x10, 0x8032, 0x803B); the microphone stream follows libfreenect
// (src/audio.c, Apache-2.0).
class UsbAudio {
 public:
  enum class Led : uint32_t {
    kOff = 1,
    kBlinkGreen = 2,
    kGreen = 3,
    kRed = 4,
  };

  struct State {
    // Raw accelerometer counts; about 819 per g.
    int16_t accelerometer[3];
    // The accelerometer through the firmware's two stages of filtering; the
    // second is the gravity xam reports. The raw counts jitter by a couple
    // of degrees at rest, which titles take for the sensor being moved.
    int16_t filtered[3];
    int16_t gravity[3];
    // Degrees from horizontal, as the motor reports it.
    int8_t tilt_angle;
    // Motor status flags from the firmware's status block.
    uint32_t flags;
  };

  // The firmware clamps tilt requests to this range.
  static constexpr int32_t kTiltMinimum = -30;
  static constexpr int32_t kTiltMaximum = 30;

  // Returns nullptr if the audio device isn't running the firmware.
  static std::unique_ptr<UsbAudio> Open();

  virtual ~UsbAudio() = default;

  virtual bool SetTiltAngle(int32_t degrees) = 0;
  virtual bool StopMovement() = 0;
  virtual bool SetLed(Led led) = 0;
  virtual bool ReadState(State& state) = 0;

  // Sends a message to the firmware's audio processing: {'CEMD', 0, id,
  // payload}. Id 0 loads the microphone array calibration, 5 sets the beam
  // angle (radians, as a float) and 6 resets the processing.
  static constexpr uint32_t kAudioMessageMagic = 0x444D4543;
  static constexpr uint32_t kAudioMessageCalibration = 0;
  static constexpr uint32_t kAudioMessageBeamAngle = 5;
  // A calibration message: the header and 0x64008 bytes of calibration.
  static constexpr size_t kAudioCalibrationBytes = 0x64014;
  virtual bool SendAudioMessage(const std::vector<uint8_t>& message) = 0;

  // 16 ms of microphone audio at 16 kHz, as the sensor sends it.
  static constexpr uint32_t kMicrophoneCount = 4;
  static constexpr uint32_t kSamplesPerWindow = 256;
  struct MicrophoneWindow {
    // Sensor timestamp; advances by 3 per window.
    uint16_t window;
    // The sensor's echo-cancelled mix of the array.
    int16_t processed[kSamplesPerWindow];
    // Each microphone's samples.
    int32_t microphones[kMicrophoneCount][kSamplesPerWindow];
  };

  // Streams the microphones until StopMicrophones; `on_window` runs on the
  // USB thread for each complete window.
  virtual bool StartMicrophones(std::function<void()> on_window) = 0;
  virtual void StopMicrophones() = 0;
  // Copies window number `next` of the stream (counting from 0 at
  // StartMicrophones), or the oldest one still queued if it was dropped, and
  // sets `next` past it. Returns false when it hasn't arrived yet.
  virtual bool ReadMicrophoneWindow(uint64_t& next,
                                    MicrophoneWindow& window) = 0;
};

// The Kinect audio device (045e:02ad) starts in a bootloader and, on model
// 1473, also carries the motor, LED and accelerometer, which answer only once
// Microsoft's audio firmware (audios.bin, shipped in Xbox 360 system updates)
// is running on it.
//
// Uploads the firmware image unless it's already running, and waits for the
// device to come back with it. Returns whether the firmware is running
// afterwards.
bool LoadAudioFirmware(const std::vector<uint8_t>& image);

}  // namespace kinect
}  // namespace hid
}  // namespace xe

#endif  // XENIA_HID_KINECT_USB_AUDIO_H_

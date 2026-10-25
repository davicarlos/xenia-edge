/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_KERNEL_KINECT_KINECT_DEVICE_H_
#define XENIA_KERNEL_KINECT_KINECT_DEVICE_H_

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "xenia/hid/kinect/usb_audio.h"
#include "xenia/hid/kinect/usb_camera.h"

namespace xe {
namespace kernel {
namespace kinect {

// The Kinect sensor the emulator gives titles: a connected Kinect v1 when
// allow_kinect_camera is set, a recording with kinect_replay_path, or none.
// It is opened the first time a title uses its Kinect, so that other titles
// leave the device to the host.
//
// On the console the kernel owns the sensor's three USB functions: the camera
// (PsCamDeviceRequest), the audio device, which also drives model 1473's motor
// (McaDeviceRequest), and model 1414's hub with its own motor
// (DetroitDeviceRequest). Here the audio device drives the motor of both.
class KinectDevice {
 public:
  KinectDevice();
  ~KinectDevice();

  bool camera_present() const { return camera_ != nullptr; }
  bool audio_present() const { return audio_ != nullptr; }
  bool is_recording() const { return is_recording_; }

  // The camera, with the calibration it reports and its newest frames.
  hid::kinect::UsbCamera* camera() const { return camera_.get(); }
  // Called from a camera thread whenever a frame arrives.
  void SetFrameListener(std::function<void()> listener);

  // Model 1473's motor, LED and accelerometer, through the audio device.
  bool SetTiltAngle(int32_t degrees);
  bool StopTilt();
  bool ReadMotorState(hid::kinect::UsbAudio::State& state);
  bool SetLed(hid::kinect::UsbAudio::Led led);

  // CEMD messages to the audio firmware (array calibration, beam angle).
  bool SendAudioMessage(const std::vector<uint8_t>& message);
  // Sends the microphone array calibration from kinect_audio_calibration_path
  // (the console's MicArrayCalibrate.data, made by the Kinect Tuner's audio
  // check) as CEMD 0. False without one.
  bool SendAudioCalibration();

  // The microphone array: four raw channels and the processed one, 256
  // samples at 16 kHz per window.
  bool StartMicrophones(std::function<void()> on_window);
  void StopMicrophones();
  bool ReadMicrophoneWindow(uint64_t& next,
                            hid::kinect::UsbAudio::MicrophoneWindow& window);

  // The camera's 12-character serial number, empty without a sensor.
  const std::string& serial_number();

 private:
  void OnFrame();

  std::unique_ptr<hid::kinect::UsbCamera> camera_;
  std::unique_ptr<hid::kinect::UsbAudio> audio_;
  bool is_recording_ = false;
  std::mutex listener_mutex_;
  std::function<void()> frame_listener_;
  // The audio device takes one control transfer at a time.
  std::mutex audio_mutex_;
  std::once_flag serial_once_;
  std::string serial_number_;
};

}  // namespace kinect
}  // namespace kernel
}  // namespace xe

#endif  // XENIA_KERNEL_KINECT_KINECT_DEVICE_H_

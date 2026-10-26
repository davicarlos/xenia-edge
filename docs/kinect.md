# Kinect

Xenia can give titles a Kinect for Xbox 360 (model 1473): its depth and color
cameras, its microphone array and its motor. It can also play back a recording
of the sensor, so that Kinect titles can be worked on without anyone standing
in front of one.

## Using a sensor

1. Plug the Kinect in through its power adapter. The camera shows up as USB
   device `045e:02ae` and the audio device as `045e:02ad`.
2. Linux: the user running Xenia needs write access to the device nodes. A
   udev rule such as

   ```
   SUBSYSTEM=="usb", ATTR{idVendor}=="045e", MODE="0666"
   ```

   does it; replug the sensor after adding it. Xenia detaches the kernel's
   `gspca_kinect` driver itself while it runs.

   Windows: the camera goes through UsbDk (`kinect_use_usbdk`, the default) or
   the WinUSB driver.
3. Set `allow_kinect_camera = true` and `allow_nui_initialization = true`. The
   second tells titles the console has a Kinect set up.
4. For the microphones and the motor, point `kinect_audio_firmware_path` at
   Microsoft's audio firmware: `audios.bin`, or the system update package
   that carries it (`$SystemUpdate/FFFE07DF00000001`). The sensor forgets the
   firmware when it loses power, so Xenia uploads it every time it opens the
   sensor. Without it the cameras still work.

The skeleton tracker of Kinect titles runs on the GPU and reads its output
back, so the defaults of `readback_resolve` and
`snorm16_render_target_full_range` must stay on.

## Options

| Option | Default | What it does |
|---|---|---|
| `allow_kinect_camera` | false | Give titles a connected sensor |
| `allow_nui_initialization` | false | Report a Kinect set up on the console (the `KinectInitialized` retail flag) |
| `kinect_disabled` | false | Report the Kinect switched off in the console's settings |
| `kinect_use_usbdk` | true | Windows: UsbDk rather than WinUSB |
| `kinect_audio_firmware_path` | | The audio firmware to upload |
| `kinect_audio_calibration_path` | | The console's microphone array calibration (`MicArrayCalibrate.data`), sent to the sensor |
| `kinect_microphone_gain` | 8 | Multiplies the microphone samples titles get; they come off USB quieter than titles expect |
| `kinect_chat_microphone` | true | Use the Kinect's microphones for voice chat |
| `kinect_elevation` | 0 | The tilt the Kinect Tuner saved, in degrees |
| `kinect_elevation_calibrated` | false | Whether `kinect_elevation` is set; until then titles look for the floor themselves |
| `kinect_guide_gesture` | true | Open the Guide when a player holds the Kinect Guide gesture |
| `kinect_record_path` | | Record the frames titles get into this directory |
| `kinect_replay_path` | | Play the recording in this directory instead of a sensor |

## Recordings

A recording is a directory with:

- `calibration.bin`: the sensor's registration data and depth constants;
- `depth_NNNNN.bin`: depth frames as the camera sends them (11-bit packed);
- `color_NNNNN.bin`: the color frame taken with each one (Bayer).

Make one with `kinect_record_path` while a title runs, or with
`xenia-kinect-camera-test --record_dir=<dir>`. Play it with
`kinect_replay_path=<dir>`: it loops at 30 frames per second and needs neither
a sensor nor `allow_kinect_camera`. Recordings have no microphones or motor.

`xenia-kinect-camera-test` also checks the hardware without a title: by
default it streams the camera for `seconds` and prints the frame rates it got.
`motor_test`, `audio_wav` and `audio_firmware` exercise the motor, the
microphones and the firmware upload.

## Kinect Tuner

The Kinect Tuner opens when a title calls `XamShowNuiTroubleshooterUI`, or from
the guide button menu while a title runs. It shows the depth and color images
with the title's skeletons over them, whether the player stands where the
tracker can see them, the microphone level, and tilts the sensor. The tilt it
saves goes to `kinect_elevation`.

## How it fits together

On the console the kernel owns the sensor and xam adds the system's Kinect
features. A title links its own NUI runtime, which does the tracking and talks
to both. Xenia follows the same split.

**USB** (`src/xenia/hid/kinect/`) talks to the hardware through libusb:

- `usb_camera_libusb.cc`: the camera's registers and its isochronous depth and
  color streams, assembled into frames. The protocol is the one libfreenect
  documents.
- `usb_audio_libusb.cc`: uploads the firmware to the audio device's
  bootloader, then sends the firmware's commands (motor, LED, accelerometer,
  audio messages) and reads the microphone stream.
- `usb_camera_recording.cc`: recordings, written and played back as a camera.

**Sensor** (`src/xenia/kernel/kinect/`):

- `kinect_device.cc`: the sensor titles get. It opens the hardware or a
  recording the first time a title uses the Kinect, so that other titles leave
  the device alone.
- `kinect_audio.cc`: the microphones as titles read them. It keeps the audio
  render driver's 5.33 ms tic, completes a title's window requests every three
  tics and calls its echo cancellation client.

**Kernel** (`src/xenia/kernel/xboxkrnl/`):

- `xboxkrnl_pscam.cc`: `PsCamDeviceRequest`, the camera as titles' runtimes
  drive it: state, start and stop, frame reads, the registration and depth
  tables, frame timing and camera settings. Each frame read also gets the
  frame's sequence number and time in milliseconds; titles use the time to
  tell frames apart.
- `xboxkrnl_audio.cc`: the render driver tic and the echo cancellation client
  registration.

**xam** (`src/xenia/kernel/xam/`):

- `xam_nui.cc`: the `XamNui*` exports: device state, tilt and elevation, depth
  calibration, camera properties, Kinect ID settings and biometric data.
- `xam_nui_identity.cc`: the XamApp messages titles' runtimes send (`0x2A0xx`,
  `0x2B0xx`, `0x2C0xx`): registering the runtime and its engines, publishing
  each skeleton frame, Kinect ID.
- `xam_nui_hud.cc`: the system HUD's work on each skeleton frame: choosing the
  players to track, engagement, the wave, and the Guide gesture.
- `xam_voice.cc`: the microphone array exports.
- `xam_fitness.cc`: the fitness profile Kinect fitness titles share.
- `ui/kinect_tuner_ui.cc`: the Kinect Tuner.

`src/xenia/kernel/nui.cc` keeps what xam keeps between calls: the runtime a
title registered, the callbacks it gave, Kinect ID enrollments, the tilt and
the latest skeleton frame.

## Troubleshooting

- *"the USB device node is not writable"*: the udev rule above is missing, or
  the sensor wasn't replugged after adding it.
- *"the audio firmware isn't running; no motor or LED"*:
  `kinect_audio_firmware_path` is unset or unreadable.
- The title says no Kinect is connected: check `allow_kinect_camera` and
  `allow_nui_initialization`, and that the log shows the camera streaming.
- Skeletons are never tracked: keep the GPU readback defaults above, stand
  1.2 to 3.5 m from the sensor, and check the Kinect Tuner's depth image.

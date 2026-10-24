# Kinect

Xenia can hand a Kinect for Xbox 360 (v1) connected to the host to titles:
the depth and color cameras, and, once its audio firmware runs, the tilt
motor, LED, accelerometer and microphone array. The sensor is read directly
over USB with the vendored libusb; nothing else needs installing. Windows and
Linux are supported; other platforms report no sensor.

A Kinect needs its own power supply (the Xbox 360 S/E port powers it; on a PC
use the AC adapter with the USB plug).

## Setup

### Linux

The sensor's USB device nodes must be writable by your user. If you get
`Kinect: the USB device node is not writable` in the log, add a udev rule,
for example `/etc/udev/rules.d/60-xenia-kinect.rules`:

```
# Kinect for Xbox 360: camera, audio (microphones, motor on model 1473) and
# motor (model 1414), plus the Kinect for Windows variants.
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", TAG+="uaccess"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", TAG+="uaccess"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", TAG+="uaccess"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c2", TAG+="uaccess"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02be", TAG+="uaccess"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02bf", TAG+="uaccess"
```

then reload the rules and replug the sensor:

```sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

`TAG+="uaccess"` gives the logged-in user access. On a system without
systemd-logind use `MODE="0666"` instead. Distributions' OpenNI2 packages
(`libopenni2-0`) install an equivalent rule.

The kernel's `gspca_kinect` driver binds the camera as a webcam; Xenia
detaches it while it uses the camera and gives it back afterwards.

### Windows

Install [UsbDk](https://github.com/daynix/UsbDk/releases), which lends the
camera to Xenia while it runs. Without UsbDk, bind the camera to WinUSB (for
example with Zadig) and set `kinect_use_usbdk = false`.

### Console settings

Kinect titles check that the console has a sensor set up. Enable
`allow_nui_initialization` so the emulated console reports one.

## Options

All are in the `[HID]` section of the config, except `allow_nui_initialization`, `kinect_disabled` and `kinect_chat_microphone`, which are in `[Console]`.

| Option | Meaning |
|---|---|
| `allow_kinect_camera` | Use the connected sensor. |
| `allow_nui_initialization` | Report a set-up Kinect to titles. |
| `kinect_disabled` | Report the sensor as turned off in the console's settings; titles that turn it off or on change it. |
| `kinect_chat_microphone` | Let titles use the sensor's microphones for voice chat, the console's Kinect setting; on by default. |
| `kinect_audio_firmware_path` | Microsoft's Kinect audio firmware: `audios.bin`, or the Xbox 360 system update package holding it, `$SystemUpdate/FFFE07DF00000001`. Uploaded when the camera opens. |
| `kinect_audio_calibration_path` | The console's microphone array calibration (`MicArrayCalibrate.data`, from the `MicArrayCalibrate.mec` system package a console makes when you run its Kinect audio tuning). Sent to the sensor when it opens. |
| `kinect_microphone_gain` | Multiplier on the microphone samples titles get; 8 by default, which Kinect speech recognition needs. |
| `kinect_auto_tilt` | Tilt the sensor to frame the nearest tracked player from head to feet; on by default. |
| `kinect_elevation` | The angle titles tilt the sensor to when they start. Saved by auto tilt and by the Kinect Tuner's tilt slider. |
| `kinect_guide_gesture` | Open the Kinect Guide on the guide gesture (the left arm held out about 45 degrees down for three seconds) in titles that leave it to the system; on by default. |
| `kinect_mirror` | Mirror the images before titles get them. |
| `kinect_use_usbdk` | Windows only: UsbDk rather than WinUSB. |
| `kinect_record_path` | Record every frame handed to the title into this directory. |
| `kinect_replay_path` | Play a recording instead of a connected sensor. |
| `kinect_dump_depth_path` | Write the latest depth frame to this file every two seconds. |

## Audio firmware: motor, LED, accelerometer, microphones

The sensor's audio device (USB `045e:02ad`) starts in a bootloader. Its
microphones and, on model 1473 (the common Kinect for Xbox 360), its tilt
motor, LED and accelerometer only answer once Microsoft's audio firmware
runs on it. Xenia doesn't ship the firmware: point
`kinect_audio_firmware_path` at your own copy. It's in every Xbox 360 system
update from Microsoft, in `$SystemUpdate/FFFE07DF00000001`; Xenia reads it
from that package directly.

The firmware stays loaded until the sensor loses power. With it:

- titles' tilt requests move the motor, and they read the real angle back;
- the LED is solid green while a title uses the sensor;
- titles that read the microphone array get it (16 kHz, four microphones),
  and their voice commands work: Nike+ Kinect Training answers "Xbox" and the
  commands after it. Titles' echo cancellation gets the sound the title
  plays, so game audio from the speakers is filtered out of the commands.
  At a `kinect_microphone_gain` of 4 the first commands were sometimes
  missed; 8 works from the start.

## Tilt

The console's Kinect setup tells titles at which angle the sensor sees the
player. Xenia reports `kinect_elevation`, and titles tilt to it when they
start. With `kinect_auto_tilt`, Xenia watches the skeletons the title
tracks: when the nearest player's head or feet are out of the picture, it
tilts the sensor to fit them, and once the player fits it saves that angle
to `kinect_elevation`. Stand where you play for a few seconds the first
time.

The sensor also mixes the microphones itself, cancelling echo with a
calibration the console measures in its Kinect audio tuning. Without that
calibration it sends its mix only for a moment after starting, so titles get
the four microphones and do their own; with `kinect_audio_calibration_path`
set Xenia sends it, and titles can point the array's beam
(`XamVoiceSetMicArrayBeamAngle`).

Without the firmware the tilt is simulated: titles see the angle they set,
and the sensor stays still.

## Kinect ID

Kinect titles that recognize faces (Kinect ID) enroll them themselves and
keep them as the profile's biometric data, which the console stores on Xbox
Live. Xenia keeps it with the profile instead, in
`content/<profile XUID>/FFFE07D1/NuiBiometric.bin`; delete the file to forget
the faces enrolled for that profile.

## Players and gestures

Titles that leave players to the system get what the console does: the two
nearest players are tracked, a player standing too close, at the edge of the
picture, turned away or without a hand in sight is reported with position
only, and a player who waves a raised hand engages (becomes the one the title
follows). The guide gesture opens the Kinect Guide. Titles with their own
handling of players, such as Kinect Sports and Nike+ Kinect Training, aren't
affected.

Titles' depth calibration is kept in `kinect_depth_calibration.bin` in the
storage root.

## Kinect Tuner

With a title running, the guide button menu has a Kinect Tuner: live depth
and color previews, the skeletons the title tracks, the sensor's elevation
with a tilt slider, and the microphone level. Titles that open the Kinect
Guide (the guide gesture) open that same menu.

## Testing the sensor without a title

`xenia-kinect-camera-test` streams the camera for a few seconds and reports
the frame rate, to tell a USB problem from one in a title:

```sh
xenia-kinect-camera-test --seconds=5
xenia-kinect-camera-test --audio_firmware=path/to/audios.bin
xenia-kinect-camera-test --motor_test --tilt=10 --led=3 --beam_angle=0.3
xenia-kinect-camera-test --audio_wav=mic --seconds=5
xenia-kinect-camera-test --record_dir=recording --seconds=30
```

`--audio_wav` writes `mic_processed.wav` and `mic_microphones.wav`.
`--record_dir` makes a recording for `kinect_replay_path`.

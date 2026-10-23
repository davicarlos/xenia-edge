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

All are in the `[HID]` section of the config, except `allow_nui_initialization`.

| Option | Meaning |
|---|---|
| `allow_kinect_camera` | Use the connected sensor. |
| `allow_nui_initialization` | Report a set-up Kinect to titles. |
| `kinect_audio_firmware_path` | Microsoft's Kinect audio firmware: `audios.bin`, or the Xbox 360 system update package holding it, `$SystemUpdate/FFFE07DF00000001`. Uploaded when the camera opens. |
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
- titles that read the microphone array get it (16 kHz, four microphones).
  Xenia doesn't pass its own sound output to the sensor's echo
  cancellation, so loud game audio can reach the microphones.

Without the firmware the tilt is simulated: titles see the angle they set,
and the sensor stays still.

## Kinect Tuner

With a title running, the guide button menu has a Kinect Tuner: live depth
and color previews, the skeletons the title tracks, the sensor's elevation
with a tilt slider, and the microphone level.

## Testing the sensor without a title

`xenia-kinect-camera-test` streams the camera for a few seconds and reports
the frame rate, to tell a USB problem from one in a title:

```sh
xenia-kinect-camera-test --seconds=5
xenia-kinect-camera-test --audio_firmware=path/to/audios.bin
xenia-kinect-camera-test --motor_test --tilt=10 --led=3
xenia-kinect-camera-test --audio_wav=mic --seconds=5
xenia-kinect-camera-test --record_dir=recording --seconds=30
```

`--audio_wav` writes `mic_processed.wav` and `mic_microphones.wav`.
`--record_dir` makes a recording for `kinect_replay_path`.

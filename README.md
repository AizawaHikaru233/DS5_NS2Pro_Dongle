# DS5 NS2Pro Dongle

[简体中文](README.CN.md)

Firmware for a Raspberry Pi Pico 2 W based dongle that exposes a DualSense-compatible USB device and supports both original DualSense input and NS2Pro input.

The desktop manager is published separately at [AizawaHikaru233/DS5-NS2Pro-Dongle-Manager](https://github.com/AizawaHikaru233/DS5-NS2Pro-Dongle-Manager).

## Install

1. Download the latest `ds5_ns2pro_dongle_v*.uf2` from [Releases](https://github.com/AizawaHikaru233/DS5_NS2Pro_Dongle/releases).
2. Hold `BOOTSEL` on the Pico 2 W and plug it into USB.
3. Copy the UF2 file to the mounted `RPI-RP2` drive.
4. After reboot, open the desktop manager and pair a controller.

## Features

- DualSense-compatible USB device identity for games and Windows.
- Original DS5 Bluetooth path kept as close as possible to upstream DS5Dongle.
- NS2Pro wired input through the desktop bridge.
- NS2Pro Bluetooth input directly on the Pico BLE path.
- NS2Pro stick, gyro, rumble, haptics and pairing behavior handled by firmware.
- NS2Pro settings are kept independent from DS5 settings where possible.
- Firmware manager HID protocol for configuration, status, pairing and calibration.

## Difference From Upstream DS5Dongle

Upstream [awalol/DS5Dongle](https://github.com/awalol/DS5Dongle) focuses on bridging a real DualSense over Bluetooth and exposing it as a wired DualSense-compatible USB device.

This fork keeps that path and adds an NS2Pro path:

- NS2Pro reports are parsed and translated into DualSense input reports.
- NS2Pro gyro is exposed through the DualSense report layer with NS2Pro-specific calibration.
- DS5 haptics and normal rumble are converted to NS2Pro rumble output.
- NS2Pro wired and BLE input share one logical configuration profile unless a feature explicitly needs separation.
- DS5 and NS2Pro pairing services coexist, but the firmware focuses on one connected controller at a time.

## Build Requirements

- CMake
- Ninja
- Python 3
- Git
- ARM GNU Toolchain `14.2.rel1`
- Raspberry Pi Pico SDK `2.2.0`
- TinyUSB `0.20.0`

The Windows build script can install or download these automatically.

## Build On Windows

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\build-windows.ps1 -Variant standard
```

Output:

```text
tools\ds5_ns2pro_dongle_v1.0.0.uf2
%USERPROFILE%\Desktop\ds5_ns2pro_dongle_v1.0.0.uf2
```

## Manual Build

```sh
git submodule update --init --recursive
cmake -S . -B build/standard -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DPICO_SDK_PATH=/path/to/pico-sdk \
  -DENABLE_NS2PRO_SERIAL_BRIDGE=ON
cmake --build build/standard --target ds5_ns2pro_dongle
```

## One-Click Release

GitHub Actions includes `Release firmware`.

1. Open Actions.
2. Select `Release firmware`.
3. Run workflow.
4. Enter a version such as `1.0.0`.

The workflow creates or updates release tag `v1.0.0`, builds `ds5_ns2pro_dongle_v1.0.0.uf2`, and uploads it to the GitHub Release.

## References

- Original firmware base: [awalol/DS5Dongle](https://github.com/awalol/DS5Dongle)
- Desktop manager base: [GooGuJiang/ds5dongle-manager](https://github.com/GooGuJiang/ds5dongle-manager)
- NS2Pro BLE/pairing reference: [LeonChrome/y700-switch2-pro-bridge](https://github.com/LeonChrome/y700-switch2-pro-bridge)
- DualSense report reference: [controllers.fandom.com/wiki/Sony_DualSense](https://controllers.fandom.com/wiki/Sony_DualSense)
- DualSense haptics POC: [egormanga/SAxense](https://github.com/egormanga/SAxense)
- DualSense speaker report reference: [Paliverse/DualSenseX](https://github.com/Paliverse/DualSenseX)
- Pico DualSense inspiration: [rafaelvaloto/Pico_W-Dualsense](https://github.com/rafaelvaloto/Pico_W-Dualsense)

## License

MIT. Upstream attribution is kept for derived code.

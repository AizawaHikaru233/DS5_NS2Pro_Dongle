# DS5 NS2Pro Dongle

[English](README.md)

这是基于 Raspberry Pi Pico 2 W 的固件。Pico 会枚举成 DualSense 兼容 USB 设备，并同时支持原始 DualSense 输入和 NS2Pro 输入。

桌面管理器单独发布在 [AizawaHikaru233/DS5-NS2Pro-Dongle-Manager](https://github.com/AizawaHikaru233/DS5-NS2Pro-Dongle-Manager)。

## 安装

1. 从 [Releases](https://github.com/AizawaHikaru233/DS5_NS2Pro_Dongle/releases) 下载最新的 `ds5_ns2pro_dongle_v*.uf2`。
2. 按住 Pico 2 W 的 `BOOTSEL` 并插入 USB。
3. 把 UF2 文件复制到弹出的 `RPI-RP2` 磁盘。
4. Pico 自动重启后，打开桌面管理器并配对手柄。

## 功能

- 对外枚举为 DualSense 兼容 USB 设备。
- 尽量保持原始 DS5 蓝牙路径接近上游 DS5Dongle。
- 支持通过桌面桥接接入 NS2Pro 有线输入。
- 支持 Pico 侧直接连接 NS2Pro 蓝牙输入。
- NS2Pro 摇杆、陀螺仪、震动、触觉反馈和配对逻辑由固件处理。
- 在可行范围内让 NS2Pro 设置独立于 DS5 设置。
- 提供管理 HID 协议，用于配置、状态、配对和校准。

## 跟原版 DS5Dongle 的区别

原版 [awalol/DS5Dongle](https://github.com/awalol/DS5Dongle) 主要负责把真实 DualSense 通过 Pico 蓝牙桥接为有线 DualSense 兼容 USB 设备。

这个分支保留原始 DS5 路径，并额外加入 NS2Pro 路径：

- 解析 NS2Pro 输入报告并转译为 DualSense 输入报告。
- 通过 DualSense 报文层暴露 NS2Pro 陀螺仪，并使用 NS2Pro 专用校准。
- 把 DS5 触觉反馈和普通震动转换为 NS2Pro 可识别的震动输出。
- NS2Pro 有线和蓝牙输入默认共用一套逻辑配置。
- DS5 和 NS2Pro 的配对服务可以共存，但固件同一时间专注于一个已连接手柄。

## 构建依赖

- CMake
- Ninja
- Python 3
- Git
- ARM GNU Toolchain `14.2.rel1`
- Raspberry Pi Pico SDK `2.2.0`
- TinyUSB `0.20.0`

Windows 构建脚本会自动安装或下载这些依赖。

## Windows 构建

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\build-windows.ps1 -Variant standard
```

输出：

```text
tools\ds5_ns2pro_dongle_v1.0.0.uf2
%USERPROFILE%\Desktop\ds5_ns2pro_dongle_v1.0.0.uf2
```

## 手动构建

```sh
git submodule update --init --recursive
cmake -S . -B build/standard -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DPICO_SDK_PATH=/path/to/pico-sdk \
  -DENABLE_NS2PRO_SERIAL_BRIDGE=ON
cmake --build build/standard --target ds5_ns2pro_dongle
```

## 一键发布

GitHub Actions 里已经提供 `Release firmware`。

1. 打开 Actions。
2. 选择 `Release firmware`。
3. 点击运行 workflow。
4. 输入版本号，例如 `1.0.0`。

workflow 会创建或更新 `v1.0.0` Release，构建 `ds5_ns2pro_dongle_v1.0.0.uf2`，并上传到 GitHub Release。

## 参考来源

- 原始固件来源：[awalol/DS5Dongle](https://github.com/awalol/DS5Dongle)
- 桌面管理器来源：[GooGuJiang/ds5dongle-manager](https://github.com/GooGuJiang/ds5dongle-manager)
- NS2Pro 蓝牙/配对参考：[LeonChrome/y700-switch2-pro-bridge](https://github.com/LeonChrome/y700-switch2-pro-bridge)
- DualSense 报文参考：[controllers.fandom.com/wiki/Sony_DualSense](https://controllers.fandom.com/wiki/Sony_DualSense)
- DualSense 触觉反馈 POC：[egormanga/SAxense](https://github.com/egormanga/SAxense)
- DualSense 扬声器报文参考：[Paliverse/DualSenseX](https://github.com/Paliverse/DualSenseX)
- Pico DualSense 灵感来源：[rafaelvaloto/Pico_W-Dualsense](https://github.com/rafaelvaloto/Pico_W-Dualsense)

## 许可证

MIT。来自上游项目的代码保留来源说明。

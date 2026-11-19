# Livox MID-70 原生桥接

`LivoxBridge.dll` 将官方旧版 `Livox-SDK` 的 C++ 回调转换为稳定的 C ABI，供 .NET Framework 4.0 x64 通过 P/Invoke 调用。仓库不包含厂商 SDK、编译产物或固件。

## 环境

- Windows x64
- Visual Studio 2019/2022 C++ 工具链与 CMake
- 固定使用官方 [Livox-SDK v2.3.1](https://github.com/Livox-SDK/Livox-SDK/tree/v2.3.1)，MID-70 不使用 Livox-SDK2
- MID-70 固件最低版本 `10.04.0`；程序连接后会读取并校验实际固件版本

## 构建

```powershell
.\prepare-livox-sdk.ps1 -Build
```

脚本下载锁定版本、构建静态 SDK 和桥接 DLL，并把 `LivoxBridge.dll` 部署到 `capture\app\bin`。真机联调前应保证 MID-70 与 ADT7880 使用独立网卡或不同网段，并在防火墙中允许 Livox UDP 数据。

当前 C ABI 为 `create/start/poll/get_device_info/stop/destroy`。不填写广播码时桥接自动锁定首台发现的 MID-70；`get_device_info` 返回实际广播码和固件版本。桥接不再把 SDK 点提前转换成浮点米，而是把 `LivoxRawPoint` 的 `XYZ int32 mm + reflectivity` 原样交给 C# 落盘；预览和 PLY 导出时才换算成米。点云回调同时记录 Windows QPC 和 Livox 设备时间戳，内部保留最多 8 个批次，消费者落后时覆盖最旧批次并累计丢批计数，避免阻塞 SDK 回调。

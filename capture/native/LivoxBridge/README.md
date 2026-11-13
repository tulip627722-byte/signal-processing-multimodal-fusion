# Livox MID-70 原生桥接

`LivoxBridge.dll` 将官方旧版 `Livox-SDK` 的 C++ 回调转换为稳定的 C ABI，供 .NET Framework 4.0 x64 通过 P/Invoke 调用。仓库不包含厂商 SDK、编译产物或固件。

## 环境

- Windows x64
- Visual Studio 2019 C++ 工具链与 CMake
- 官方 [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK)，MID-70 不使用 Livox-SDK2

## 构建

```powershell
cmake -S . -B build -A x64 -DLIVOX_SDK_ROOT=C:\src\Livox-SDK
cmake --build build --config Release
```

将生成的 `LivoxBridge.dll` 和 Livox-SDK 所需运行库放到 `Acquire.exe` 同目录。真机联调前应保证 MID-70 与 ADT7880 使用独立网卡或不同网段，并在防火墙中允许 Livox UDP 数据。

当前 C ABI 为 `create/start/poll/stop/destroy`。桥接内部保留最多 8 个点云批次，消费者落后时覆盖最旧批次，避免阻塞 SDK 回调。

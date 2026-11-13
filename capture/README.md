# 多模态同步采集（Radar + RGB + Depth + LiDAR）

目标：基于已有可靠性采集程序，一键同步记录雷达 ADC、彩色图、深度图和 Livox 点云，
四路使用统一时间基准，并在单窗口 `2×2` 看板中实时显示。

```
D:\7880\Realtime\采集\
├─ app\            采集 GUI（C# WinForms，独立于 2.0\gui，不改动原 GUI）
│   ├─ src\
│   ├─ build.ps1
│   └─ bin\
├─ takes\          每次采集一个子目录
├─ docs\           协议、同步方案、踩坑记录
└─ README.md
```

## 一次采集的产物

```
takes\20261026_173000_室内2米\
├─ radar_adc.mat          雷达 ADC，MAT v5，变量 ADCData（分析脚本可直接读）
├─ radar_frames.csv       每帧一行：帧号、首包时间、末包时间、丢包数
├─ color\000001.png       彩色图，按帧序号
├─ depth\000001.png       深度图，16-bit PNG（原始深度单位）
├─ lidar\000001.bin       LIVOXBIN_V1 点云批次（XYZ + 反射率）
├─ depth_scale.txt        深度单位（例如 0.001 m/单位）
├─ manifest.csv           stream,frame,t_rel_ms,t_utc,file   ← 四路对齐靠它
└─ meta.json              延时、时长、设备型号、雷达帧率、实际帧数、丢包统计
```

## 交互

- 延迟启动：0~60 s，倒计时显示在界面上
- 采集时长：1~600 s，到点自动停并落盘
- 手动停止随时可用
- 四窗口：RGB、深度伪彩、毫米波距离谱和 LiDAR 点云
- 点云：鼠标拖动旋转，滚轮缩放
- 状态栏：四路实时帧率、已录帧/点数、丢包与恢复次数、剩余时间

预览使用最新帧覆盖缓冲：界面绘制落后时只覆盖旧预览帧，不阻塞设备读取和原始数据写盘。

## Livox MID-70

- 无硬件时选择“模拟点云”，可完整验证界面、时间轴、二进制落盘和回放。
- 真机使用官方旧版 `Livox-SDK`，不是面向 Mid-360/HAP 的 Livox-SDK2。
- 原生桥接源码与构建方法见 `native/LivoxBridge/README.md`；厂商 SDK 和 DLL 不进入仓库。
- `ILidarSource` 同时支持合成源、`LIVOXBIN_V1` 回放源与真机 P/Invoke 数据源。
- 真机当前以主机收包时刻写入统一 `t_rel_ms`；PTP/PPS 仍需设备到位后验证。

## 当前雷达侧事实

- 固件：10 FPS 乒乓 + UDP 直发，每帧 2048 包 × 2048 字节（4 MB）
- 包结构、payload 解包规则见 `2.0\gui\src\RadarTransport.cs`
- ADCData 列含义：`{时间戳, packetId, type, payload}`
- 10 fps 时网络占用约 320 Mbps；20 fps 约 640 Mbps（千兆可承受）；30 fps 约 960 Mbps（超过千兆实测上限）

## 硬约束

1. **不改动** `2.0\gui` 和 `1.1\gui` 的任何文件，采集 GUI 完全独立
2. 编译方式沿用 `2.0\gui\build.ps1`：`C:\Windows\Microsoft.NET\Framework64\v4.0.30319\csc.exe`，`/platform:x64`
3. 无硬件时也要能跑 `--self-test`（生成一份假的 take 目录，验证文件结构和 manifest 格式）
4. 四路主机时间戳必须来自同一个单调时钟，允许事后换算，不允许各自为政

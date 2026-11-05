# 采集（Radar + Color + Depth 同步录像）

目标：一键采一批"有说服力"的数据——雷达 ADC、彩色图、深度图三路同时记录，带统一时间戳，
支持**延迟启动**和**定时自动停止**。

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
├─ radar_adc.mat          雷达 ADC，v7.3，变量 ADCData（与现有一致，分析脚本可直接读）
├─ radar_frames.csv       每帧一行：帧号、首包时间、末包时间、丢包数
├─ color\000001.png       彩色图，按帧序号
├─ depth\000001.png       深度图，16-bit PNG（原始深度单位）
├─ depth_scale.txt        深度单位（例如 0.001 m/单位）
├─ manifest.csv           stream,frame,t_rel_ms,t_utc,file   ← 三路对齐靠它
└─ meta.json              延时、时长、设备型号、雷达帧率、实际帧数、丢包统计
```

## 交互

- 延迟启动：0~60 s，倒计时显示在界面上
- 采集时长：1~600 s，到点自动停并落盘
- 手动停止随时可用
- 状态栏：三路各自的实时帧率、已录帧数、丢包数、剩余时间

## 当前雷达侧事实

- 固件：10 FPS 乒乓 + UDP 直发，每帧 2048 包 × 2048 字节（4 MB）
- 包结构、payload 解包规则见 `2.0\gui\src\RadarTransport.cs`
- ADCData 列含义：`{时间戳, packetId, type, payload}`
- 10 fps 时网络占用约 320 Mbps；20 fps 约 640 Mbps（千兆可承受）；30 fps 约 960 Mbps（超过千兆实测上限）

## 硬约束

1. **不改动** `2.0\gui` 和 `1.1\gui` 的任何文件，采集 GUI 完全独立
2. 编译方式沿用 `2.0\gui\build.ps1`：`C:\Windows\Microsoft.NET\Framework64\v4.0.30319\csc.exe`，`/platform:x64`
3. 无硬件时也要能跑 `--self-test`（生成一份假的 take 目录，验证文件结构和 manifest 格式）
4. 三路时间戳必须来自同一个单调时钟，允许事后换算，不允许各自为政

# 多模态同步采集（Radar + RGB + Depth + LiDAR）

目标：基于已有可靠性采集程序，一键同步记录雷达 ADC、彩色图、深度图和 Livox 点云，
四路使用统一时间基准，并在单窗口 `2×2` 看板中实时显示。

三种传感器彼此独立：RGB-D 相机、ADT7880 毫米波雷达或 MID-70 任意一路未启用、启动失败或没有数据时，程序会给出警告并继续记录其它可用传感器。`meta.json` 中的 `requested`、`available`、`startupError` 和 `warnings` 可用于判断每一路的实际采集状态；只有三路真实数据均有效时 `validForCalibration` 才为 `true`。

毫米波提供两种互斥模式。默认“AC880 实时点云”严格沿用 `RealtimeRadarKit_Research_Lite` 的接线与流程：雷达网口接 AC880 PL、AC880 PS 接电脑，电脑网卡为 `192.168.221.100/24`，点云由 `192.168.221.2:5001` 回传；采集开始时发送 `Stop D001`，等待 750ms 后发送 `Start D000`，结束发送 `Stop D001`。切换到“ADC 原始直采”后才使用雷达直连电脑的 `192.168.220.x:8080` 并显示 ADC 距离谱。两种拓扑不能同时使用。Livox MID-70 预览严格按 `1000ms` 聚合刷新，原始 `.bin` 批次仍按采集线程无损写入。

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
├─ lidar\000001.bin       LIVOXBIN_V3 原始笛卡尔记录（int32 mm + 反射率 + 包级时间）
├─ lidar_frames.csv       点云批次时间范围、设备时间戳类型、包数和点数
├─ aligned_frames.csv     以雷达帧为主帧的相机近邻与 LiDAR 1000 ms 窗口
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
- `ILidarSource` 同时支持合成源、兼容 V1/V2/V3 的回放源与真机 P/Invoke 数据源。
- 原始 Livox 点坐标固定为 `X 前 / Y 左 / Z 上`；预览只做显示换轴，不改落盘坐标。
- 真机在原生 SDK 回调内记录 Windows QPC，并换算到统一 `t_rel_ms`；同时保存 Livox 设备时间戳及其类型，PTP/PPS 仍需设备到位后验证。
- V3 无损保存 Livox-SDK 的 `LivoxRawPoint`：`XYZ int32 mm + reflectivity uint8`，并记录每个 SDK 包的点范围和时间；这是 SDK 解出的原始笛卡尔记录，不是网卡层 UDP 抓包。

采集完成后生成 PLY：

```powershell
cd E:\leida\radar-camera-alignment\capture\app
.\build.ps1
.\export-lidar.ps1 -TakeDirectory "完整take目录" -Merged
```

输出包含逐批 PLY、`index.csv` 和可选 `merged.ply`。合并点云不含运动补偿，仅用于静态标定或静态场景检查。

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
## ADC 三传感器帧对齐

真实采集时选择 `ADT7880` 的 `ADC 直连`、`Livox MID-70 真机` 和带深度流的 RGB-D 相机后，程序会先启动相机与 LiDAR，等待 LiDAR 累积至少 1000 ms，再启动 ADC 雷达。雷达帧是同步主帧；每个雷达帧会在 `aligned_frames.csv` 记录最近的深度/彩色帧，以及该雷达中心时刻往前 1000 ms 内相交的全部 `lidar/*.bin` 批次。原始 `radar_adc.mat`、PNG、LiDAR BIN 和现有 CSV 不会被替换。

若相机、MID-70 真机或 1000 ms LiDAR 预缓存未就绪，程序会在雷达启动前拒绝本次采集。`meta.json` 的 `alignment` 对象记录窗口、阈值和有效帧数量。

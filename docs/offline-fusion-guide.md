# 三模态点云融合操作指南

本指南用于完成 ADT7880 毫米波雷达、Astra Depth 和 Livox MID-70 的离线采集、处理、标定、融合和 MATLAB 可视化。

统一坐标系为 ADT7880 雷达坐标：X=右，Y=前，Z=上，单位为米。

## 1. 编译并启动采集程序

在 PowerShell 中运行：

~~~powershell
cd E:\leida\radar-camera-alignment\capture\app
.\build.ps1
.\bin\Acquire.exe
~~~

在采集界面中选择真实的 ADT7880、Orbbec Astra 和 Livox MID-70；点击“安装位置…”保存当前安装关系，然后点击“开始采集”。

有效 take 至少应包含 radar_adc.mat、meta.json、深度 PNG、lidar\*.bin 和 manifest.csv / aligned_frames.csv。

## 2. 采集后验收原始数据

~~~matlab
cd('E:\leida\radar-camera-alignment\matlab')
addpath('lidar')
takeDir = 'E:\leida\radar-camera-alignment\capture\takes\你的take目录';
report = inspect_lidar_radar_take(takeDir);
~~~

检查 LiDAR 文件数量、LIVOXBIN 版本、毫米波帧数、丢包、最近帧时间差和 meta.json 中三路设备状态。三路不完整时先重新采集。

## 3. 生成雷达 / Depth 处理结果

~~~matlab
cd('E:\leida\radar-camera-alignment\matlab')
addpath(genpath(pwd))
[resFile, summary] = compute_take(takeDir);
~~~

批量计算：

~~~matlab
compute_all([], struct('root', 'E:\leida\radar-camera-alignment\capture\takes'));
~~~

成功标志是生成 results\takes\<take名>_res.mat。修改参数或重新采集后强制重算：

~~~matlab
compute_all([], struct('root', 'E:\leida\radar-camera-alignment\capture\takes', 'force', true));
~~~

## 4. 安装位置改变时重新标定

标定期间固定三台设备。建议至少采集 5 组静止棋盘或高反射板数据，覆盖左、右、上、下、近、远。

~~~matlab
cd('E:\leida\radar-camera-alignment\matlab')
addpath('lidar')
config = lidar_radar_calibration_config();
config.takeNames = ["take_01"; "take_02"; "take_03"; "take_04"; "take_05"];
report = run_lidar_radar_auto_calibration(config);
~~~

只使用输出中 applicable=true 的候选外参。候选文件写入 results\lidar_radar_calibration_时间戳\，不会覆盖正式 JSON。

## 5. 导出 MID-70 PLY（可选）

~~~powershell
cd E:\leida\radar-camera-alignment\capture\app
.\export-lidar.ps1 -TakeDirectory "完整take目录" -Merged
~~~

输出位于 <take目录>\lidar_cloud\。

## 6. 生成离线三模态融合文件

~~~matlab
cd('E:\leida\radar-camera-alignment\matlab')
addpath(genpath(pwd))
fusionReport = run_offline_fusion(takeDir);
~~~

主要输出是 *_offline_fusion.mat、*_offline_fusion.ply 和质量报告 JSON。重点查看：

~~~matlab
fusionReport.quality.status
fusionReport.quality.radarBoardStatus
fusionReport.quality.depthLidarStatus
~~~

## 7. 用 view_take 查看最终结果

~~~matlab
view_take(resFile)
~~~

也可以直接传入 take 目录：

~~~matlab
view_take(takeDir)
~~~

指定 LiDAR 时间窗和候选外参：

~~~matlab
view_take(resFile, 'lidarWindowMs', 1000, 'lidarExtrinsicsFile', '候选外参JSON完整路径')
~~~

最终四窗口应能区分 Depth、MID-70 原生点云、ADT7880 原始毫米波点云和统一到 ADT7880 坐标后的三路融合点云。原始毫米波点不会被强制移动到板面。

## 8. 常见问题

### load_offline_fusion 无法识别

~~~matlab
addpath(genpath('E:\leida\radar-camera-alignment\matlab'))
which view_take -all
which run_offline_fusion -all
~~~

### 找不到 _res.mat

先运行第 3 步的 compute_take 或 compute_all，再运行 view_take。

### 融合文件找不到

先运行第 6 步的 run_offline_fusion(takeDir)，确认 *_offline_fusion.mat 已生成。

### 雷达点不在板面

不要直接平移原始雷达点。检查 radarBoardStatus、距离/角度偏差、安装快照和多姿态标定数据；必要时补采后重新标定。

## 9. 建议保存的交付文件

- 原始 take 目录
- <take名>_res.mat
- <take名>_offline_fusion.mat
- _offline_fusion_report.json
- applicable=true 的候选外参 JSON
- view_take 最终截图

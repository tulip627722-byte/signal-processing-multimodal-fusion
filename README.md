# Radar-Camera Alignment

ADT7880 毫米波雷达与 Orbbec Astra 深度相机的同步采集、MATLAB 点云处理、坐标投影和外参验证工程。

本仓库只保存可审查、可复现的源码与说明。原始 ADC、彩色图、深度图、点云、MATLAB 结果和厂商固件工程不进入 Git。

## 仓库状态

- `main`：稳定基线。使用正式相机到雷达外参，雷达距离/方位/俯仰默认零补偿。
- `experiment/radar-angle-bias-20261105`：2026-11-05 室外 3–7 m 棋盘实验补偿。包含候选参数、验证脚本和分析报告。
- 尚未创建 GitHub 远程仓库，也没有推送。

实验分支的候选补偿为：

```text
range_corrected = range_measured - 0.149477 m
az_corrected    = az_measured    - 3.2096 deg
el_corrected    = el_measured    - 1.9813 deg
```

该候选留一验证五组中四组通过，3 m 组为 `0.16 m`，略高于 `0.15 m` 门限，因此不能直接合并到 `main`。

## 目录结构

```text
capture/
  app/src/              C# WinForms 同步采集源码
  docs/                 采集、同步和硬件注意事项
matlab/
  algorithm/            距离/多普勒/CFAR/近场 Bartlett/聚类
  tools/                标定、诊断和验证工具
  verify_range_fft/     距离 FFT 独立核验工具
  compute_take.m        单组计算入口
  compute_all.m         批量计算入口
  run_compute_interactive.m  图形化文件选择入口
docs/
  extrinsics/           正式外参说明与验证图
  BRANCHES.md           分支边界与合并规则
  NOTES.md              使用限制和注意事项
```

## MATLAB 快速使用

需要 MATLAB R2025a 或兼容版本。将采集数据保存在仓库外部，然后运行：

```matlab
cd('E:\leida\radar-camera-alignment\matlab')
run_compute_interactive
```

交互入口支持：

- 单组：选择一次采集目录里的 `radar_adc.mat`。
- 批量：选择包含多组采集目录的父目录。
- `main` 默认零补偿，也可以手动输入试算参数。
- 结果写到仓库内的 `results/takes`，该目录已被 Git 忽略。

命令行方式：

```matlab
takeDir = 'E:\your-data\20261105_155359_outdoor-7m';
[resFile, summary] = compute_take(takeDir);

compute_all([], struct('root', 'E:\your-data'));
```

## 采集程序

C# 采集程序源码位于 `capture/app/src`，目标为 .NET Framework 4.0 x64：

```powershell
cd E:\leida\radar-camera-alignment\capture\app
.\build.ps1
```

编译结果位于 `capture/app/bin`，该目录不会提交。设备要求、输出格式和自检方法见 [capture/README.md](capture/README.md)。

## 正式坐标系与外参

相机 OpenNI 坐标：`X 右 / Y 下 / Z 前`。  
雷达坐标：`X 右 / Y 前 / Z 上`。

稳定主分支使用：

```text
R = [1  0  0
     0  0  1
     0 -1  0]
t = [-0.0093, -0.0063, 0.1000] m

p_radar = R * p_camera + t
```

完整来源和限制见 [雷达相机外参标定结果与坐标投影说明](docs/extrinsics/雷达相机外参标定结果与坐标投影说明.md)。

## 数据要求

一次有效采集目录至少需要：

```text
radar_adc.mat
radar_frames.csv
manifest.csv
meta.json
color/*.png
depth/*.png
depth_scale.txt
```

原始数据体积很大，必须放在 Git 仓库外。不要用 Git LFS 代替数据管理，除非后续明确建立独立数据仓库。

## 验收原则

- 至少五组有效目标数据。
- 独立留出验证的三维板心误差不超过 `0.15 m`。
- 参数不能贴搜索边界。
- 机械外参与雷达算法偏差必须分开保存。
- Astra Pro 室外深度只作弱参考，雷达距离验证优先使用卷尺距离和原始距离谱。

详细分支和合并规则见 [docs/BRANCHES.md](docs/BRANCHES.md)，运行注意事项见 [docs/NOTES.md](docs/NOTES.md)。


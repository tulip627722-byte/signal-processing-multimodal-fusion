# 多模态感知信息实时融合系统

项目成员：余佳欣、苟怡菲、闫傲雪

面向室内验证、普通室外 3–7 m 与后续矿山感知场景，构建 **ADT7880 毫米波雷达 + 视觉相机 + Livox MID-70 激光雷达** 的多源同步采集、标定与融合平台。

---

## 一、项目目标

### 考核指标

| 项 | 目标 |
|---|---|
| 项目目标 | 多模态感知信息**实时融合**，交付一套实时多模态融合系统 |
| 硬件平台 | ADT7880 毫米波雷达 + 视觉相机 + Livox MID-70 激光雷达 |
| 丢帧率 | `< 1%` |
| P95 预览时延 | `≤ 2` 个帧周期 |
| 稳定性 | 连续 `30 min` 无异常断流 |
| 几何验证 | 独立数据三维板心误差 `≤ 0.15 m` |

### 技术路线

贯穿「理论 → 仿真 → 实现 → 工程化」：

1. **理论与算法**：FMCW 雷达原理、信号处理与融合
2. **仿真与算法验证**：MATLAB 处理链（距离–多普勒 / CFAR / 近场 Bartlett / 聚类）
3. **实时采集实现**：C# WinForms 四路同步采集上位机
4. **工程化训练**：时空同步与校准、无硬件自测、压力测试、代码工程化

### 场景与传感器路线

- **室内 / 短距对照**：Astra Pro 深度相机
- **普通室外 3–7 m**：优先验证被动双目 Stereolabs ZED 2i
- **车载 / 露天矿 / 长期工业部署**：ZED X 级工业双目 + LiDAR 融合

最终型号需在真实多云、逆光、粉尘、振动条件下完成 3 / 5 / 7 m A/B 测试。

---

## 二、目前进展

### 1. 实现毫米波雷达与相机的坐标对齐

已完成 ADT7880 毫米波雷达与相机的外参标定与坐标投影验证：正式外参留出板心误差由 `0.473 m` 降至 `0.150 m`，五组实验平均三维误差由 `0.359 m` 降至 `0.102 m`，留一验证 `4 / 5` 组达到 `0.15 m` 门限。

![雷达-相机坐标对齐前后](docs/extrinsics/alignment_before_after.png)

![目标中心补偿前后对比](docs/experiments/2026-11-05/center_alignment_before_after.png)

![留一验证结果](docs/extrinsics/holdout_validation.png)

### 2. 改良采集程序，实现可视化 GUI

在原采集程序基础上完成可靠性改良与四窗口可视化上位机，单窗口 `2×2` 同步展示 RGB、深度伪彩、ADT7880 距离谱与点云，预览与原始数据落盘解耦，为后期**实时点云直接多模态重合**做铺垫。

![四窗口可视化上位机](docs/images/multiview-dashboard.png)

### 测试状态

- **相机**：已测试成功
- **毫米波雷达、激光雷达**：尚未进行真机测试

---

## 三、仓库结构

```text
signal-processing-multimodal-fusion/
├─ capture/                C# WinForms 同步采集程序（相机 / 雷达 / LiDAR / 深度 / 预览）
│  ├─ app/                 采集上位机源码与 build.ps1
│  ├─ docs/                采集说明、进度、固件改动、丢帧排查
│  └─ native/LivoxBridge/  MID-70 原生桥接（C++/CMake）
├─ matlab/                 信号处理链与外参标定 / 验证工具
│  ├─ algorithm/           距离 FFT / CFAR / 近场 Bartlett / 聚类
│  ├─ tools/               外参标定、候选求解、残差与板心验证
│  └─ verify_range_fft/    距离 FFT 验证 GUI
├─ docs/
│  ├─ extrinsics/          雷达–相机外参标定结果与坐标投影说明
│  ├─ experiments/2026-11-05/  室外 3–7 m 棋盘实验报告
│  └─ images/              文档配图
└─ README.md
```

---

## 四、快速运行

采集程序（.NET Framework 4.0 / WinForms / x64，系统 `csc.exe` 构建）：

```powershell
cd E:\leida\radar-camera-alignment\capture\app
.\build.ps1
.\bin\Acquire.exe
```

无硬件四路全链路测试：

```powershell
.\bin\Acquire.exe --self-test --delay 0 --duration 3 --fps 2
```

MATLAB 单组或批量处理：

```matlab
cd('E:\leida\radar-camera-alignment\matlab')
run_compute_interactive
```

更多说明见 [capture/README.md](capture/README.md)、[docs/BRANCHES.md](docs/BRANCHES.md)。

---

## 五、后续计划

1. 完成毫米波雷达、激光雷达真机测试。
2. 接入 MID-70 真机，验证 PTPv2 / PPS 时间同步与断流恢复。
3. 在真机数据基础上开展毫米波、激光点云与 RGB / 深度的坐标统一与**实时融合叠加**。
4. 实测收敛丢帧率 `< 1%`、P95 时延 `≤ 2` 帧周期与连续 `30 min` 稳定性，补齐考核指标。

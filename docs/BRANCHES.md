# 分支划分

## main

`main` 只保存已经确认的稳定能力：

- C# 同步采集源码。
- MATLAB ADC 解包、距离 FFT、多普勒 FFT、CFAR、近场 Bartlett 和点云聚类。
- 正式相机到雷达机械外参。
- 单组、批量和图形化文件选择入口。
- 通用标定、诊断和测试工具。
- 雷达测量偏差接口，但默认参数为零。

`main` 不自动应用 2026-11-05 的实验角度补偿。

## experiment/radar-angle-bias-20261105

该分支在 `main` 基础上增加：

- `radar_measurement_correction.json`。
- 自动加载距离、方位和俯仰候选补偿。
- 室外 3/4/5/6/7 m 验证脚本。
- 补偿配置测试。
- 原始验证 CSV 和文字分析。

候选参数：

```text
rangeBiasM       = 0.149477
azimuthBiasDeg   = 3.2096
elevationBiasDeg = 1.9813
```

## experiment/pixel-level-extrinsics-20261105

该分支在 `main` 基础上增加：

- 以棋盘格角点中心为独立真值的像素级对齐流程。
- 方位角补偿后，改用像素判据重解 `yaw`；再固定 `yaw` 一维搜索 `pitch`。
- `pixel_verify2.m`、`yaw_solve.m`、`pitch_solve.m`、`show_centers.m`。
- 文档 `docs/extrinsics/像素级外参校准-20261105.md` 与标注图。

结果口径：

```text
t            = [+0.0038, −0.0439, +0.1295] m
az 补偿       = 3.2096°
pitch        = 8.73°（中位，σ 1.93°，逐组使用实测值）
yaw          = 逐组解 −0.85° ~ +4.80°
6 组水平偏差   ≤ 5.2 px（4.3 cm）
6 组垂直偏差   ≤ 0.3 px（0.2 cm）
```

## 合并条件

实验分支只有满足以下条件后才允许合并到 `main`：

1. 新采集独立数据，不参与参数拟合。
2. 至少覆盖近、中、远三个距离和左右不同位置。
3. 每组目标关联明确，不使用“最近点”代替目标板心。
4. 所有独立组的三维误差均不超过 `0.15 m`。
5. 补偿在更换场景后仍保持同号、同量级。
6. MATLAB 单帧测试、批量计算和结果元数据检查全部通过。

## 后续分支命名

- 新算法：`feature/<name>`
- 标定实验：`experiment/<topic>-<date>`
- 修复：`fix/<problem>`
- 文档：`docs/<topic>`

不要直接在 `main` 上试调参数。


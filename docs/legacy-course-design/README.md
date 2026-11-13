# Legacy：雷达-相机融合课设阶段记录（2026-10）

本目录归档自原仓库 `tulip627722-byte/radar-camera-fusion`（信号处理课课设，项目目录 `E:\xinhaochuli`）。
该仓库已被主线 `signal-processing-multimodal-fusion`（`E:\leida`）全面取代，故删除原仓库，仅保留历史记录于此。

## 为什么被取代

| 维度 | 课设阶段（本归档） | 主线（当前仓库） |
|---|---|---|
| 采集 | 依赖老师提供的 GUI | 自研 C# WinForms 采集程序，新增激光雷达 Livox MID-70 同步 |
| 信号处理 | 依赖老师的算法链 + mex（`mex_RD_CFAR` 等） | 自研 MATLAB 链：距离 FFT / CFAR / 近场 Bartlett / 聚类 |
| 外参标定 | 仅 `estimate_extrinsic_from_pairs`（RANSAC+LM），当时不合格 | 完整工具链 + 候选外参 + 验收门限（板心误差 ≤ 0.15 m） |
| XYZ 验证 | 仅写了方案（`04`），未实现 | 已完成：板心对齐、留一验证、六折统计 |
| 内参 | MATLAB 标定彩色相机（1280×720） | 深度相机内参从 `meta.json` 读取 |

## 归档内容

- `01_采集与标定记录.md` — 课设阶段采集/标定全过程记录与踩坑（基线数据检查、内参标定过程）
- `02_静态外参采集执行步骤.md` — 静态外参采集 SOP
- `03_进展与目标_20261021.md` — 课设进展与卡点分析（卡在对应点质量）
- `04_XYZ验证方案.md` — 当时计划但未实现的 XYZ 验证方案（主线已完成）
- `calibrate_intrinsics.m` — MATLAB 棋盘格内参标定脚本（主线用 C# 从设备读内参，此脚本留作备用）
- `原仓库_README.md` — 原仓库 README

> 注意：课设 `code/` 下其余脚本（`export_radar_camera_pairs`、`estimate_extrinsic_from_pairs` 等）依赖老师的点云链与 mex，主线已重写，未迁移。

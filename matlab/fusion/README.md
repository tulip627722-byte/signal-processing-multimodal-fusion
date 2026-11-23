# 静态三模态镜头下融合

本目录提供 `Depth + MID-70 + ADT7880` 静态离线融合链路，不覆盖正式外参。

## 一键运行

```matlab
cd('E:\leida\radar-camera-alignment\matlab\fusion')
opts = struct('nDepthFrames', 20, 'maxDiagnosticPoints', 4000, ...
    'maxFusionPointsPerSensor', 30000, 'savePly', false);
report = run_offline_fusion( ...
    'E:\leida\11-18\20261118_193821_室内4米', opts);
```

输出到 `results/static_fusion/`：

- `*_offline_fusion.mat`：供 `view_take` 自动加载的完整离线结果。
- `*_fused.mat`：结构化 `fusedPc`、关联、预处理数据和诊断结果。
- `*_overview.png`：统一 ADT7880 坐标与镜头内三色融合图。
- `*_report.mat`：完整报告。

## 坐标和约束

- 统一坐标为 ADT7880：`X 右 / Y 前 / Z 上`。
- 变换均使用列向量约定 `p_target = R*p_source + t`。
- Depth 和 MID-70 参与几何诊断/可选 Point-to-Plane ICP。
- ADT7880 不参与 ICP，原始雷达点和簇心不会被吸附到几何表面。
- “雷达距离约束板面点”是跨模态语义点：毫米波原始 ADC 提供距离和回波能量，已对齐的 Depth/MID-70 板面提供方向。它与原始毫米波 XYZ 分开保存和显示。
- 距离候选只有在五组留一验证通过后才可使用；仓库归档候选的距离偏差为 `182.8 mm`，留一中位 `20.0 mm`、P95 `55.0 mm`。
- MID-70 和毫米波再反变换到相机坐标，用深度相机内参进行视锥裁切。

## 当前真实数据结果

对 `20261118_193821_室内4米` 使用 20 帧静态深度中值、1.5 s MID-70 累积和毫米波跨帧持续性过滤：Depth 271600 点，MID-70 90274 点，镜头内毫米波 38 点；Depth→MID-70 最近邻中位 114.4 mm，P95 227.6 mm。雷达距离约束板面点与板心误差约 `22 mm`；原始雷达簇仍按实测位置显示。

`view_take` 会自动查找 `results/static_fusion/<take>_offline_fusion.mat`，也可通过 `offlineFusionFile` 显式指定。

质量报告分别给出 `radarBoardStatus` 和 `depthLidarStatus`。总状态只有两项均通过才为 `PASS`；雷达板面通过但 Depth/MID-70 全场统计处于诊断范围时为 `WARN`。

`refine_depth_lidar_extrinsics` 已提供 ICP 候选接口；调用时设置 `runRefinement=true`，结果只写入报告，不会覆盖正式外参。

# 3.2 —— 距离参数按固件改正版

3.2 = 3.1 + **把测距参数改成固件里的真实值**。全自写、零二进制、无经验系数。

## 一、改了什么

原来的距离换算用的是 **chirp 周期 67 µs** 当斜坡时间。固件 `mmw_message.c` 里写的是：

```c
.startFreq = 77.000f;   .fm = 4000;        // 4 GHz
.chirpT0 = 2.00f;   .chirpT1 = 60.00f;     // ← 上升沿只有 60 µs
.chirpT2 = 5.00f;   .chirpT3 = 0.00f;      // 周期 = 2+60+5+0 = 67 µs
.adcSampleFreq = ADC_CLOCK_16P67M;         // 250 MHz / 15 = 16.667 MHz
.adcSamplePoint = 1024;
```

所以 `process_final.m` 现在这样算：

```matlab
T0 = 2e-6; T1 = 60e-6; T2 = 5e-6; T3 = 0;
Tchirp = T0+T1+T2+T3;      % 67 µs，只用于速度
Tramp  = T1;               % 60 µs，斜率用它
slope  = fm / Tramp;
dR     = c*Fs/(2*slope*NfftR);    % = 3.66 cm/bin
dv8T   = lambda/(2*Ndop*8*Tchirp);
```

**没有任何经验系数**（原来的 `rangeScale` 已删除）。`rangeOffsetM` 保留但默认 0，留给将来的偏移标定用。

修正前 dR = 4.09 cm（+12%），修正后 3.66 cm。

## 二、现在的残差（板子差分法实测）

| 标注 | 深度板 | 雷达 bin | 雷达 m | 雷达−深度 | 相对 |
|---|---|---|---|---|---|
| 2.0 m | 1.94 | 58 | 2.09 | **+0.15** | 7.8% |
| 2.5 m | 2.42 | 72 | 2.60 | **+0.18** | 7.4% |
| 3.0 m | （深度差分失败） | 86 | 3.11 | — | — |

**残差约 +0.16 m / 7%。** 顺便验证了一件事：去掉 12% 的斜率错误之后，2 米那组的板子峰从错误的 bin 68 挪到了正确的 **bin 58**，说明参数改对之后峰识别也变准了。

## 三、这 7% 到底是什么，还没定

两个模型都能拟合这两个点：

| 模型 | 参数 | 残差 |
|---|---|---|
| 固定偏移 | `r = r + 0.17` | 0.15 / 0.18 → 误差 < 3 cm |
| 比例误差 | `r = 1.075 · r` | 2.09/2.60/3.11 vs 1.94/2.42/2.93 → 误差 < 3 cm |

两个点跨越只有 0.5 m，分不出"加常数"还是"乘系数"。

**判别方法**：再采 1.0 / 1.5 / 4.0 / 5.0 m 四组板子（有板无板成对），6 个点一起拟合
`rng = a·bin + b`：

- `b ≈ 0`、`a` 大 7% → 比例误差（改斜率或带宽）
- `b ≈ 0.17`、`a` 正常 → 固定偏移（射频链群时延，约 1.1 ns，`rangeOffsetM` 设 −0.17 即可）

## 四、顺带查到的疑点

厂家库 `analog.c` 里有个寄存器换算：

```c
ch1Df = round(fm * 131.072 / (ch1StepNum - 1));
fmcwActualFM = ch1Df * (ch1StepNum - 1) * 1200 / pow(2,18);   // MHz
```

按 4 GHz / 7500 步代进去，算出来是 **2400 MHz**，不是 4000。要么量纲我读错了，要么实际扫频只有标称的 60%。
没有数据手册没法定论，先记在这里。

## 五、用法

### 图形化选择文件（推荐）

在 MATLAB 命令窗口运行：

```matlab
cd('E:\leida\3.2\matlab')
run_compute_interactive
```

界面支持：

- 单组计算：选择采集目录中的 `radar_adc.mat`。
- 批量计算：选择包含多组采集的父目录，例如 `E:\leida\11-5`。
- 计算前选择“应用当前补偿”或“原始数据（不补偿）”。
- 结果统一保存在 `3.2\results\takes`。

### 命令行调用

```matlab
cd <3.2>\matlab
compute_all                                        % 默认目录
compute_all([], struct('root','...\takes\11-4'))   % 指定采集根目录
view_take
```

### 雷达/相机对齐规则

- 时间：用 `radar_frames.csv` 中每个雷达物理帧的首包/末包时刻中点，匹配最近的深度帧和彩色帧。
- 坐标轴：相机 `X 右 / Y 下 / Z 前` 转为雷达 `X 右 / Y 前 / Z 上`。
- 平移：默认相机光心在雷达原点上方 `0.10 m`，来自装机尺寸，属于安装估计，不是精密外参标定。
- 旧的“每帧用 2% 高度分位数拉到地面”会随场景漂移，默认已关闭；仅可用 `useFloorAlignment=true` 复现旧结果。
- 外参唯一来源是 `radar_camera_extrinsics.json`。其中 `cameraToRadarR` 把相机点旋转到雷达轴；当前 `cameraOriginInRadarM=[-0.0093,-0.0063,0.1000] m`。X/Y 来自五组同步表面拟合，另一整组随机留出验证；Z=10 cm 仍来自安装测量，因为当前正前方数据不能独立约束垂直平移。
- 雷达测量补偿独立来自 `radar_measurement_correction.json`，当前按 `r-0.149477 m`、`az-3.2096 deg`、`el-1.9813 deg` 修正。它描述雷达算法偏差，不属于相机/雷达机械外参。
- `view_take` 的中间画面会把雷达点以红圈叠加到深度点云中；两者已经处于同一个雷达坐标系，可以直接检查重合。
- 修改外参文件后重新运行 `compute_all([], struct('force',true))`。每个结果的 `metaOut` 会保存实际使用的旋转矩阵、平移和外参状态。

## 六、工具

| 脚本 | 用途 |
|---|---|
| `tools\residual_check.m` | 板子差分，量当前参数下的残差（就是上面那张表） |
| `tools\alignment_report.m` | 把雷达点投影到同步深度图，报告空间残差与时间误差 |
| `tools\validate_outdoor_radar.m` | 用室外多距离棋盘数据验证测距、测角和坐标对齐 |
| `tools\test_radar_measurement_correction.m` | 检查雷达补偿配置、符号约定和正式外参未被改写 |
| `tools\calibrate_extrinsics_from_res.m` | 仅用九组 `res.mat` 点云差分，生成含角度、平移、测距偏差及六折验证的候选外参 |

### 基于 3.2 点云标定三脚架角度

```matlab
cd <3.2>\matlab
addpath tools
report = calibrate_extrinsics_from_res;
```

输出在 `results\extrinsics_calibration`。脚本不会覆盖正式外参；只有报告为 `PASS`、候选 JSON 中 `applicable=true` 时才能显式应用：

```matlab
candidate = fullfile('..','results','extrinsics_calibration','radar_camera_extrinsics_candidate.json');
compute_take(takeDir, struct('extrinsicsFile', candidate));
```

候选中的 `radarRangeBiasM` 会作为独立雷达测距补偿应用，不会混入相机到雷达的平移向量。
| `tools\board_diff.m` | 同上，早期版本 |
| `tools\verify_range.m` | 用 res 点云核对板子距离 |
| `tools\measure_gate.m` | CFAR 门限对比（1.60 / 1.53） |
| `tools\compare_dump.m` | 两版算法逐帧对比 |

# verify_range_fft

把 3.2 主链砍到只剩**距离维 FFT**，画距离-功率图。多普勒、CFAR、门限、测角、聚类、
ROI 全部不参与。

## 文件

| 文件 | 作用 |
|---|---|
| `range_fft_gui.m` | **弹窗**：选采集目录 / take / 帧数，出图、存图 |
| `range_params.m` | 全部参数：固件 chirp 值 → `slope` → `dR`，无经验系数 |
| `range_fft_frame.m` | 核心：单帧 → 距离维 FFT → 距离像，与 `process_final.m` 同源 |
| `take_range_profile.m` | 读 take，取 N 帧平均距离像（N=1 就是单帧） |
| `plot_range_power.m` | 画图；也可以直接命令行调用，或画进指定 axes |

`range_fft_frame.m` 里保留的就是这两句，一行没改：

```matlab
rangeFft = fft(adc8 .* reshape(wR, Nadc, 1, 1), NfftR, 1);
rangeFft = rangeFft(1:NfftR/2, :, :);
```

后面只加一步非相干累加（不含相位、不含多普勒）：`prof = sum(abs(rangeFft), [2 3])`。
窗函数直接复用 `..\algorithm\lib\hamming.m`。

## 用法一：弹窗

```matlab
cd 'D:\7880\Realtime\RealtimeOnPC\3.2\matlab\verify_range_fft'
range_fft_gui
```

- **采集目录**：take 的父目录，点「浏览…」也可以选；选完点「刷新」
- **take**：下拉列表，按时间新→旧排，只列含 `radar_adc.mat` 的子目录
- **帧数**：`1` = 单帧；`n` = 均匀取 n 帧平均
- **模式**：8T8R（64 通道）/ 1T8R（8 通道）
- **最远(m)**：横轴上限
- **峰区(m)**：只在这个距离窗口里找峰，窗口外不参与；窗口在图上是一块浅蓝带。
  缺省 `0.50`–`最远`
- **邻域(bin)**：峰的判据是局部极大值：前 `邻域` 个 bin 都更低、后 `邻域` 个 bin
  也都更低，才算一个峰（缺省 5）。区间里可能有多个峰，全部用橙点标出，
  最强的一个用红三角 + 文字，状态栏会列出来
- **画图 / 存图**；按回车也等于画图

打开时会自动选最新的那个 take 并画一次。

## 用法二：命令行

```matlab
plot_range_power                                        % 默认 2 米那组
plot_range_power(struct('nFrames', 1))                  % 单帧
plot_range_power(struct('takeDir', 'D:\...\20261104_134904_室内3米_1'))
plot_range_power(struct('xMax', 12, 'mode', '1T8R'))
plot_range_power(struct('peakRange', [1.5 3.5]))        % 只在 1.5-3.5 m 里找峰
plot_range_power(struct('peakRange', [1.5 3.5], 'peakHalf', 5))
plot_range_power(struct('savePath', 'D:\x.png'))
```

不打开界面：

```powershell
& 'D:\MATLAB\R2025b\bin\matlab.exe' -batch "cd('D:\7880\Realtime\RealtimeOnPC\3.2\matlab\verify_range_fft'); plot_range_power(struct('savePath','D:\range_power.png'))"
```

## 距离换算

```matlab
fm = 4e9;  Tramp = 60e-6;  Fs = 16.667e6;  NfftR = 1024;
slope = fm / Tramp;                    % 斜率只用上升沿（不是 67 us 周期）
dR = 3e8 * Fs / (2 * slope * NfftR);   % 0.0366 m/bin，1 m ≈ 27.3 bin
```

每 bin 16.28 kHz，每米 ≈ 444 kHz。`Fs/2` 对应 18.7 m。

## 峰的判据

不是区间最高点，是局部极大值：

```matlab
prof(k) > prof(k-1 .. k-5)   且   prof(k) > prof(k+1 .. k+5)
```

前后各 5 个 bin 都比它低，它才算一个峰（5 就是弹窗里的「邻域」，命令行对应
`peakHalf`）。这样泄漏裙那种一直往下走的斜坡不会被当成峰选进来，而一道墙、
一块板子这种真正的凸起会被选中。区间里如果有好几个峰，全部报出来，最强的是主峰。

## 看图

- 最前面那根是收发直接耦合的泄漏（`kRStart` 左边，前 8 bin），主链会切掉。
- 曲线是 8T8R 的 64 通道 × 16 chirp 非相干累加，不是单通道谱。
- 2 米那组里能同时看到板子（2.09 m，较弱）和它后面的墙（2.45 m，较强）。

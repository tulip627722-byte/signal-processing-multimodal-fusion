function mask = cfar_ca(RD, gR, gD, tR, tD, thr, bias)
%CFAR_CA 纯 MATLAB 的 2D CA-CFAR，行为对齐 mex_RD_CACFAR
%
%   mask = cfar_ca(RD, gR, gD, tR, tD, thr)
%   mask = cfar_ca(RD, gR, gD, tR, tD, thr, bias)
%
% 语义（由 512x128 尺寸下的探针实测反推）：
%   噪声估计 = 以测试单元为中心、(2*gR+1) x (2*gD+1) 窗口内 除自身外 的平均
%   判定：RD(i,j) > thr * bias * mean
%   边界：只算 i ∈ [gR+2, nR-gR-1]、j ∈ [gD+2, nD-gD-1]，其余一律 0
%
% 关于阈值：原厂 MEX 的等效噪声均值比纯算术均值低约 4.5%，所以同样的 1.6 门限下
% 自写版会少检约 10%。这里不内置任何魔法系数（bias 默认 1），
% 需要对齐时直接把门限按比例调低（1.6 → 1.53），见 tools\measure_gate.m 的实测。
%
% tR / tD 在实测里没有体现（传进去不影响结果），保留接口兼容。
%
% 与只认 512x128 的 MEX 不同，这里任意尺寸都能算。

if nargin < 3 || isempty(gD), gD = 6; end
if nargin < 6 || isempty(thr), thr = 1.6; end
if nargin < 7 || isempty(bias), bias = 1.0; end

RD = double(RD);
[nR, nD] = size(RD);
mask = false(nR, nD);

winR = gR; winD = gD;
mR = winR + 1;              % 边界余量（实测：行 1..19 不判，列 1..7 不判）
mD = winD + 1;

i1 = mR + 1; i2 = nR - mR;
j1 = mD + 1; j2 = nD - mD;
if i1 > i2 || j1 > j2, return; end

% 积分图（比真实 RD 各方向大 1）
I = zeros(nR + 1, nD + 1);
I(2:end, 2:end) = cumsum(cumsum(RD, 1), 2);

ii = (i1:i2).';
jj = j1:j2;
r1 = ii - winR; r2 = ii + winR;
c1 = jj - winD; c2 = jj + winD;

% 窗口和：S(i,j) = I(r2+1,c2+1) - I(r1,c2+1) - I(r2+1,c1) + I(r1,c1)
A = I(r2 + 1, c2 + 1);      % 行 = i, 列 = j
B = I(r1,     c2 + 1);      % r1 是窗口上沿，积分图里要取 r1 这一行
C = I(r2 + 1, c1);
D = I(r1,     c1);
S = A - B - C + D;

nCells = (2 * winR + 1) * (2 * winD + 1) - 1;
meanWin = (S - RD(i1:i2, j1:j2)) / nCells;

mask(i1:i2, j1:j2) = RD(i1:i2, j1:j2) > thr * bias * meanWin;
end

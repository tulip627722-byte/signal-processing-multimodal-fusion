function ground_pitch(varargin)
%GROUND_PITCH 用深度点云的地面平面性反解相机俯角
%
%   ground_pitch                       % 默认用 4/5/6m 三组
%   ground_pitch(resFile1, resFile2, ...)
%
% 原理：深度点云是按"相机光轴水平"假设反投影的。若相机实际有俯角 θ，
% 则地面在这个坐标系里是倾斜的；绕 X 轴旋转 θ 后地面应变成水平面
% （Z 恒定）。找到让地面 Z 最集中（宽度最小）的 θ 即为相机俯角。
%
% 只取 Y 远、|X| 小的区域（地面为主，避开桌子/墙交界），并用
% "Z 直方图最高峰附近的散布宽度"做鲁棒评分，避免墙面/桌子点干扰。

resDir = 'G:\3.2\results\takes';
if nargin == 0
    files = { fullfile(resDir, '20261104_162154_室内4米_res.mat'), ...
              fullfile(resDir, '20261104_162348_室内5米_res.mat'), ...
              fullfile(resDir, '20261104_162530_室内6米_res.mat') };
else
    files = varargin;
end

P = [];
for f = 1:numel(files)
    if ~isfile(files{f}), continue; end
    S = load(files{f}, 'depthPts', 'depthFrame');
    np = numel(S.depthPts);
    pick = round(linspace(1, np, min(10, np)));
    for k = pick
        Q = double(S.depthPts{k});
        if isempty(Q) || size(Q, 2) < 3, continue; end
        Q = Q(:, 1:3);
        smp = max(1, floor(size(Q, 1) / 1500));
        P = [P; Q(1:smp:end, :)]; %#ok<AGROW>
    end
    fprintf('载入 %s（%d 帧采样）\n', S.depthFrame(1), numel(pick));
end
if isempty(P), error('没有深度点云'); end

% 地面候选：远处、靠近正前方、位于相机下方
sel = P(:, 2) > 2.5 & P(:, 2) < 7.5 & abs(P(:, 1)) < 1.3 & P(:, 3) < 0.2;
P = P(sel, :);
fprintf('地面候选点 %d 个\n', size(P, 1));
if size(P, 1) < 200, error('地面候选点太少'); end

ths = -40:0.5:40;
w = zeros(size(ths));
zc = zeros(size(ths));
for i = 1:numel(ths)
    Q = P * Rx(ths(i));
    z = Q(:, 3);
    [w(i), zc(i)] = peakWidth(z);
end
[~, bi] = min(w);
fprintf('\n最优俯角 = %+.1f°（地面 Z 宽度 %.3f m，位于 Z=%.2f m）\n', ...
    ths(bi), w(bi), zc(bi));
fprintf('俯角 0° 时：宽度 %.3f m\n', w(ths == 0));

% 打印几条代表线
fprintf('\n%6s %8s %8s\n', 'θ(°)', '宽度m', 'Z中位');
for t = [-20 -15 -10 -5 0 5 10 15 20]
    j = find(abs(ths - t) < 0.26, 1);
    if ~isempty(j), fprintf('%6.1f %8.3f %8.2f\n', ths(j), w(j), zc(j)); end
end

% 相机高度 = 地面在相机系下的 Z（俯角修正后）
fprintf('\n→ 相机离地高度 ≈ %.2f m\n', abs(zc(bi)));
end

% ============================================================
function [wid, zc] = peakWidth(z)
% 取 Z 直方图最高 bin，返回该峰附近 ±0.35 m 内点的 5-95 分位宽度
z = z(isfinite(z));
if numel(z) < 50, wid = inf; zc = 0; return; end
edges = (min(z)):0.05:(max(z) + 0.05);
h = histcounts(z, edges);
[~, mi] = max(h);
zc = (edges(mi) + edges(mi + 1)) / 2;
in = z > zc - 0.45 & z < zc + 0.45;
if nnz(in) < 30, wid = inf; return; end
zz = sort(z(in));
lo = zz(max(1, round(0.05 * numel(zz))));
hi = zz(min(numel(zz), round(0.95 * numel(zz))));
wid = hi - lo;
end

function R = Rx(a)
s = sind(a); c = cosd(a); R = [1 0 0; 0 c -s; 0 s c];
end

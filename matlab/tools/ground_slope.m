function ground_slope(varargin)
%GROUND_SLOPE 用深度点云的"最低面包络"反解相机俯角
%
% 原理：地面是场景中最低的面。按纵向距离 Y 分箱，取每箱 Z 的最低 5% 分位，
% 得到地面轮廓线。相机若俯视 θ，这条线在光轴系中的斜率就是 -tan(θ)。
%
%   ground_slope                          % 默认 4/5/6m
%   ground_slope(resFile1, resFile2, ...)

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
    S = load(files{f}, 'depthPts');
    np = numel(S.depthPts);
    for k = round(linspace(1, np, min(12, np)))
        Q = double(S.depthPts{k});
        if isempty(Q) || size(Q, 2) < 3, continue; end
        Q = Q(:, 1:3);
        smp = max(1, floor(size(Q, 1) / 1200));
        P = [P; Q(1:smp:end, :)]; %#ok<AGROW>
    end
end
if isempty(P), error('没有深度点云'); end

sel = P(:, 2) > 1.0 & P(:, 2) < 7.5 & abs(P(:, 1)) < 1.4;
P = P(sel, :);
fprintf('参与统计的深度点 %d 个\n', size(P, 1));

edges = 1.0:0.4:7.5;
ctr = (edges(1:end-1) + edges(2:end)) / 2;
zmin = nan(size(ctr)); ncnt = zeros(size(ctr));
for i = 1:numel(ctr)
    in = P(:, 2) >= edges(i) & P(:, 2) < edges(i + 1);
    if nnz(in) < 40, continue; end
    zmin(i) = prctile(P(in, 3), 3);   % 该纵距区间内的最低面
    ncnt(i) = nnz(in);
end

good = isfinite(zmin) & ncnt > 150;
fprintf('\n%8s %9s %8s\n', 'Y(m)', 'Z最低', '点数');
for i = find(good)'
    fprintf('%8.2f %9.2f %8d\n', ctr(i), zmin(i), ncnt(i));
end
if nnz(good) < 3, error('有效区间太少'); end

cf = polyfit(ctr(good), zmin(good), 1);
theta = -atan(cf(1)) * 180 / pi;
fprintf('\n直线拟合：Z = %.3f·Y %+.2f\n', cf(1), cf(2));
fprintf('→ 相机俯角 ≈ %+.1f°（俯视为正）\n', theta);
fprintf('→ 地面在 Y=0 处 Z = %+.2f m，即相机离地 %.2f m\n', cf(2), abs(cf(2)));
fprintf('→ 说明：若俯角显著，深度相机读数是斜距，需乘 cos(θ)=%.3f 才是水平距离\n', cosd(theta));
end

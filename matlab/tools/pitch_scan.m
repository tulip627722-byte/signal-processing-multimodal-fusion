function pitch_scan(resFile, frameIdx, varargin)
%PITCH_SCAN 扫描相机俯角/偏航，找雷达点最贴合照片轮廓的参数
%
%   pitch_scan(resFile, frameIdx)
% 评分 = 雷达点投影位置上的照片边缘强度均值（强散射点应落在物体轮廓上）

ip = inputParser;
ip.addParameter('imin', 15);
ip.addParameter('pitch', []);
ip.addParameter('yaw', []);
ip.parse(varargin{:});
o = ip.Results;

S = load(resFile);
meta = S.metaOut;
n = numel(S.frames);
k = max(1, min(n, frameIdx));
K = [meta.fx meta.fy meta.cx meta.cy];
W = 640; H = 480;

fno = S.colorFrame(k);
img = imread(fullfile(meta.takeDir, 'color', sprintf('%06d.png', fno)));
g = double(img); if size(g, 3) > 1, g = mean(g, 3); end
[gx, gy] = gradient(g);
G = sqrt(gx.^2 + gy.^2);

P = S.pts{k};
P = P(P(:, 4) >= o.imin, :);
fprintf('take=%s 帧=%d 点数=%d\n', meta.takeName, fno, size(P, 1));

base = struct('tx', 0, 'ty', 0, 'tz', 0.10, 'yaw', 0, 'pitch', 0, 'roll', 0);
if ~isempty(o.pitch)
    base.pitch = o.pitch;
    if ~isempty(o.yaw), base.yaw = o.yaw; end
    prm = base;
    fprintf('直接出图：pitch=%+d° yaw=%+d°\n', prm.pitch, prm.yaw);
    render_out(P(:, 1:3), prm, K, W, H, img, resFile, meta, k, o.imin);
    return;
end
pitches = -45:1:45;
sc = zeros(size(pitches));
for i = 1:numel(pitches)
    q = base; q.pitch = pitches(i);
    sc(i) = score(P, q, K, G, W, H);
end
[bs, bi] = max(sc);
fprintf('最优 pitch = %+d°（得分 %.1f，pitch=0 时 %.1f）\n', pitches(bi), bs, sc(pitches == 0));

% 在最优 pitch 基础上扫 yaw
yaws = -25:1:25;
sc2 = zeros(size(yaws));
for i = 1:numel(yaws)
    q = base; q.pitch = pitches(bi); q.yaw = yaws(i);
    sc2(i) = score(P, q, K, G, W, H);
end
[bs2, bi2] = max(sc2);
fprintf('最优 yaw   = %+d°（得分 %.1f）\n', yaws(bi2), bs2);

prm = base; prm.pitch = pitches(bi); prm.yaw = yaws(bi2);

% 细扫 tz / tx
tzs = -0.6:0.02:0.6; sc3 = zeros(size(tzs));
for i = 1:numel(tzs)
    q = prm; q.tz = tzs(i); sc3(i) = score(P, q, K, G, W, H);
end
[~, i3] = max(sc3); prm.tz = tzs(i3);
txs = -0.6:0.02:0.6; sc4 = zeros(size(txs));
for i = 1:numel(txs)
    q = prm; q.tx = txs(i); sc4(i) = score(P, q, K, G, W, H);
end
[~, i4] = max(sc4); prm.tx = txs(i4);

fprintf('细化后：tx=%+.2f tz=%+.2f yaw=%+d° pitch=%+d°（得分 %.1f）\n', ...
    prm.tx, prm.tz, prm.yaw, prm.pitch, score(P, prm, K, G, W, H));

% 出图
[u, v, ok] = project_pts(P(:, 1:3), prm, K, W, H);
f = figure('Visible', 'off', 'Position', [0 0 640 480], 'Color', 'w');
imshow(img); hold on;
plot(u(ok), v(ok), 'o', 'MarkerSize', 5, 'MarkerFaceColor', [0 1 0], 'MarkerEdgeColor', 'k');
title(sprintf('pitch=%d° yaw=%d° tz=%.2f tx=%.2f  (%d 点)', prm.pitch, prm.yaw, prm.tz, prm.tx, nnz(ok)));
out = fullfile(fileparts(resFile), '..', sprintf('scan_%s_f%d.png', meta.takeName, k));
try
    exportapp(f, out);
catch
    print(f, out, '-dpng', '-r100');
end
close(f);
fprintf('已存图：%s\n', out);
end

% ============================================================
function render_out(P, prm, K, W, H, img, resFile, meta, k, imin)
[u, v, ok] = project_pts(P, prm, K, W, H);
f = figure('Visible', 'off', 'Position', [0 0 640 480], 'Color', 'w');
imshow(img); hold on;
plot(u(ok), v(ok), 'o', 'MarkerSize', 5, 'MarkerFaceColor', [0 1 0], 'MarkerEdgeColor', 'k');
plot(u(ok), v(ok), 'o', 'MarkerSize', 9, 'MarkerEdgeColor', [1 0 0], 'LineWidth', 1.2);
title(sprintf('pitch=%+d° yaw=%+d° tz=%.2f  (%d 点)', prm.pitch, prm.yaw, prm.tz, nnz(ok)));
out = fullfile(fileparts(resFile), '..', ...
    sprintf('manual_%s_f%d_p%d_y%d.png', meta.takeName, k, prm.pitch, prm.yaw));
try
    exportapp(f, out);
catch
    print(f, out, '-dpng', '-r100');
end
close(f);
fprintf('已存图：%s\n', out);
end

% ============================================================
function s = score(P, prm, K, G, W, H)
if size(P, 2) > 3, P = P(:, 1:3); end
[u, v, ok] = project_pts(P, prm, K, W, H);
if nnz(ok) < 30, s = 0; return; end
s = mean(interp2(G, u(ok), v(ok), 'linear', 0));
end

function [u, v, ok] = project_pts(P, prm, K, W, H)
t = [prm.tx, prm.ty, prm.tz];
R = Rz(prm.yaw) * Rx(prm.pitch) * Ry(prm.roll);
Pc = (P - t) * R;
xo = Pc(:, 1); yo = -Pc(:, 3); zo = Pc(:, 2);
ok = zo > 0.25;
u = nan(size(zo)); v = nan(size(zo));
u(ok) = K(1) .* xo(ok) ./ zo(ok) + K(3);
v(ok) = K(2) .* yo(ok) ./ zo(ok) + K(4);
ok = ok & u >= 1 & u <= W - 2 & v >= 1 & v <= H - 2;
u(~ok) = NaN; v(~ok) = NaN;
end

function R = Rz(a)
s = sind(a); c = cosd(a); R = [c -s 0; s c 0; 0 0 1];
end
function R = Ry(a)
s = sind(a); c = cosd(a); R = [c 0 s; 0 1 0; -s 0 c];
end
function R = Rx(a)
s = sind(a); c = cosd(a); R = [1 0 0; 0 c -s; 0 s c];
end

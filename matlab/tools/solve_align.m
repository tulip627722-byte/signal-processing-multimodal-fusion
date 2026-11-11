function solve_align(resFile, frameIdx, varargin)
%SOLVE_ALIGN 用深度图作为几何基准，解雷达-相机外参 t 和姿态
%
%   solve_align(resFile, frameIdx)
%
% 判据：把雷达点投影到彩色图，取同一像素处的深度读数 D（米），
% 要求雷达点在相机光轴方向的距离 zo 与 D 相等。
% 即最小化 mean(|zo - D|)。深度相机是独立的几何基准，不依赖照片纹理，
% 比"数边缘"稳健得多。
%
% 搜索 pitch / yaw / tx / ty / tz（坐标下降 + 多轮细化）

ip = inputParser;
ip.addParameter('show', true);
ip.addParameter('fixT', false);   % true=锁死安装值 [0,0,0.10]；false=平移也参与解算
ip.addParameter('K', []);         % 覆盖内参 [fx fy cx cy]；空则用 meta（深度口径）
ip.parse(varargin{:});
o = ip.Results;

S = load(resFile);
meta = S.metaOut;
if isempty(o.K)
    K = [meta.fx meta.fy meta.cx meta.cy];
else
    K = o.K;
end
W = 640; H = 480;
n = numel(S.frames);
k = max(1, min(n, frameIdx));

% 深度图（镜像，与彩色同向）
dfno = S.depthFrame(k);
dfile = fullfile(meta.takeDir, 'depth', sprintf('%06d.png', dfno));
if ~isfile(dfile), error('缺深度图 %s', dfile); end
D = imread(dfile);
if size(D, 3) > 1, D = D(:, :, 1); end
D = fliplr(double(D) * 0.001);
D(D < 0.3 | D > 9) = NaN;

P = double(S.pts{k});
if isempty(P), error('该帧没有点'); end
P = P(P(:, 2) > 0.5 & P(:, 2) < 9, :);
fprintf('take=%s 帧=%d 点数=%d 深度帧=%d\n', meta.takeName, k, size(P, 1), dfno);

% 平移固定为安装值 t = [0, 0, 0.10]（相机在雷达正上方 10cm），只解姿态
prm = struct('tx', 0, 'ty', 0, 'tz', 0.10, 'yaw', 0, 'pitch', 0, 'roll', 0);
if contains(meta.takeName, '2米') || contains(meta.takeName, '3米')
    prm.pitch = 20; prm.yaw = -10;
end
fprintf('初值 pitch=%+.1f yaw=%+.1f → 中位残差 %.3f m\n', prm.pitch, prm.yaw, score(P, prm, K, D, W, H));

% 第一轮：粗网格（只搜姿态）
prm = search(prm, 'pitch', -35, 45, 2, P, K, D, W, H);
prm = search(prm, 'yaw', -30, 30, 1.5, P, K, D, W, H);
prm = search(prm, 'roll', -8, 8, 1, P, K, D, W, H);
if ~o.fixT
    prm = search(prm, 'tz', -0.3, 0.6, 0.03, P, K, D, W, H);
    prm = search(prm, 'tx', -0.4, 0.4, 0.03, P, K, D, W, H);
    prm = search(prm, 'ty', -0.4, 0.4, 0.03, P, K, D, W, H);
end

% 第二轮：细搜
for r = 1:3
    prm = search(prm, 'pitch', prm.pitch - 2.5, prm.pitch + 2.5, 0.25, P, K, D, W, H);
    prm = search(prm, 'yaw', prm.yaw - 2.5, prm.yaw + 2.5, 0.25, P, K, D, W, H);
    prm = search(prm, 'roll', max(-8, prm.roll - 1), min(8, prm.roll + 1), 0.25, P, K, D, W, H);
    if ~o.fixT
        prm = search(prm, 'tz', prm.tz - 0.06, prm.tz + 0.06, 0.01, P, K, D, W, H);
        prm = search(prm, 'tx', prm.tx - 0.06, prm.tx + 0.06, 0.01, P, K, D, W, H);
        prm = search(prm, 'ty', prm.ty - 0.06, prm.ty + 0.06, 0.01, P, K, D, W, H);
    end
end

fprintf('\n===== 解算结果 =====\n');
fprintf('t   = [%+.3f, %+.3f, %+.3f] m\n', prm.tx, prm.ty, prm.tz);
fprintf('yaw = %+.2f°  pitch = %+.2f°  roll = %+.2f°\n', prm.yaw, prm.pitch, prm.roll);
fprintf('中位残差 %.3f m（初值 %.3f m）\n', score(P, prm, K, D, W, H), ...
    score(P, struct('tx',0,'ty',0,'tz',0.10,'yaw',0,'pitch',0,'roll',0), K, D, W, H));

% 出图
if o.show
    img = imread(fullfile(meta.takeDir, 'color', sprintf('%06d.png', S.colorFrame(k))));
    [u, v, ok] = proj(P(:, 1:3), prm, K, W, H);
    ok = ok & u >= 1 & u <= W & v >= 1 & v <= H;
    f = figure('Position', [80 80 1180 460], 'Color', 'w');
    subplot(1, 2, 1);
    imshow(img); hold on;
    plot(u(ok), v(ok), 'o', 'MarkerSize', 6, 'MarkerFaceColor', [0 1 1], ...
         'MarkerEdgeColor', [1 0 0], 'LineWidth', 1.2);
    title(sprintf('解算后：t=[%.2f %.2f %.2f] pitch=%.1f° yaw=%.1f°', ...
        prm.tx, prm.ty, prm.tz, prm.pitch, prm.yaw));
    subplot(1, 2, 2);
    DD = fliplr(D); DD(isnan(DD)) = 8;
    imagesc(DD); axis image; colormap(gca, parula); caxis([0.3 8]); hold on;
    plot(u(ok), v(ok), 'o', 'MarkerSize', 6, 'MarkerFaceColor', [1 0 1], ...
         'MarkerEdgeColor', 'w', 'LineWidth', 1.2);
    title('深度图 + 解算后投影');
    out = fullfile(fileparts(resFile), '..', sprintf('solved_%s_f%d.png', meta.takeName, k));
    try, exportapp(f, out); catch, print(f, out, '-dpng', '-r120'); end
    fprintf('已存图：%s\n', out);
    if ~usejava('desktop'), close(f); end
end

% 存结果
saveFile = fullfile(fileparts(resFile), '..', 'extrinsics_solved.json');
try
    T = struct('take', meta.takeName, 'frame', k, 't', [prm.tx prm.ty prm.tz], ...
        'yaw', prm.yaw, 'pitch', prm.pitch, 'roll', prm.roll, ...
        'median_residual_m', score(P, prm, K, D, W, H));
    txt = jsonencode(T);
    fid = fopen(saveFile, 'w');
    fprintf(fid, '%s\n', txt);
    fclose(fid);
    fprintf('已写入 %s\n', saveFile);
catch
end
end

% ============================================================
function best = search(prm, fld, lo, hi, step, P, K, D, W, H)
best = prm;
bs = score(P, prm, K, D, W, H);
if lo > hi, return; end
for val = lo:step:hi
    q = prm; q.(fld) = val;
    s = score(P, q, K, D, W, H);
    if s < bs, bs = s; best = q; end
end
end

% ============================================================
function s = score(P, prm, K, D, W, H)
[u, v, ok] = proj(P(:, 1:3), prm, K, W, H);
ok = ok & u >= 1 & u <= W & v >= 1 & v <= H;
if nnz(ok) < 30, s = 99; return; end
dv = interp2(D, u(ok), v(ok), 'linear');
% 相机光轴方向距离
t = [prm.tx, prm.ty, prm.tz];
R = Rz(prm.yaw) * Rx(prm.pitch) * Ry(prm.roll);
Pc = (P(ok, 1:3) - t) * R;
zo = Pc(:, 2);
fin = isfinite(dv) & zo > 0.3;
if nnz(fin) < 20, s = 99; return; end
r = sort(abs(zo(fin) - dv(fin)));
keep = r(1:max(1, round(0.9 * numel(r))));      % 去掉最大 10% 离群
% 覆盖率惩罚：防止把点挤进深度图的一小块有效区刷低残差
ratio = nnz(fin) / size(P, 1);
s = mean(keep) + 0.35 * max(0, 0.75 - ratio);
end

function [u, v, ok] = proj(P, prm, K, W, H)
t = [prm.tx, prm.ty, prm.tz];
R = Rz(prm.yaw) * Rx(prm.pitch) * Ry(prm.roll);
Pc = (P - t) * R;
xo = Pc(:, 1); yo = -Pc(:, 3); zo = Pc(:, 2);
ok = zo > 0.25;
u = nan(size(zo)); v = nan(size(zo));
u(ok) = K(1) .* xo(ok) ./ zo(ok) + K(3);
v(ok) = K(2) .* yo(ok) ./ zo(ok) + K(4);
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

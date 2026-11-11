function solve_all(varargin)
%SOLVE_ALL 多组数据联合标定：共享平移与姿态，用深度图作为几何基准
%
%   solve_all                          % 自动用 takes 目录下所有含板的组
%   solve_all('frames', 10, 'batch', '13')   % 只用 13 点批次
%
% 判据同 solve_align：雷达点投影到彩色图后，其相机光轴距离应等于同像素深度读数。

ip = inputParser;
ip.addParameter('frames', 10);
ip.addParameter('K', [491.4 655.7 319.5 239.5]);   % Astra Pro 彩色口径
ip.addParameter('batch', '');                       % '' / '13' / '16'
ip.addParameter('dir', 'G:\3.2\results\takes');
ip.parse(varargin{:});
o = ip.Results;

W = 640; H = 480;
files = dir(fullfile(o.dir, '*_res.mat'));
G = {};
for i = 1:numel(files)
    nm = files(i).name;
    if contains(nm, '无板'), continue; end
    if ~isempty(o.batch) && ~contains(nm, ['_' o.batch]), continue; end
    G{end+1} = fullfile(o.dir, nm); %#ok<AGROW>
end
if isempty(G), error('没找到数据'); end

D = {};
for i = 1:numel(G)
    S = load(G{i});
    m = S.metaOut;
    k = max(1, min(numel(S.frames), o.frames));
    dfno = S.depthFrame(k);
    dfile = fullfile(m.takeDir, 'depth', sprintf('%06d.png', dfno));
    dd = double(imread(dfile)) * 0.001;
    if size(dd, 3) > 1, dd = dd(:, :, 1); end
    dd = fliplr(dd);
    dd(dd < 0.3 | dd > 9) = NaN;
    P = double(S.pts{k});
    P = P(P(:, 2) > 0.5 & P(:, 2) < 9, :);
    D{i} = struct('name', m.takeName, 'P', P, 'D', dd); %#ok<AGROW>
    fprintf('  [%d] %s 帧%d 点数%d\n', i, m.takeName, k, size(P, 1));
end
fprintf('共 %d 组，K=[%.1f %.1f %.1f %.1f]\n', numel(D), o.K);

prm = struct('tx', 0, 'ty', 0, 'tz', 0.10, 'yaw', 0, 'pitch', 0, 'roll', 0);
if ~isempty(o.batch) && strcmp(o.batch, '13'), prm.pitch = 20; prm.yaw = -10; end
fprintf('初值 → %.3f m\n', score(D, prm, o.K, W, H));

% 粗搜
prm = search(D, prm, 'pitch', -30, 40, 2.5, o.K, W, H);
prm = search(D, prm, 'yaw', -30, 25, 1.5, o.K, W, H);
prm = search(D, prm, 'roll', -8, 8, 1, o.K, W, H);
prm = search(D, prm, 'tz', -0.3, 0.5, 0.03, o.K, W, H);
prm = search(D, prm, 'tx', -0.4, 0.4, 0.03, o.K, W, H);
prm = search(D, prm, 'ty', -0.4, 0.4, 0.03, o.K, W, H);

% 细搜
for r = 1:4
    prm = search(D, prm, 'pitch', prm.pitch - 2, prm.pitch + 2, 0.2, o.K, W, H);
    prm = search(D, prm, 'yaw', prm.yaw - 2, prm.yaw + 2, 0.2, o.K, W, H);
    prm = search(D, prm, 'roll', max(-8, prm.roll-1), min(8, prm.roll+1), 0.25, o.K, W, H);
    prm = search(D, prm, 'tz', prm.tz - 0.04, prm.tz + 0.04, 0.005, o.K, W, H);
    prm = search(D, prm, 'tx', prm.tx - 0.04, prm.tx + 0.04, 0.005, o.K, W, H);
    prm = search(D, prm, 'ty', prm.ty - 0.04, prm.ty + 0.04, 0.005, o.K, W, H);
end

fprintf('\n===== 联合解 =====\n');
fprintf('t   = [%+.3f, %+.3f, %+.3f] m\n', prm.tx, prm.ty, prm.tz);
fprintf('yaw = %+.2f°  pitch = %+.2f°  roll = %+.2f°\n', prm.yaw, prm.pitch, prm.roll);
fprintf('联合残差 %.3f m\n', score(D, prm, o.K, W, H));
fprintf('\n各组残差：\n');
for i = 1:numel(D)
    fprintf('  %-28s %.3f m\n', D{i}.name, score_one(D{i}, prm, o.K, W, H));
end

out = fullfile(o.dir, '..', 'extrinsics_joint.json');
fid = fopen(out, 'w');
fprintf(fid, '%s\n', jsonencode(struct('t', [prm.tx prm.ty prm.tz], 'yaw', prm.yaw, ...
    'pitch', prm.pitch, 'roll', prm.roll, 'K', o.K, 'joint_residual_m', score(D, prm, o.K, W, H))));
fclose(fid);
fprintf('\n已写入 %s\n', out);
end

% ============================================================
function s = score(D, prm, K, W, H)
s = 0;
for i = 1:numel(D), s = s + score_one(D{i}, prm, K, W, H); end
s = s / numel(D);
end

function s = score_one(G, prm, K, W, H)
[u, v, ok] = proj(G.P(:, 1:3), prm, K, W, H);
ok = ok & u >= 1 & u <= W & v >= 1 & v <= H;
if nnz(ok) < 25, s = 1; return; end
dv = interp2(G.D, u(ok), v(ok), 'linear');
R = Rz(prm.yaw) * Rx(prm.pitch) * Ry(prm.roll);
Pc = (G.P(ok, 1:3) - [prm.tx prm.ty prm.tz]) * R;
zo = Pc(:, 2);
fin = isfinite(dv) & zo > 0.3;
if nnz(fin) < 20, s = 1; return; end
r = sort(abs(zo(fin) - dv(fin)));
keep = r(1:max(1, round(0.9 * numel(r))));
ratio = nnz(fin) / size(G.P, 1);
s = mean(keep) + 0.35 * max(0, 0.75 - ratio);
end

function best = search(D, prm, fld, lo, hi, step, K, W, H)
best = prm;
bs = score(D, prm, K, W, H);
if lo > hi, return; end
for val = lo:step:hi
    q = prm; q.(fld) = val;
    s = score(D, q, K, W, H);
    if s < bs, bs = s; best = q; end
end
end

function [u, v, ok] = proj(P, prm, K, W, H)
R = Rz(prm.yaw) * Rx(prm.pitch) * Ry(prm.roll);
Pc = (P - [prm.tx prm.ty prm.tz]) * R;
xo = Pc(:, 1); yo = -Pc(:, 3); zo = Pc(:, 2);
ok = zo > 0.25;
u = nan(size(zo)); v = nan(size(zo));
u(ok) = K(1) .* xo(ok) ./ zo(ok) + K(3);
v(ok) = K(2) .* yo(ok) ./ zo(ok) + K(4);
end

function R = Rz(a), s = sind(a); c = cosd(a); R = [c -s 0; s c 0; 0 0 1]; end
function R = Ry(a), s = sind(a); c = cosd(a); R = [c 0 s; 0 1 0; -s 0 c]; end
function R = Rx(a), s = sind(a); c = cosd(a); R = [1 0 0; 0 c -s; 0 s c]; end

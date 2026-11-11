function solve_t_all()
%SOLVE_T_ALL 逐组解雷达-相机外参 t，最后按残差加权平均
%
% 运行：
%   cd G:\3.2\matlab
%   addpath('tools')
%   solve_t_all
%
% 判据：雷达点投影到彩色图后，其相机光轴距离应等于同像素处的深度读数。
% 内参用 Astra Pro 彩色口径 fx=491.4 fy=655.7 cx=319.5 cy=239.5。

K = [491.4 655.7 319.5 239.5];
W = 640; H = 480;
takesDir = 'G:\3.2\results\takes';

files = dir(fullfile(takesDir, '*_res.mat'));
R = struct('name', {}, 't', {}, 'yaw', {}, 'pitch', {}, 'roll', {}, 'resid', {}, 'depthMax', {});

fprintf('\n%-30s %8s %8s %8s | %8s %8s %8s | %7s %7s\n', ...
    '组', 'tx', 'ty', 'tz', 'yaw', 'pitch', 'roll', '残差m', '深度m');
fprintf('%s\n', repmat('-', 1, 104));

for i = 1:numel(files)
    nm = files(i).name;
    if contains(nm, '无板'), continue; end   % 无板组噪声大，不参与

    S = load(fullfile(takesDir, nm));
    m = S.metaOut;
    k = max(1, min(numel(S.frames), 10));

    df = fullfile(m.takeDir, 'depth', sprintf('%06d.png', S.depthFrame(k)));
    dd = double(imread(df)) * 0.001;
    if size(dd, 3) > 1, dd = dd(:, :, 1); end
    dd = fliplr(dd); dd(dd < 0.3 | dd > 9) = NaN;
    dMax = max(dd(:)); dMax = dMax(isfinite(dMax));
    if isempty(dMax), dMax = 0; end

    P = double(S.pts{k});
    P = P(P(:, 2) > 0.5 & P(:, 2) < min(9, dMax + 1.5), :);  % 深度外的雷达点没用，过滤掉

    if size(P, 1) < 50
        fprintf('%-30s %-4s  深度量程仅 %.2fm，雷达点 %d 太少，跳过\n', m.takeName, '[跳过]', dMax, size(P,1));
        continue;
    end

    % 初值：所有组先给一个合理初值，让搜索自己找
    prm = struct('tx', 0, 'ty', -0.070, 'tz', 0.13, 'yaw', 0, 'pitch', 0, 'roll', 0);
    if contains(m.takeName, '_16')
        prm.yaw = 1.5; prm.pitch = 10;
    elseif contains(m.takeName, '室外')
        prm.yaw = -1.0; prm.pitch = 3.0;   % 室外新批次先给个温和的初值
    else
        prm.yaw = -4.3; prm.pitch = 5.4;
    end

    % 粗搜（围绕初值收窄，提速）
    prm = srch(P, dd, prm, 'pitch', prm.pitch - 10, prm.pitch + 10, 1, K, W, H);
    prm = srch(P, dd, prm, 'yaw',   prm.yaw - 8,  prm.yaw + 8,  1, K, W, H);
    prm = srch(P, dd, prm, 'roll',  -6, 6, 1, K, W, H);
    prm = srch(P, dd, prm, 'tz',   -0.2, 0.5, 0.03, K, W, H);
    prm = srch(P, dd, prm, 'tx',   -0.3, 0.3, 0.02, K, W, H);
    prm = srch(P, dd, prm, 'ty',   -0.3, 0.3, 0.02, K, W, H);

    % 细搜两轮
    for r = 1:2
        prm = srch(P, dd, prm, 'pitch', prm.pitch - 2, prm.pitch + 2, 0.25, K, W, H);
        prm = srch(P, dd, prm, 'yaw',   prm.yaw - 2,   prm.yaw + 2,   0.25, K, W, H);
        prm = srch(P, dd, prm, 'roll',  prm.roll - 1,  prm.roll + 1,  0.25, K, W, H);
        prm = srch(P, dd, prm, 'tz',    prm.tz - 0.04, prm.tz + 0.04, 0.008, K, W, H);
        prm = srch(P, dd, prm, 'tx',    prm.tx - 0.04, prm.tx + 0.04, 0.008, K, W, H);
        prm = srch(P, dd, prm, 'ty',    prm.ty - 0.04, prm.ty + 0.04, 0.008, K, W, H);
    end

    rs = score_one(P, dd, prm, K, W, H);
    R(end+1) = struct('name', m.takeName, 't', [prm.tx prm.ty prm.tz], ...
        'yaw', prm.yaw, 'pitch', prm.pitch, 'roll', prm.roll, 'resid', rs, 'depthMax', dMax); %#ok<AGROW>

    fprintf('%-30s %+8.3f %+8.3f %+8.3f | %+7.2f %+7.2f %+7.2f | %7.3f %6.2f\n', ...
        m.takeName, prm.tx, prm.ty, prm.tz, prm.yaw, prm.pitch, prm.roll, rs, dMax);
end

if isempty(R)
    fprintf('\n所有组都被跳过，无法解算。\n'); return;
end

% ---- 过滤：去掉残差 >= 0.5 的坏组 ----
keep = [R.resid] < 0.5;
if nnz(keep) ~= numel(R)
    fprintf('\n过滤残差 >= 0.5 的组：');
    for i = find(~keep)
        fprintf('%s ', R(i).name);
    end
    fprintf('\n');
end
R = R(keep);

% ---- 加权平均（权重 = 1/残差²）----
resid = [R.resid];
tall = vertcat(R.t);
w = 1 ./ max(resid, 0.05).^2;
tAvg = sum(tall .* w', 1) / sum(w);
tMed = median(tall, 1);

fprintf('\n%s\n', repmat('=', 1, 104));
fprintf('参与统计 %d 组，残差范围 [%.3f, %.3f]\n', numel(R), min(resid), max(resid));
fprintf('残差加权平均 t = [%+.3f, %+.3f, %+.3f] m\n', tAvg(1), tAvg(2), tAvg(3));
fprintf('中位数        t = [%+.3f, %+.3f, %+.3f] m\n', tMed(1), tMed(2), tMed(3));
fprintf('逐组标准差    σ = [%.3f, %.3f, %.3f] m\n', std(tall, 0, 1));

% ---- 写结果 ----
out = fullfile(takesDir, '..', 'tall_result.json');
fid = fopen(out, 'w');
fprintf(fid, '%s\n', jsonencode(struct( ...
    'K', K, 't_weighted_mean', tAvg, 't_median', tMed, ...
    'sigma', std(tall, 0, 1), 'groups', {R})));
fclose(fid);
fprintf('\n已写入 %s\n', out);
end

% ============================================================
function best = srch(P, D, prm, fld, lo, hi, step, K, W, H)
best = prm;
bs = score_one(P, D, prm, K, W, H);
if lo > hi, return; end
for v = lo:step:hi
    q = prm; q.(fld) = v;
    s = score_one(P, D, q, K, W, H);
    if s < bs, bs = s; best = q; end
end
end

function s = score_one(P, D, prm, K, W, H)
[u, v, ok] = proj(P(:, 1:3), prm, K, W, H);
ok = ok & u >= 1 & u <= W & v >= 1 & v <= H;
if nnz(ok) < 25, s = 1; return; end
dv = interp2(D, u(ok), v(ok), 'linear');
Rm = Rz(prm.yaw) * Rx(prm.pitch) * Ry(prm.roll);
Pc = (P(ok, 1:3) - [prm.tx prm.ty prm.tz]) * Rm;
zo = Pc(:, 2);
fin = isfinite(dv) & zo > 0.3;
if nnz(fin) < 20, s = 1; return; end
r = sort(abs(zo(fin) - dv(fin)));
keep = r(1:max(1, round(0.9 * numel(r))));
ratio = nnz(fin) / size(P, 1);
s = mean(keep) + 0.35 * max(0, 0.75 - ratio);
end

function [u, v, ok] = proj(P, prm, K, W, H)
Rm = Rz(prm.yaw) * Rx(prm.pitch) * Ry(prm.roll);
Pc = (P - [prm.tx prm.ty prm.tz]) * Rm;
xo = Pc(:, 1); yo = -Pc(:, 3); zo = Pc(:, 2);
ok = zo > 0.25;
u = nan(size(zo)); v = nan(size(zo));
u(ok) = K(1) .* xo(ok) ./ zo(ok) + K(3);
v(ok) = K(2) .* yo(ok) ./ zo(ok) + K(4);
end

function R = Rz(a), s = sind(a); c = cosd(a); R = [c -s 0; s c 0; 0 0 1]; end
function R = Ry(a), s = sind(a); c = cosd(a); R = [c 0 s; 0 1 0; -s 0 c]; end
function R = Rx(a), s = sind(a); c = cosd(a); R = [1 0 0; 0 c -s; 0 s c]; end

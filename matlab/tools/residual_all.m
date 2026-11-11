function residual_all()
%RESIDUAL_ALL 六个距离的板子残差 + 拟合 rng = a·r_meas + b
%
% 2 / 2.5 / 3 m：有板−无板 差分定位板子（深度差分 + 雷达距离像差分）
% 4 / 5 / 6 m：无无板对照组，用深度图中央近距块自动找板+桌，雷达在 ±0.6 m 窗口内找峰
%
% 拟合结果判别：
%   b ≈ 0.17、a ≈ 1 → 固定偏移（射频群时延），process_final 里 rangeOffsetM = -b
%   b ≈ 0、a ≈ 1.07 → 比例误差（带宽/斜率），需改斜率或 dR

here = fileparts(mfilename('fullpath'));
ml = fileparts(here);
addpath(fullfile(ml, 'algorithm'), '-begin');
addpath(fullfile(ml, 'algorithm', 'lib'), '-begin');

root = 'G:\11-4\11-4';
c = 3e8; Fs = 16.667e6; fm = 4e9; T1 = 60e-6; Nfft = 1024;
dR = c * Fs / (2 * (fm / T1) * Nfft);
fprintf('dR = %.4f m/bin\n', dR);

rows = {};   % {标注, 深度板距, 雷达bin}

% ---------- 成对差分：2 / 2.5 / 3 m ----------
pairs = { ...
    {'20261104_132932_室内2米5_1', '20261104_133030_室内2米5_2'}, '20261104_133249_室内2米5_3_（无板子）', 2.5; ...
    {'20261104_133930_室内2米_1', '20261104_134227_室内2米_2'},   '20261104_134333_室内2米_3（无板子）',   2.0; ...
    {'20261104_134904_室内3米_1', '20261104_134957_室内3米_2'},   '20261104_135142_室内3米_3（无板子）',   3.0};
for p = 1:size(pairs, 1)
    lab = pairs{p, 3};
    dWith = mean_depth(root, pairs{p, 1});
    dNo   = mean_depth(root, {pairs{p, 2}});
    if isempty(dWith) || isempty(dNo)
        fprintf('%g m：深度图读不出来，跳过\n', lab); continue;
    end
    dd = dWith - dNo; dd(dWith <= 0 | dNo <= 0) = 0;
    mask = dd < -0.10;
    if ~any(mask(:))
        fprintf('%g m：深度差分没找到板，跳过\n', lab); continue;
    end
    dBoard = median(dWith(mask));
    rWith = mean_prof(root, pairs{p, 1});
    rNo   = mean_prof(root, {pairs{p, 2}});
    dr = rWith - rNo; dr(1:12) = 0; dr(dr < 0) = 0;
    [~, bi] = max(dr);
    rows{end+1} = {lab, dBoard, bi}; %#ok<AGROW>
end

% ---------- 单组：4 / 5 / 6 m ----------
singles = {'20261104_162154_室内4米', 4.0;
           '20261104_162348_室内5米', 5.0;
           '20261104_162530_室内6米', 6.0};
for s = 1:size(singles, 1)
    lab = singles{s, 2};
    dBoard = board_from_depth_single(fullfile(root, singles{s, 1}));
    if isnan(dBoard)
        fprintf('%g m：深度图没找到板/桌近距块，跳过\n', lab); continue;
    end
    rWith = mean_prof(root, {singles{s, 1}});
    lo = max(13, round((dBoard - 0.6) / dR) + 1);
    hi = min(numel(rWith), round((dBoard + 0.6) / dR) + 1);
    if hi <= lo
        fprintf('%g m：雷达窗口异常，跳过\n', lab); continue;
    end
    [~, rel] = max(rWith(lo:hi));
    bi = lo + rel - 1;
    rows{end+1} = {lab, dBoard, bi}; %#ok<AGROW>
end

% ---------- 汇总 + 拟合 ----------
n = numel(rows);
if n < 3, error('有效点太少（%d 个），没法拟合', n); end
x = zeros(n, 1); y = x; lbl = cell(n, 1);
for i = 1:n
    lbl{i} = sprintf('%g', rows{i}{1});
    x(i) = (rows{i}{3} - 1) * dR;   % 雷达读数（米）
    y(i) = rows{i}{2};              % 深度相机读数（米）
end

fprintf('\n%-8s %10s %10s %10s\n', '标注', '深度板', '雷达m', '雷达-深度');
fprintf('%s\n', repmat('-', 1, 42));
for i = 1:n
    fprintf('%-8s %10.2f %10.2f %+10.2f\n', lbl{i}, y(i), x(i), x(i) - y(i));
end

ab = [x, ones(n, 1)] \ y;
resid1 = x - y;                    % 只加常数 b 的模型残差
resid2 = x - (ab(1) * y + ab(2));  % 两参拟合残差
rms1 = sqrt(mean(resid1.^2)); rms2 = sqrt(mean(resid2.^2));
fprintf('\n===== 拟合 rng = a·r_meas + b =====\n');
fprintf('a = %.4f,  b = %+.3f m\n', ab(1), ab(2));
fprintf('只加常数（b 取均值 %+.3f）RMS = %.3f m\n', mean(resid1), rms1);
fprintf('两参拟合 RMS = %.3f m\n', rms2);
if abs(mean(resid1)) <= rms1 * 1.3 && abs(ab(1) - 1) < 0.03
    fprintf('→ 判别：固定偏移为主，rangeOffsetM 设 %.2f\n', -mean(resid1));
elseif abs(ab(1) - 1) >= 0.03
    fprintf('→ 判别：比例误差为主（约 %+.1f%%），需改斜率/带宽\n', 100 * (ab(1) - 1));
else
    fprintf('→ 判别：不明显，需要更准的参照\n');
end
end

% ============================================================
function M = mean_depth(root, takes)
M = [];
for t = 1:numel(takes)
    td = fullfile(root, takes{t});
    if ~isfolder(td), continue; end
    f = dir(fullfile(td, 'depth', '*.png'));
    pick = f(round(linspace(1, numel(f), 8)));
    acc = [];
    for j = 1:numel(pick)
        D = imread(fullfile(td, 'depth', pick(j).name));
        if size(D, 3) > 1, D = D(:, :, 1); end
        Z = double(D) * 0.001;
        if isempty(acc), acc = Z; else, acc = acc + Z; end
    end
    if ~isempty(acc), M = acc / numel(pick); end
end
end

% ============================================================
function dBoard = board_from_depth_single(td)
% 单组无对照：深度图中央区域的近距块 = 桌子+板，取块上半（板面）的中位距离
f = dir(fullfile(td, 'depth', '*.png'));
pick = f(round(linspace(1, numel(f), 8)));
S = [];
for j = 1:numel(pick)
    D = imread(fullfile(td, 'depth', pick(j).name));
    if size(D, 3) > 1, D = D(:, :, 1); end
    Z = reshape(double(D) * 0.001, [size(D, 1), size(D, 2), 1]);
    if isempty(S), S = Z; else, S = cat(3, S, Z); end
end
if isempty(S), dBoard = NaN; return; end
M = mean(S, 3);
M(M < 0.3 | M > 9) = NaN;
[rh, ~] = size(M);
inner = false(rh, size(M, 2));
inner(120:420, 140:500) = true;
Mc = M; Mc(~inner) = NaN;
bg = median(Mc(isfinite(Mc)));
if isnan(bg), dBoard = NaN; return; end
mask = inner & M < bg - 0.35;
if nnz(mask) < 300, mask = inner & M < bg - 0.20; end
if nnz(mask) < 200, dBoard = NaN; return; end
rr = find(any(mask, 2));
upper = rr(1):floor((rr(1) + rr(end)) / 2);
m2 = false(size(mask)); m2(upper, :) = true;
mask = mask & m2;
dBoard = median(M(mask));
end

% ============================================================
function prof = mean_prof(root, takes)
prof = [];
for t = 1:numel(takes)
    td = fullfile(root, takes{t});
    if ~isfolder(td), continue; end
    S = load(fullfile(td, 'radar_adc.mat'), 'ADCData');
    adt = S.ADCData; per = 2048; payloadBytes = numel(adt{1, 4});
    ids = zeros(size(adt, 1), 1);
    for m = 1:size(adt, 1)
        v = adt{m, 2}; if iscell(v) && numel(v) == 1, v = v{1}; end
        ids(m) = double(v(1));
    end
    [sIds, sIdx] = sort(ids);
    fidx = floor(sIds / per) + 1; [uf, ~, grp] = unique(fidx); cnt = accumarray(grp, 1);
    done = sort(uf(cnt == per));
    pick = done(round(linspace(1, numel(done), 4)));
    acc = [];
    for k = pick(:).'
        raw = assemble(adt, sIds, sIdx, k, per, payloadBytes);
        RD = build_rd(raw);
        pr = mean(RD(:, 58:70), 2);
        if isempty(acc), acc = pr; else, acc = acc + pr; end
    end
    if ~isempty(acc), prof = acc / numel(pick); end
end
end

% ============================================================
function RD = build_rd(raw)
Nadc = 1024; Nrx = 8; Ntx = 8; NchirpPerTx = 16; Ndop = 128; NfftR = 1024;
raw = reshape(uint8(raw), 2048, 2048);
adc = double(uint16(raw(1:2:end, :))) + 256 * double(uint16(raw(2:2:end, :))) - 2048;
wR = single(hamming(Nadc)); wD = single(hamming(NchirpPerTx)).';
adc8 = reshape(adc(:, 1025:2048), Nadc, Ntx, NchirpPerTx, Nrx);
adc8 = permute(adc8, [1, 4, 2, 3]);
adc8 = reshape(adc8, Nadc, Nrx * Ntx, NchirpPerTx);
adc8 = single(adc8);
rangeFft = fft(adc8 .* reshape(wR, Nadc, 1, 1), NfftR, 1);
rangeFft = rangeFft(1:NfftR/2, :, :);
rd = fft(rangeFft .* reshape(wD, 1, 1, NchirpPerTx), Ndop, 3);
rd = rd(:, :, [65:128, 1:64]);
RDcube = permute(rd, [1 3 2]);
RD = sum(abs(RDcube), 3);
end

% ============================================================
function raw = assemble(adt, sortedIds, sortIdx, frameNumber, per, payloadBytes)
lo = (frameNumber - 1) * per;
p1 = find(sortedIds >= lo, 1, 'first');
raw = zeros(payloadBytes, per, 'uint8');
for i = 1:per
    raw(:, i) = uint8(adt{sortIdx(p1 + i - 1), 4}(:));
end
end

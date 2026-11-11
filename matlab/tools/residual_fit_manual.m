function residual_fit_manual()
%RESIDUAL_FIT_MANUAL 六点残差拟合（4/5/6m 用手工框定的板位置）
%
% 拟合 雷达读数 = a·真实距离 + b：
%   a≈1, b≈+0.17 → 固定偏移（群时延）→ process_final 的 rangeOffsetM = -b
%   a≈1.07, b≈0  → 比例误差（带宽）→ 需改斜率

here = fileparts(mfilename('fullpath'));
ml = fileparts(here);
addpath(fullfile(ml, 'algorithm'), '-begin');
addpath(fullfile(ml, 'algorithm', 'lib'), '-begin');

root = 'G:\11-4\11-4';
c = 3e8; Fs = 16.667e6; fm = 4e9; T1 = 60e-6; Nfft = 1024;
dR = c * Fs / (2 * (fm / T1) * Nfft);

rows = {};   % {标注, 深度真值m, 雷达读数m}

% ---------- 成对差分（可信）：2 / 2.5 m ----------
pairs = { ...
    {'20261104_132932_室内2米5_1', '20261104_133030_室内2米5_2'}, '20261104_133249_室内2米5_3_（无板子）', 2.5; ...
    {'20261104_133930_室内2米_1', '20261104_134227_室内2米_2'},   '20261104_134333_室内2米_3（无板子）',   2.0};
for p = 1:size(pairs, 1)
    lab = pairs{p, 3};
    dWith = mean_depth(root, pairs{p, 1});
    dNo   = mean_depth(root, {pairs{p, 2}});
    dd = dWith - dNo; dd(dWith <= 0 | dNo <= 0) = 0;
    mask = dd < -0.10;
    if ~any(mask(:)), fprintf('%g m：深度差分失败\n', lab); continue; end
    dBoard = median(dWith(mask));
    rWith = mean_prof(root, pairs{p, 1});
    rNo   = mean_prof(root, {pairs{p, 2}});
    dr = rWith - rNo; dr(1:12) = 0; dr(dr < 0) = 0;
    [~, bi] = max(dr);
    rows{end+1} = {lab, dBoard, (bi - 1) * dR}; %#ok<AGROW>
end

% ---------- 手工框：3 / 4 / 5 / 6 m（框 = 彩色图坐标） ----------
boxes = { ...
    '20261104_134904_室内3米_1', 255, 345, 300, 400, 3.0; ...
    '20261104_162154_室内4米',   290, 375, 310, 395, 4.0; ...
    '20261104_162348_室内5米',   300, 370, 305, 375, 5.0; ...
    '20261104_162530_室内6米',   330, 390, 298, 355, 6.0};
for s = 1:size(boxes, 1)
    lab = boxes{s, 6};
    td = fullfile(root, boxes{s, 1});
    [dBoard, which, gap] = board_depth_manual(td, {boxes{s, 2:5}});
    if isnan(dBoard)
        fprintf('%g m：深度框无效（有效像素不足）\n', lab); continue;
    end
    fprintf('%g m：板距 %.2f m（用%s映射，环差 %.2f m）\n', lab, dBoard, ...
        ternary(which == 1, '直取', '镜像'), gap);
    rWith = mean_prof(root, {boxes{s, 1}});
    lo = max(13, round((dBoard - 0.5) / dR) + 1);
    hi = min(numel(rWith), round((dBoard + 0.5) / dR) + 1);
    [~, rel] = max(rWith(lo:hi));
    bi = lo + rel - 1;
    rows{end+1} = {lab, dBoard, (bi - 1) * dR}; %#ok<AGROW>
end

% ---------- 汇总拟合 ----------
n = numel(rows);
x = zeros(n, 1); y = x; lbl = cell(n, 1);
for i = 1:n
    lbl{i} = sprintf('%g', rows{i}{1});
    y(i) = rows{i}{2};   % 深度真值
    x(i) = rows{i}{3};   % 雷达读数
end
fprintf('\n%-8s %10s %10s %10s\n', '标注', '深度', '雷达', '雷达-深度');
fprintf('%s\n', repmat('-', 1, 42));
for i = 1:n
    fprintf('%-8s %10.2f %10.2f %+10.2f\n', lbl{i}, y(i), x(i), x(i) - y(i));
end
ab = [x, ones(n, 1)] \ y;
fprintf('\n拟合：雷达 = %.4f ·真实 %+.3f\n', ab(1), ab(2));
fprintf('固定偏移模型：b 均值 %+.3f m，RMS %.3f m\n', mean(x - y), sqrt(mean((x - y - mean(x - y)).^2)));
if abs(ab(1) - 1) < 0.03
    fprintf('→ 固定偏移为主，rangeOffsetM = %.2f\n', -mean(x - y));
else
    fprintf('→ 比例成分显著（a=%.3f），加常数后残余 RMS %.3f m\n', ab(1), ...
        sqrt(mean((x - (ab(1) .* y + ab(2))).^2)));
end
end

% ============================================================
function [dBoard, which, bestGap] = board_depth_manual(td, uv)
f = dir(fullfile(td, 'depth', '*.png'));
pick = f(round(linspace(1, numel(f), 8)));
S = [];
for j = 1:numel(pick)
    D = imread(fullfile(td, 'depth', pick(j).name));
    if size(D, 3) > 1, D = D(:, :, 1); end
    S = cat(3, S, reshape(double(D) * 0.001, [size(D, 1), size(D, 2), 1]));
end
M = mean(S, 3);
M(M < 0.3 | M > 9) = NaN;
[rh, W] = size(M);
u1 = uv{1}; u2 = uv{2}; v1 = uv{3}; v2 = uv{4};
cand = [u1, u2; W - 1 - u2, W - 1 - u1];   % 直取 / 镜像
dBoard = NaN; which = 0; bestGap = -inf;
for k = 1:2
    ua = max(1, round(min(cand(k, :)))); ub = min(W, round(max(cand(k, :))));
    box = M(max(1, v1):min(rh, v2), ua:ub);
    bv = box(isfinite(box));
    if numel(bv) < 50, continue; end
    ringMask = false(size(M));
    ringMask(max(1, v1 - 60):min(rh, v2 + 60), max(1, ua - 60):min(W, ub + 60)) = true;
    ringMask(max(1, v1):min(rh, v2), ua:ub) = false;
    rv = M(ringMask); rv = rv(isfinite(rv));
    if numel(rv) < 100, continue; end
    gap = median(rv) - median(bv);   % 板比周边近 → 正
    if gap > bestGap
        bestGap = gap; dBoard = median(bv); which = k;
    end
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

% ============================================================
function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end

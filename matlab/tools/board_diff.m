function board_diff(root)
%BOARD_DIFF 用"有板子 - 无板子"的差分，把板子的距离单独提取出来
%
% 这是最干净的办法：环境（窗、墙、桌子）在两次采集里都在，减掉就只剩板子。

if nargin < 1, root = 'D:\7880\Realtime\capture\takes\11-4'; end
here = fileparts(mfilename('fullpath'));
ml = fileparts(here);
addpath(fullfile(ml, 'algorithm'), '-begin');
addpath(fullfile(ml, 'algorithm', 'lib'), '-begin');

c = 3e8; Fs = 16.667e6; B = 4e9; T = 67e-6; slope = B / T; NfftR = 1024;
dR = (c / (2 * slope)) * (Fs / NfftR);

pairs = { ...
    {'20261104_132932_室内2米5_1', '20261104_132932_室内2米5_2'}, '20261104_133249_室内2米5_3_（无板子）', 2.5; ...
    {'20261104_133930_室内2米_1',  '20261104_134227_室内2米_2'},  '20261104_134333_室内2米_3（无板子）', 2.0; ...
    {'20261104_134904_室内3米_1',  '20261104_134957_室内3米_2'},  '20261104_135142_室内3米_3（无板子）', 3.0};

fprintf('\ndR = %.4f m/bin\n', dR);
fprintf('%-8s %10s %10s %10s\n', '标注m', '深度差分m', '雷达差分m', '雷达-深度');
fprintf('%s\n', repmat('-', 1, 44));

for p = 1:size(pairs, 1)
    withTakes = pairs{p, 1};
    noTake = pairs{p, 2};
    tape = pairs{p, 3};

    % ---- 深度差分 ----
    dWith = mean_depth_map(root, withTakes);
    dNo = mean_depth_map(root, {noTake});
    if isempty(dWith) || isempty(dNo), continue; end
    dd = dWith - dNo;                 % 板子处深度变小 → 差分为负
    dd(dWith <= 0 | dNo <= 0) = 0;
    dd(dd > -0.05) = 0;               % 只保留明显变化的地方
    if any(dd(:) ~= 0)
        Zc = dWith(dd ~= 0);
        depthBoard = median(Zc);
    else
        depthBoard = NaN;
    end

    % ---- 雷达差分 ----
    rWith = mean_range_profile(root, withTakes);
    rNo = mean_range_profile(root, {noTake});
    if isempty(rWith) || isempty(rNo), continue; end
    dr = rWith - rNo;
    dr(1:12) = 0;
    dr(dr < 0) = 0;
    [~, bi] = max(dr(round(0.8/dR):min(end, round(5/dR))));
    lo = round(0.8/dR);
    bi = lo + bi - 1;
    radarBoard = (bi - 1) * dR;

    fprintf('%-8.1f %10.2f %10.2f %+10.2f\n', tape, depthBoard, radarBoard, radarBoard - depthBoard);
end
end

% ============================================================
function M = mean_depth_map(root, takes)
M = [];
for t = 1:numel(takes)
    td = fullfile(root, takes{t});
    if ~isfolder(td), continue; end
    dfiles = dir(fullfile(td, 'depth', '*.png'));
    pick = dfiles(round(linspace(1, numel(dfiles), 8)));
    acc = [];
    for j = 1:numel(pick)
        D = imread(fullfile(td, 'depth', pick(j).name));
        if size(D,3) > 1, D = D(:,:,1); end
        Z = double(D) * 0.001;
        if isempty(acc), acc = Z; else, acc = acc + Z; end
    end
    if ~isempty(acc), M = acc / numel(pick); end
end
end

% ============================================================
function prof = mean_range_profile(root, takes)
prof = [];
for t = 1:numel(takes)
    td = fullfile(root, takes{t});
    if ~isfolder(td), continue; end
    S = load(fullfile(td, 'radar_adc.mat'), 'ADCData');
    adt = S.ADCData; per = 2048; payloadBytes = numel(adt{1,4});
    ids = zeros(size(adt,1),1);
    for m = 1:size(adt,1)
        v = adt{m,2}; if iscell(v) && numel(v)==1, v = v{1}; end
        ids(m) = double(v(1));
    end
    [sIds, sIdx] = sort(ids);
    fidx = floor(sIds/per)+1; [uf,~,grp] = unique(fidx); cnt = accumarray(grp,1);
    done = sort(uf(cnt==per));
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
adc = double(uint16(raw(1:2:end,:))) + 256*double(uint16(raw(2:2:end,:))) - 2048;
wR = single(hamming(Nadc)); wD = single(hamming(NchirpPerTx)).';
adc8 = reshape(adc(:,1025:2048), Nadc, Ntx, NchirpPerTx, Nrx);
adc8 = permute(adc8, [1,4,2,3]);
adc8 = reshape(adc8, Nadc, Nrx*Ntx, NchirpPerTx);
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

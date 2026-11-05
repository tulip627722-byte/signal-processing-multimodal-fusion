function residual_check()
%RESIDUAL_CHECK 3.2（纯固件参数，无任何经验系数）下，板子的距离残差
%
% 用差分法（有板 − 无板）在距离像上找板子，再换算成米，和深度对照。

here = fileparts(mfilename('fullpath'));
ml = fileparts(here);
addpath(fullfile(ml, 'algorithm'), '-begin');
addpath(fullfile(ml, 'algorithm', 'lib'), '-begin');

root = 'D:\7880\Realtime\capture\takes\11-4';
c = 3e8; Fs = 16.667e6; fm = 4e9; T1 = 60e-6; Nfft = 1024;
dR = c * Fs / (2 * (fm / T1) * Nfft);
fprintf('\n固件参数：B=%.1f GHz, Tramp=%.0f us, Fs=%.3f MHz, Nfft=%d\n', ...
    fm/1e9, T1*1e6, Fs/1e6, Nfft);
fprintf('   → dR = %.4f m/bin\n\n', dR);

pairs = { ...
    {'20261104_132932_室内2米5_1','20261104_133030_室内2米5_2'}, '20261104_133249_室内2米5_3_（无板子）', 2.5; ...
    {'20261104_133930_室内2米_1','20261104_134227_室内2米_2'},   '20261104_134333_室内2米_3（无板子）', 2.0; ...
    {'20261104_134904_室内3米_1','20261104_134957_室内3米_2'},   '20261104_135142_室内3米_3（无板子）', 3.0};

fprintf('%-8s %9s %9s %9s %9s %9s\n', '标注', '深度板', '雷达bin', '雷达m', '雷达-深度', '相对误差');
fprintf('%s\n', repmat('-', 1, 62));

for p = 1:size(pairs, 1)
    dWith = mean_depth(root, pairs{p,1});
    dNo   = mean_depth(root, {pairs{p,2}});
    if isempty(dWith) || isempty(dNo), continue; end
    dd = dWith - dNo;  dd(dWith <= 0 | dNo <= 0) = 0;
    mask = dd < -0.10;
    if ~any(mask(:)), continue; end
    depthBoard = median(dWith(mask));

    rWith = mean_prof(root, pairs{p,1});
    rNo   = mean_prof(root, {pairs{p,2}});
    if isempty(rWith) || isempty(rNo), continue; end
    dr = rWith - rNo; dr(1:12) = 0; dr(dr < 0) = 0;
    [~, bi] = max(dr);
    rM = (bi - 1) * dR;
    fprintf('%-8.1f %9.2f %9d %9.2f %+9.2f %8.1f%%\n', pairs{p,3}, depthBoard, bi, rM, ...
        rM - depthBoard, 100*(rM - depthBoard)/depthBoard);
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
        if size(D,3) > 1, D = D(:,:,1); end
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

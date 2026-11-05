function tune_cfar()
%TUNE_CFAR 用一个比例因子把自写 CFAR 的检出数对齐到原厂 MEX
addpath(fullfile(fileparts(mfilename('fullpath')), 'algorithm'), '-begin');
addpath(fullfile(fileparts(mfilename('fullpath')), 'algorithm', 'lib'), '-begin');

S30 = load(fullfile(fileparts(mfilename('fullpath')), '..', 'results', 'cmp30.mat'));
ref = S30.res;
cap = 'D:\7880\Realtime\capture\takes';
ks = 0.90:0.005:1.05;

fprintf('帧                MEX CFAR  %s\n', sprintf('%7.3f', ks(1:min(8, numel(ks)))));
errSum = zeros(size(ks));

for t = 1:numel(ref)
    td = fullfile(cap, ref(t).take);
    S = load(fullfile(td, 'radar_adc.mat'), 'ADCData');
    adt = S.ADCData; per = 2048; payloadBytes = numel(adt{1,4});
    ids = zeros(size(adt,1),1);
    for i = 1:size(adt,1)
        v = adt{i,2}; if iscell(v) && numel(v)==1, v = v{1}; end
        ids(i) = double(v(1));
    end
    [sIds, sIdx] = sort(ids);
    fidx = floor(sIds/per)+1; [uf,~,grp] = unique(fidx); cnt = accumarray(grp,1);
    done = sort(uf(cnt==per));

    raw = assemble(adt, sIds, sIdx, ref(t).frame, per, payloadBytes);
    RD = build_rd(raw);

    counts = zeros(size(ks));
    for i = 1:numel(ks)
        m = cfar_ca(RD, 18, 6, 6, 2, 1.6 * ks(i));
        counts(i) = nnz(m);
    end
    errSum = errSum + abs(log(counts / ref(t).cfar));
    fprintf('%s/%d  %6d      %s\n', ref(t).take(1:12), ref(t).frame, ref(t).cfar, ...
        sprintf('%7d', counts(1:min(8, numel(ks)))));
end

[~, bi] = min(errSum);
fprintf('\n最优比例 k = %.3f（阈值 = 1.6*k*均值）\n', ks(bi));
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

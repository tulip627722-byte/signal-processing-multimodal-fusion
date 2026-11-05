function range_profile_peak(root, names)
%RANGE_PROFILE_PEAK 直接从距离像找板子的峰（不经过测角/ROI），和深度对比
if nargin < 2 || isempty(names)
    names = {'20261104_132932_室内2米5_1','20261104_133030_室内2米5_2', ...
             '20261104_133930_室内2米_1','20261104_134227_室内2米_2', ...
             '20261104_134904_室内3米_1','20261104_134957_室内3米_2'};
end
here = fileparts(mfilename('fullpath'));
ml = fileparts(here);
addpath(fullfile(ml, 'algorithm'), '-begin');
addpath(fullfile(ml, 'algorithm', 'lib'), '-begin');

c = 3e8; Fs = 16.667e6; B = 4e9; T = 67e-6; slope = B / T; NfftR = 1024;
dR = (c / (2 * slope)) * (Fs / NfftR);
fprintf('距离分辨率 dR = %.4f m/bin\n\n', dR);
fprintf('%-34s %8s %8s %8s %8s\n', 'take', '深度中位', '雷达峰bin', '雷达峰m', '差');
fprintf('%s\n', repmat('-', 1, 74));

for i = 1:numel(names)
    td = fullfile(root, names{i});
    if ~isfolder(td), continue; end

    % 深度：中央区域
    dfiles = dir(fullfile(td, 'depth', '*.png'));
    pick = dfiles(round(linspace(1, numel(dfiles), 5)));
    vd = [];
    for j = 1:numel(pick)
        D = imread(fullfile(td, 'depth', pick(j).name));
        if size(D,3) > 1, D = D(:,:,1); end
        Z = double(D) * 0.001;
        c0 = Z(180:300, 250:390);
        c0 = c0(c0 > 0.5 & c0 < 6);
        if ~isempty(c0), vd(end+1) = median(c0); end %#ok<AGROW>
    end

    % 雷达：距离像在 Doppler=0 附近的峰值（静态板子）
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
    pickr = done(round(linspace(1, numel(done), 3)));
    bins = [];
    for k = pickr(:).'
        raw = assemble(adt, sIds, sIdx, k, per, payloadBytes);
        RD = build_rd(raw);
        prof = mean(RD(:, 58:70), 2);          % Doppler=0 附近取平均（静止目标）
        prof(1:12) = 0;                         % 去掉泄漏区
        [~, bi] = max(prof);
        bins(end+1) = bi; %#ok<AGROW>
    end
    if isempty(bins) || isempty(vd), continue; end
    rbin = median(bins);
    rrange = (rbin - 1) * dR;
    fprintf('%-34s %8.2f %8.0f %8.2f %+8.2f\n', names{i}, median(vd), rbin, rrange, rrange - median(vd));
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

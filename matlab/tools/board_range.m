function board_range(root, names)
%BOARD_RANGE 只测板子：深度取板子 ROI 的近端，雷达取距离像在板子附近的峰
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

fprintf('\n%-30s %8s %8s %8s %8s %8s\n', 'take', '深度板m', '雷达峰bin', '雷达m', '雷达-深度', '深度-标注');
fprintf('%s\n', repmat('-', 1, 84));

for i = 1:numel(names)
    td = fullfile(root, names{i});
    if ~isfolder(td), continue; end
    tape = parse_tape(names{i});

    % ---- 深度：板子 ROI（画面下半、中央偏左）----
    dfiles = dir(fullfile(td, 'depth', '*.png'));
    pick = dfiles(round(linspace(1, numel(dfiles), 5)));
    vd = [];
    for j = 1:numel(pick)
        D = imread(fullfile(td, 'depth', pick(j).name));
        if size(D,3) > 1, D = D(:,:,1); end
        Z = double(D) * 0.001;
        roi = Z(300:470, 150:340);
        roi = roi(roi > 0.5 & roi < 5);
        if isempty(roi), continue; end
        vd(end+1) = median(roi); %#ok<AGROW>
    end

    % ---- 雷达：距离像（Doppler=0 附近）在 0.8~5 m 内找最大峰 ----
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
        prof = mean(RD(:, 58:70), 2);
        prof(1:12) = 0;
        lo = round(0.8 / dR) + 1; hi = min(numel(prof), round(5.0 / dR) + 1);
        [~, idx] = max(prof(lo:hi));
        bins(end+1) = lo + idx - 1; %#ok<AGROW>
    end
    rbin = median(bins);
    rrange = (rbin - 1) * dR;
    fprintf('%-30s %8.2f %8.0f %8.2f %+8.2f %+8.2f\n', ...
        names{i}, median(vd), rbin, rrange, rrange - median(vd), median(vd) - tape);
end
end

% ============================================================
function D = parse_tape(nm)
D = NaN;
if contains(nm, '2米5'), D = 2.5;
elseif contains(nm, '3米'), D = 3.0;
elseif contains(nm, '2米'), D = 2.0;
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

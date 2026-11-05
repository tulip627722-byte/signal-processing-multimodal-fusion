function [points, info] = process_final(rawFrame, cfg)
%PROCESS_FINAL 移植版的单帧算法链（无 MEX，全部可用 C 重写）。
%
% 分工（这三行是整条链的骨架）：
%   8T8R 64 通道  -> 检测（CFAR）、测角（近场 Bartlett）、强度、速度
%   1T8R  8 通道  -> 门限参考（多普勒增益高 9 dB，谱更干净）
%   detectMode='1T' 时，1T8R 还负责出候选（师兄那套 1T 索引 + 8T 测角）
%
% 输出 points: N×6 [X Y Z 强度dB 距离 速度]
%      info  : 各阶段计数、耗时、以及逐候选的 (kR,kD,az,el)

if nargin < 2, cfg = struct(); end
cfg = setdef(cfg);

Nadc = 1024; Nrx = 8; Ntx = 8; NchirpPerTx = 16; Ndop = 128;
NfftR = cfg.nfftR;

if numel(rawFrame) ~= cfg.payloadBytes*cfg.packetsPerFrame
    error('帧长不对：%d', numel(rawFrame));
end
rawFrame = reshape(uint8(rawFrame), cfg.payloadBytes, cfg.packetsPerFrame);
adc = double(uint16(rawFrame(1:2:end,:))) + 256*double(uint16(rawFrame(2:2:end,:))) - 2048;

wR  = single(hamming(Nadc));
wD  = single(hamming(NchirpPerTx)).';
wD1 = single(hamming(Ndop)).';

% ---- 波形参数：直接取固件 mmw_message.c 的 chirpCalStr ----
%   .startFreq=77.000 GHz, .fm=4000 MHz, .chirpT0=2us, .chirpT1=60us,
%   .chirpT2=5us, .chirpT3=0, .adcSampleFreq=16.67 MHz, .adcSamplePoint=1024
c = 3e8;
Fs    = 16.667e6;        % ADC 采样率
fm    = 4e9;             % 扫频带宽 4000 MHz
T0 = 2e-6; T1 = 60e-6; T2 = 5e-6; T3 = 0;
Tchirp = T0 + T1 + T2 + T3;   % 67 us，一个 chirp 的周期
Tramp  = T1;                  % 60 us，真正的上升沿（斜率用这个）
slope = fm / Tramp;
dR = (c/(2*slope))*(Fs/NfftR);   % 纯固件参数，不加任何经验系数
kRStart = NfftR/128 + 1;
lambda = c/77e9;
dv8T = lambda/(2*Ndop*8*Tchirp);   % 速度用 chirp 周期，不是上升沿

% ---------------- 1T8R：8 个 RX 非相干累加 ----------------
adc1 = permute(reshape(adc(:,1:1024), Nadc, Ndop, Nrx), [1 3 2]);
adc1 = single(adc1);
X1 = fft(adc1 .* reshape(wR, Nadc, 1, 1), NfftR, 1);
X1 = X1(1:NfftR/2, :, :);
X1 = fft(X1 .* reshape(wD1, 1, 1, Ndop), Ndop, 3);
RDc = abs(X1);
RDsum1T = reshape(sum(RDc, 2), size(RDc,1), size(RDc,3));   % 沿 RX 累加
assert(size(RDsum1T,2) == Ndop, '1T8R 多普勒轴不对');
clear adc1 X1 RDc

% ---------------- 8T8R：64 通道复数 RD ----------------
adc8 = reshape(adc(:,1025:2048), Nadc, Ntx, NchirpPerTx, Nrx);
adc8 = permute(adc8, [1,4,2,3]);
adc8 = reshape(adc8, Nadc, Nrx*Ntx, NchirpPerTx);
adc8 = single(adc8);
rangeFft = fft(adc8 .* reshape(wR, Nadc, 1, 1), NfftR, 1);
rangeFft = rangeFft(1:NfftR/2, :, :);
rd = fft(rangeFft .* reshape(wD, 1, 1, NchirpPerTx), Ndop, 3);
rd = rd(:, :, [65:128, 1:64]);
RDcube = permute(rd, [1 3 2]);       % (Nrange, Ndop, 64)
assert(size(RDcube,2) == Ndop, '8T8R 多普勒轴不对');
RDsum8T = sum(abs(RDcube), 3);
clear adc adc8 rangeFft rd

% ---------------- 检测 ----------------
tDet = tic;
switch upper(cfg.detectMode)
    case '8T'
        mask = cfar_dispatch(double(RDsum8T), cfg);
        [kR, kD] = find(mask);
        amp = RDsum8T(sub2ind(size(RDsum8T), kR, kD));
        nCfar = numel(kR);
    case '1T'
        mask1 = cfar_dispatch(double(RDsum1T), cfg);
        [kr1, kd1] = find(mask1);
        nCfar = numel(kr1);
        kR = zeros(0,1); kD = zeros(0,1); amp = zeros(0,1);
        for i = 1:numel(kr1)
            r0 = max(1,kr1(i)-cfg.assocRangeBins) : min(size(RDsum8T,1), kr1(i)+cfg.assocRangeBins);
            d0 = mod((kd1(i)-1) + (-cfg.assocDopBins:cfg.assocDopBins), Ndop) + 1;
            [m, idx] = max(RDsum8T(r0,d0), [], 'all');
            if m <= 0, continue; end
            [ri, di] = ind2sub([numel(r0), numel(d0)], idx);
            kR(end+1,1) = r0(ri); kD(end+1,1) = d0(di); amp(end+1,1) = m; %#ok<AGROW>
        end
    otherwise
        error('detectMode 未知：%s', cfg.detectMode);
end
detMs = toc(tDet)*1000;

% ---------------- 门限（1T8R 当参考）----------------
tGate = tic;
prof = max(RDsum1T, [], 2);
base = conv(prof, ones(65,1)/65, 'same');
keep = prof(kR) >= single(cfg.relativeGateRatio) * base(kR);
kR = kR(keep); kD = kD(keep); amp = amp(keep);
nGate = numel(kR);

% 切掉最前面几个距离 bin（全是泄漏），再按强度截断
keep2 = kR >= kRStart;
kR = kR(keep2); kD = kD(keep2); amp = amp(keep2);
if numel(kR) > cfg.maxDetections
    [~,ord] = sort(amp,'descend');
    sel = ord(1:cfg.maxDetections);
    kR = kR(sel); kD = kD(sel); amp = amp(sel);
end
gateMs = toc(tGate)*1000;

% ---------------- 测角（三种实现可切）----------------
%   'ours'      本目录的自写近场 Bartlett（默认，移植目标）
%   'mex'       原厂 MEX（无源码，仅作对照）
%   'bartlett2' 2.0 师兄的批量近场 Bartlett（仅作对照）
tAng = tic;
nCand = numel(kR);
azAll = nan(nCand,1); elAll = nan(nCand,1);
velAll = (kD - 65) * dv8T;

switch lower(cfg.angleMode)
    case 'ours'
        for n = 1:nCand
            v64 = reshape(double(squeeze(RDcube(kR(n), kD(n), :))), 1, []);
            [azAll(n), elAll(n)] = angle_nf_bartlett(v64, (kR(n)-1)*dR, velAll(n), cfg.angleCfg);
        end
    case 'mex'
        RxCorr = exp(1i*[0, 313, 221, 21, 158, 346, 225, 241]*pi/180);
        TxCorr = exp(1i*[48, 73, 193, 178, 0, 212, 320, 307]*pi/180);
        for n = 1:nCand
            kd = kD(n);
            v64 = reshape(double(squeeze(RDcube(kR(n), kd, :))), 1, []);
            k2 = round((kd-1)/max(Ndop-1,1)*1023) + 1;
            [sinLR,~] = mex_getSinLR_8T8R(v64, TxCorr, RxCorr, k2);
            [sinUD,~] = mex_getSinUD_8T8R(v64, TxCorr, RxCorr, k2, sinLR);
            azAll(n) = -asind(max(-1,min(1,(double(sinLR)-513)/512)));
            elAll(n) =  asind(max(-1,min(1,(double(sinUD)-513)/512)));
        end
    case 'bartlett2'
        ar = estimate_fast_angles_bartlett(struct('RDcube64', RDcube), ...
            kR, kD, velAll, cfg.radar2, cfg.angle2, (kR-1)*dR);
        azAll = ar.azimuthDeg; elAll = ar.elevationDeg;
    otherwise
        error('angleMode 未知：%s', cfg.angleMode);
end

% 雷达测量偏差和机械外参分开处理。offset 是直接加到测量值上的量；
% 例如测得方位系统性偏右 3.2 deg 时，azimuthOffsetDeg 应设为 -3.2。
azRaw = azAll;
elRaw = elAll;
azAll = azAll + cfg.azimuthOffsetDeg;
elAll = elAll + cfg.elevationOffsetDeg;
angMs = toc(tAng)*1000;

% ---------------- ROI → 点云 ----------------
out = zeros(nCand, 6); keepP = false(nCand,1);
for n = 1:nCand
    if ~isfinite(azAll(n)) || ~isfinite(elAll(n)), continue; end
    rng = max((kR(n)-1)*dR + cfg.rangeOffsetM, 0);
    X = rng*cosd(elAll(n))*sind(azAll(n));
    Y = rng*cosd(elAll(n))*cosd(azAll(n));
    Z = rng*sind(elAll(n));
    if X >= cfg.xLim(1) && X <= cfg.xLim(2) && ...
       Y >= cfg.yLim(1) && Y <= cfg.yLim(2) && ...
       Z >= cfg.zLim(1) && Z <= cfg.zLim(2)
        out(n,:) = [X, Y, Z, 20*log10(RDsum8T(kR(n),kD(n))+1e-12), rng, velAll(n)];
        keepP(n) = true;
    end
end
points = out(keepP,:);
nRoi = size(points,1);

if cfg.minSpeed > 0 && ~isempty(points)
    points = points(abs(points(:,6)) >= cfg.minSpeed, :);
end

info = struct('cfar', nCfar, 'gate', nGate, 'cand', nCand, 'roi', nRoi, ...
              'final', size(points,1), ...
              'msDetect', detMs, 'msGate', gateMs, 'msAngle', angMs, ...
              'total', detMs + gateMs + angMs, ...
              'candKR', kR, 'candKD', kD, ...
              'candAzRaw', azRaw, 'candElRaw', elRaw, ...
              'candAz', azAll, 'candEl', elAll, ...
              'rangeOffsetM', cfg.rangeOffsetM, ...
              'azimuthOffsetDeg', cfg.azimuthOffsetDeg, ...
              'elevationOffsetDeg', cfg.elevationOffsetDeg, ...
              'candAmp', amp);
end

% ============================================================
function cfg = setdef(cfg)
d = struct( ...
    'payloadBytes', 2048, 'packetsPerFrame', 2048, ...
    'nfftR', 1024, 'maxDetections', 800, ...
    'gR', 18, 'gD', 6, 'tR', 6, 'tD', 2, ...
    'detectMode', '8T', 'cfarThreshold', 1.53, ...
    'relativeGateRatio', 0.5, 'cfarSource', 'local', 'cfarBias', 1.0, ...
    'rangeOffsetM', 0, ...
    'azimuthOffsetDeg', 0, 'elevationOffsetDeg', 0, ...
    'assocRangeBins', 2, 'assocDopBins', 4, ...
    'minSpeed', 0, 'angleCfg', struct(), ...
    'angleMode', 'ours', 'radar2', [], 'angle2', [], ...
    'xLim', [-3 3], 'yLim', [1 8], 'zLim', [-1 1.5]);
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(cfg, f{i}), cfg.(f{i}) = d.(f{i}); end
end
end

% ============================================================
function mask = cfar_dispatch(RD, cfg)
%CFAR_DISPATCH 按 cfg.cfarSource 选实现：local（自写）/ mex（原厂，仅供对照）
if isfield(cfg, 'cfarSource') && strcmpi(cfg.cfarSource, 'mex')
    mask = mex_RD_CACFAR(RD, cfg.gR, cfg.gD, cfg.tR, cfg.tD, cfg.cfarThreshold);
else
    bias = 1.0;
    if isfield(cfg, 'cfarBias'), bias = cfg.cfarBias; end
    mask = cfar_ca(RD, cfg.gR, cfg.gD, cfg.tR, cfg.tD, cfg.cfarThreshold, bias);
end
end

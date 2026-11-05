function [azDeg, elDeg, peakDb] = angle_nf_bartlett(v64, range_m, vel_mps, cfg)
%ANGLE_NF_BARTLETT 近场二维 Bartlett（带距离分箱缓存），移植版测角核心。
%
%   [az, el, pk] = angle_nf_bartlett(v64, range_m, vel_mps, cfg)
%
% 与 compare\03_参数微调\angle_fft_bartlett.m 的区别：
%   · 导向矩阵按“距离箱”缓存（默认 5 cm 一格），同一帧里几百个候选通常
%     只落在 50~100 个箱上，不用每个候选重建一次
%   · 粗网格 4°、细网格 0.5°，方向数从 915 降到 248，精度几乎不变
%   · 符号约定已由 sweep_angle_signs 实测定下：cal=+1, steer=-1, 近场开启
%
% 移植到 C 时的对应关系：
%   1. C.combine（64→61 合并）、C.phaseCal（相位校准）都是常量表，烧 ROM
%   2. 导向相位 (2π/λ)(d_i - R) 用查表或 CORDIC，不用 exp
%   3. 每个距离箱的导向矩阵可预生成进 SDRAM 缓存，或用递推逐点算

if nargin < 4, cfg = struct(); end
cfg = setdef(cfg);

persistent C
if isempty(C) || C.lambda ~= cfg.lambda || C.binSize ~= cfg.binSize || C.coarseStep ~= cfg.coarseStep
    C = build_ctx(cfg);
end

% ---- 64 通道预处理：相位校准 + TDM 运动相位补偿 ----
v = reshape(v64, 1, []);
tdm = exp(1i*cfg.tdmSign*2*pi*(2*vel_mps/cfg.lambda)*C.txOffset);
v61 = C.combine * (v .* C.phaseCal .* tdm).';

% ---- 按距离箱取缓存的粗导向矩阵 ----
key = max(1, round(range_m / cfg.binSize));
idx = C.keys == key;
if any(idx)
    Ac = C.cache{idx};
else
    Ac = nf_steering(C, C.azC, C.elC, range_m, cfg.lambda, cfg.steerSign);
    if numel(C.cache) < cfg.maxCacheBins
        C.cache{end+1} = Ac;
        C.keys(end+1)  = key;
    end
end

s = abs(Ac' * v61).^2;
[peakVal, ci] = max(s);
peakDb = 10*log10(peakVal / (median(s) + eps));

az0 = C.azC(ci);
el0 = C.elC(ci);

% ---- 细搜：粗峰附近 ±2°，步进 0.5° ----
fa = az0 + (-2:0.5:2);
fe = el0 + (-2:0.5:2);
fe(fe < -14 | fe > 14) = [];
[FA, FE] = meshgrid(fa, fe);
azD = FA(:); elD = FE(:);
Af = nf_steering(C, azD, elD, range_m, cfg.lambda, cfg.steerSign);
sf = abs(Af' * v61).^2;
[~, fi] = max(sf);
azDeg = azD(fi);
elDeg = elD(fi);
end

% ============================================================
function cfg = setdef(cfg)
d = struct('lambda', 3e8/77e9, 'T', 67e-6, ...
           'tdmSign', -1, 'steerSign', -1, ...
           'binSize', 0.05, 'coarseStep', 2, 'maxCacheBins', 400);
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(cfg, f{i}), cfg.(f{i}) = d.(f{i}); end
end
end

% ============================================================
function C = build_ctx(cfg)
% 8 个发射在 X/Z 上错开（mm），8 个接收沿 X 排一列
rxX = [0,-4,-8,-12,-16,-19,-21,-24];  rxZ = 0;
txX = [36,27,18,9,0,-24,-48,-72];
txZ = [32,24,16,8,0,0,0,0];
RxCorrDeg = [0, 313, 221, 21, 158, 346, 225, 241];
TxCorrDeg = [48, 73, 193, 178, 0, 212, 320, 307];

px = zeros(1,64); pz = zeros(1,64);
txIdx = zeros(1,64); rxIdx = zeros(1,64);
for tx = 1:8
    for rx = 1:8
        ch = (tx-1)*8 + rx;
        px(ch) = txX(tx) + rxX(rx);
        pz(ch) = txZ(tx) + rxZ;
        txIdx(ch) = tx; rxIdx(ch) = rx;
    end
end

C.phaseCal = exp(1i*deg2rad(TxCorrDeg(txIdx))) .* exp(1i*deg2rad(RxCorrDeg(rxIdx)));
C.txOffset = (txIdx - 1) * cfg.T;

% 合并重复虚拟阵元（64 → 61）
[~, ~, map] = unique([round(px(:)), round(pz(:))], 'rows');
n = max(map);
C.posX = zeros(1,n); C.posZ = zeros(1,n);
for k = 1:64
    C.posX(map(k)) = px(k)*1e-3;
    C.posZ(map(k)) = pz(k)*1e-3;
end
C.combine = zeros(n,64);
for k = 1:64
    C.combine(map(k),k) = 1;
end

% 粗网格
az = -60:cfg.coarseStep:60;
el = -14:cfg.coarseStep:14;
[AA,EE] = meshgrid(az,el);
C.azC = AA(:).'; C.elC = EE(:).';

C.cache = {};
C.keys  = [];
C.lambda = cfg.lambda;
C.binSize = cfg.binSize;
C.coarseStep = cfg.coarseStep;
end

% ============================================================
function A = nf_steering(C, azList, elList, R, lambda, signSteer)
% 近场导向：用 Fresnel 近似代替 sqrt，速度快很多，精度在 1~8 m 内够用：
%   d_i - R ≈ -(u·p_i) + (|p_i|^2 - (u·p_i)^2) / (2R)
R = max(R, 0.3);
az = deg2rad(azList(:).');
el = deg2rad(elList(:).');
u = sin(az).*cos(el);          % 目标方向 X 分量
w = sin(el);                   % 目标方向 Z 分量
pp = (C.posX(:).^2 + C.posZ(:).^2);         % 阵元自身半径平方（常量）
up = C.posX(:)*u + C.posZ(:)*w;             % u·p，n×D
q  = (pp - up.^2) / (2*R);                  % Fresnel 修正项
ph = signSteer * (2*pi/lambda) * (-up + q);
A  = exp(1i*ph);
end

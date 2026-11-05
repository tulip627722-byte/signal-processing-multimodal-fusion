function P = range_params()
%RANGE_PARAMS 只做距离维 FFT 需要的参数。全部显式，无经验系数。
%
% 数值来源：algorithm\process_final.m（3.2 版，波形参数取自固件 mmw_message.c
% 的 chirpCalStr）。单独拎出来，是为了让 dR 只由这几个数字决定。

c = 3e8;

% ---- 数据打包 ----
P.Nadc         = 1024;
P.Nrx          = 8;
P.Ntx          = 8;
P.NchirpPerTx  = 16;
P.Ndop         = 128;    % 1T8R 那一半的 chirp 数
P.NfftR        = 1024;
P.per          = 2048;   % 每帧包数
P.payloadBytes = 2048;

% ---- 固件 chirpCalStr ----
P.fm = 4e9;              % .fm = 4000 MHz
P.T1 = 60e-6;            % .chirpT1，真正的上升沿
P.Tramp = P.T1;
P.Fs = 16.667e6;         % .adcSampleFreq = 250 MHz / 15

% ---- 派生量 ----
P.slope = P.fm / P.Tramp;
P.dR    = c * P.Fs / (2 * P.slope * P.NfftR);   % ≈ 0.0366 m/bin
P.kRStart = P.NfftR/128 + 1;                    % 前 8 bin 是泄漏（与主链一致）

% ---- 帧内布局：前 1024 包 1T8R，后 1024 包 8T8R ----
P.col1T = 1:1024;
P.col8T = 1025:2048;
end

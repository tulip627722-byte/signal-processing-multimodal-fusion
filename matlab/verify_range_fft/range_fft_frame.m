function [rangeFft, prof] = range_fft_frame(raw, P, mode)
%RANGE_FFT_FRAME 单帧，只做距离维 FFT。到这里就停。
%
% 与 algorithm\process_final.m 里这两句完全同一条：
%     rangeFft = fft(adc8 .* reshape(wR, Nadc, 1, 1), NfftR, 1);
%     rangeFft = rangeFft(1:NfftR/2, :, :);
% 去掉的只有后面那一层：多普勒 FFT、CFAR、门限、测角、ROI。
%
%   [rangeFft, prof] = range_fft_frame(raw, range_params())        % 8T8R（默认）
%   [rangeFft, prof] = range_fft_frame(raw, P, '1T8R')
%
%   rangeFft  (NfftR/2) × Nch × Nchirp 复数谱
%   prof      sum(|rangeFft|) 沿通道与 chirp，(NfftR/2)×1 距离像

if nargin < 3 || isempty(mode), mode = '8T8R'; end

raw = reshape(uint8(raw), P.payloadBytes, P.per);
adc = double(uint16(raw(1:2:end, :))) + 256*double(uint16(raw(2:2:end, :))) - 2048;

wR = single(hamming(P.Nadc));   % algorithm\lib\hamming.m，与主链同一个窗

switch upper(mode)
    case '8T8R'
        adc8 = reshape(adc(:, P.col8T), P.Nadc, P.Ntx, P.NchirpPerTx, P.Nrx);
        adc8 = permute(adc8, [1, 4, 2, 3]);
        adc8 = reshape(adc8, P.Nadc, P.Nrx*P.Ntx, P.NchirpPerTx);
        adc8 = single(adc8);
    case '1T8R'
        adc8 = permute(reshape(adc(:, P.col1T), P.Nadc, P.Ndop, P.Nrx), [1 3 2]);
        adc8 = single(adc8);
    otherwise
        error('mode 未知：%s', mode);
end

rangeFft = fft(adc8 .* reshape(wR, P.Nadc, 1, 1), P.NfftR, 1);
rangeFft = rangeFft(1:P.NfftR/2, :, :);

prof = double(sum(abs(rangeFft), [2 3]));
prof = prof(:);
end

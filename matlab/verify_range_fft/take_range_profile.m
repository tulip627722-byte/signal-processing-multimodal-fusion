function [prof, frames] = take_range_profile(takeDir, P, nFrames, mode)
%TAKE_RANGE_PROFILE 一个 take 的平均距离像（只经过距离维 FFT）。
%
%   prof = take_range_profile(takeDir, range_params(), 4)
%   [prof, frames] = take_range_profile(takeDir, P, 1, '1T8R')
%
% 取 nFrames 个均匀分布的完整帧，逐帧算距离像再平均（nFrames = 1 就是单帧）。
% 拼包/切帧逻辑与 compute_take.m 一致。

if nargin < 3 || isempty(nFrames), nFrames = 4; end
if nargin < 4 || isempty(mode), mode = '8T8R'; end
nFrames = max(1, round(nFrames));

S = load(fullfile(takeDir, 'radar_adc.mat'), 'ADCData');
adt = S.ADCData;

ids = zeros(size(adt, 1), 1);
for i = 1:size(adt, 1)
    v = adt{i, 2}; if iscell(v) && numel(v) == 1, v = v{1}; end
    ids(i) = double(v(1));
end
[sIds, sIdx] = sort(ids);
fidx = floor(sIds / P.per) + 1;
[uf, ~, grp] = unique(fidx);
cnt = accumarray(grp, 1);
frames = sort(uf(cnt == P.per));
if isempty(frames), error('没有完整帧：%s', takeDir); end
if numel(frames) > nFrames
    frames = frames(round(linspace(1, numel(frames), nFrames)));
end

acc = zeros(P.NfftR/2, 1);
for k = 1:numel(frames)
    lo = (frames(k) - 1) * P.per;
    p1 = find(sIds >= lo, 1, 'first');
    raw = zeros(P.payloadBytes, P.per, 'uint8');
    for i = 1:P.per
        raw(:, i) = uint8(adt{sIdx(p1 + i - 1), 4}(:));
    end
    [~, pr] = range_fft_frame(raw, P, mode);
    acc = acc + pr;
end
prof = acc / numel(frames);
end

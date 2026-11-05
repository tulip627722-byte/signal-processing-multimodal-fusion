function [rngM, dB, prof, pk] = plot_range_power(opts)
%PLOT_RANGE_POWER 距离维 FFT 之后的距离-功率图（顶部标拍频），峰按"局部极大值"挑。
%
%   plot_range_power()                                    % 2 米那组，4 帧平均，0-6 m
%   plot_range_power(struct('nFrames',1))                 % 单帧
%   plot_range_power(struct('takeDir','D:\...\室内3米_1'))
%   plot_range_power(struct('peakRange',[1.5 3.5]))       % 只看 1.5–3.5 m
%   plot_range_power(struct('peakRange',[1.5 3.5],'peakHalf',5))
%   plot_range_power(struct('savePath','D:\x.png'))
%   plot_range_power(struct('ax',ax))                     % 画到指定 axes（GUI 用）
%
% 峰的判据是局部极大值，不是区间最高点：
%   prof(k) > prof(k-1..k-hw)  且  prof(k) > prof(k+1..k+hw)
% 也就是"前 hw 个 bin 都比我低、后 hw 个 bin 都比我低"。hw = peakHalf（默认 5）。
% 区间里可能同时有几个这样的点，图上全部用橙点标出，其中最强者用红三角 + 文字。
%
% peakRange 缺省时 = [第 8 个 bin 的距离, xMax]，也就是跳过泄漏段。

if nargin < 1 || isempty(opts), opts = struct(); end
o = setdef(opts);

P = range_params();
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'algorithm', 'lib'), '-begin');
% ↑ hamming 用 algorithm\lib\hamming.m，和 process_final 同一个文件

prof = take_range_profile(o.takeDir, P, o.nFrames, o.mode);

k = (0:(P.NfftR/2 - 1)).';
rngM = k * P.dR;                          % 距离（m）
freqKHz = k * (P.Fs / P.NfftR) / 1e3;     % 拍频（kHz）
dB = 20*log10(max(prof, 1e-12) / max(prof));

% ---- 选峰窗口 ----
leadR = (P.kRStart - 1) * P.dR;
pr = o.peakRange;
if isempty(pr)
    pr = [leadR, o.xMax];
end
pr = sort(pr(:).');
pr = [max(pr(1), 0), min(pr(2), rngM(end))];

% ---- 局部极大值：前后各 hw 个 bin 都比它低 ----
hw = max(1, round(o.peakHalf));
n = numel(prof);
isPk = true(n, 1);
for d = 1:hw
    isPk(1+d:end) = isPk(1+d:end) & prof(1+d:end) > prof(1:end-d);
    isPk(1:end-d) = isPk(1:end-d) & prof(1:end-d) > prof(1+d:end);
end
cand = find(isPk & rngM >= pr(1) & rngM <= pr(2));

bi = [];
if isempty(cand)
    warning('选峰区间 %.2f–%.2f m 里没有 ±%d bin 的局部极大值', pr(1), pr(2), hw);
else
    [~, kk] = max(prof(cand));
    bi = cand(kk);              % 几个极大值里最强的那个当主峰
end

% ---- 画在哪 ----
if isempty(o.ax)
    fig = figure('Color', 'w', 'Name', 'range FFT power', 'Position', [100 100 900 470]);
    ax1 = axes('Parent', fig, 'Position', [0.10 0.14 0.80 0.70]);
else
    ax1 = o.ax;
    fig = ancestor(ax1, 'figure');
    cla(ax1);
    delete(findobj(fig, 'Type', 'axes', 'Tag', 'freqAxis'));
end

xHi = max(o.xMax, pr(2));
plot(ax1, rngM, dB, 'LineWidth', 1.1, 'Color', [0.12 0.35 0.65]);
grid(ax1, 'on'); box(ax1, 'on');
xlim(ax1, [0, xHi]); ylim(ax1, [-70, 3]);
xlabel(ax1, '距离 (m)');
ylabel(ax1, '功率 (dB，相对最大值)');
if isempty(o.ax)
    % 单独成图时才写标题；画进 GUI 的 axes 时不写，否则会盖住上面的控件
    title(ax1, sprintf('%s  |  %s  %d 帧平均  |  dR = %.4f m/bin', ...
        shortname(o.takeDir), o.mode, o.nFrames, P.dR), 'Interpreter', 'none');
    ax1.Title.Units = 'normalized';
    ax1.Title.Position = [0.5, 1.12, 0];
end

% 顶部拍频副轴
kHzPerM = (P.Fs / P.NfftR) / 1e3 / P.dR;
ax2 = axes('Parent', fig, 'Position', ax1.Position, 'Color', 'none', ...
           'XAxisLocation', 'top', 'YColor', 'none', 'Tag', 'freqAxis');
xlim(ax2, [0, xHi] * kHzPerM);
xlabel(ax2, '拍频 (kHz)');
ax2.XLabel.FontSize = 9;
ax2.XLabel.Units = 'normalized';
ax2.XLabel.Position = [0.5, 1.05, 0];

% ---- 选峰窗口（浅灰底）+ 泄漏边界 + 极大值点 ----
hold(ax1, 'on');
yl = ylim(ax1);
hBand = patch(ax1, [pr(1) pr(2) pr(2) pr(1)], ...
    [yl(1) yl(1) yl(2) yl(2)], [0.90 0.93 0.99], ...
    'EdgeColor', [0.45 0.55 0.75], 'LineStyle', '-', 'LineWidth', 0.5, ...
    'FaceAlpha', 0.85, 'HandleVisibility', 'off');
uistack(hBand, 'bottom');
xline(ax1, leadR, ':', 'Color', [0.55 0.55 0.55]);

if ~isempty(cand)
    plot(ax1, rngM(cand), dB(cand), 'o', 'LineStyle', 'none', ...
         'MarkerSize', 4, 'MarkerEdgeColor', [0.90 0.55 0.10], 'LineWidth', 1);
    plot(ax1, rngM(bi), dB(bi), 'v', 'MarkerFaceColor', [0.85 0.2 0.2], ...
         'MarkerEdgeColor', 'none', 'MarkerSize', 7);
    % 标注避开图边：靠左往右放，靠右往左放
    dx = 0.035 * xHi;
    if rngM(bi) < 0.15 * xHi
        ha = 'left';   tx = rngM(bi) + dx;
    elseif rngM(bi) > 0.85 * xHi
        ha = 'right';  tx = rngM(bi) - dx;
    else
        ha = 'center'; tx = rngM(bi);
    end
    ty = min(dB(bi) + 2.5, 1.0);
    text(ax1, tx, ty, sprintf('bin %d  %.2f m', bi, rngM(bi)), ...
        'HorizontalAlignment', ha, 'FontSize', 8, 'Color', [0.6 0.1 0.1]);
else
    text(ax1, mean(pr), 0, sprintf('区间内没有 ±%d bin 的极大值', hw), ...
        'HorizontalAlignment', 'center', 'FontSize', 9, 'Color', [0.7 0.2 0.2]);
end
hold(ax1, 'off');

% ---- 输出 ----
pk = struct('bin', bi, 'rng', NaN, 'kHz', NaN, 'dB', NaN, ...
            'range', pr, 'half', hw, 'cand', cand);
if ~isempty(bi)
    pk.rng = rngM(bi); pk.kHz = freqKHz(bi); pk.dB = dB(bi);
end

fprintf('选峰 %.2f–%.2f m，邻域 ±%d bin\n', pr(1), pr(2), hw);
if isempty(cand)
    fprintf('  区间内没有局部极大值\n');
else
    [~, ord] = sort(prof(cand), 'descend');
    m = min(5, numel(ord));
    for i = 1:m
        c = cand(ord(i));
        mark = ''; if c == bi, mark = '  ← 主峰'; end
        fprintf('  bin %3d  %6.2f m  %7.0f kHz  %6.1f dB%s\n', ...
            c, rngM(c), freqKHz(c), dB(c), mark);
    end
    if numel(cand) > m
        fprintf('  ……共 %d 个极大值\n', numel(cand));
    end
end
fprintf('dR = %.4f m/bin，%.2f kHz/bin，每米 %.1f kHz\n', ...
    P.dR, P.Fs/P.NfftR/1e3, kHzPerM);

if ~isempty(o.savePath)
    exportgraphics(fig, o.savePath, 'Resolution', 160);
    fprintf('图已存：%s\n', o.savePath);
end
end

% ============================================================
function s = shortname(p)
[~, s] = fileparts(p);
end

% ============================================================
function o = setdef(o)
d = struct();
d.takeDir   = 'D:\7880\Realtime\capture\takes\11-4\20261104_133930_室内2米_1';
d.nFrames   = 4;
d.mode      = '8T8R';
d.xMax      = 6;
d.peakRange = [];       % [] = [泄漏边界, xMax]
d.peakHalf  = 5;        % 前后各 5 个 bin 都更低才算极大值
d.savePath  = '';
d.ax        = [];
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(o, f{i}), o.(f{i}) = d.(f{i}); end
end
end

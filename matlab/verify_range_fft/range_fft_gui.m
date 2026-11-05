function fig = range_fft_gui(root)
%RANGE_FFT_GUI 弹窗：选一个采集 take、选帧数，看距离维 FFT 之后的距离-功率图。
%
%   range_fft_gui
%   range_fft_gui('D:\7880\Realtime\capture\takes\11-4')
%
% 操作：
%   采集目录   填 take 的父目录，或者点「浏览…」选
%   take       下拉列表里选某一次采集（按时间新→旧排）
%   帧数       1 = 单帧；n = 均匀取 n 帧平均（默认 4）
%   模式       8T8R（64 通道）/ 1T8R（8 通道）
%   最远距离   图横轴上限（米）
%   画图       重算重画；回车也可以
%   存图       导出 PNG
%
% 只做距离维 FFT，多普勒/CFAR/测角都不参与。

if nargin < 1 || isempty(root)
    root = 'D:\7880\Realtime\capture\takes\11-4';
end

st = struct();
st.root = root;
st.nFrames = 4;
st.mode = '8T8R';
st.xMax = 6;
st.peakLo = 0.5;    % 选峰区间下限（m）
st.peakHi = 6;      % 选峰区间上限（m）
st.peakHalf = 5;    % 局部极大值邻域（前后各几个 bin）
st.takes = {};

fig = figure('Color', 'w', 'Units', 'normalized', ...
    'Position', [0.12 0.12 0.76 0.76], 'NumberTitle', 'off', ...
    'Name', '距离维 FFT — 距离/功率', 'MenuBar', 'none', 'ToolBar', 'none');

% ---------- 第一行：目录 + 浏览 ----------
uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.03 0.935 0.07 0.045], 'String', '采集目录', ...
    'HorizontalAlignment', 'left', 'FontSize', 10);
h.edRoot = uicontrol(fig, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.105 0.935 0.67 0.05], 'String', root, ...
    'HorizontalAlignment', 'left', 'FontSize', 9, 'Callback', @(~, ~) onRoot());
uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.785 0.935 0.075 0.05], 'String', '浏览…', ...
    'FontSize', 10, 'Callback', @(~, ~) onBrowse());
uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.87 0.935 0.075 0.05], 'String', '刷新', ...
    'FontSize', 10, 'Callback', @(~, ~) refresh_takes());

% ---------- 第二行：take + 帧数 ----------
uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.03 0.865 0.07 0.045], 'String', 'take', ...
    'HorizontalAlignment', 'left', 'FontSize', 10);
h.popTake = uicontrol(fig, 'Style', 'popupmenu', 'Units', 'normalized', ...
    'Position', [0.105 0.865 0.53 0.05], 'String', {'（空）'}, ...
    'HorizontalAlignment', 'left', 'FontSize', 9, 'Value', 1);

uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.655 0.865 0.045 0.045], 'String', '帧数', ...
    'HorizontalAlignment', 'left', 'FontSize', 10);
h.edFrames = uicontrol(fig, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.705 0.865 0.05 0.05], 'String', num2str(st.nFrames), ...
    'FontSize', 10, 'Callback', @(~, ~) draw_now());
uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.765 0.867 0.22 0.042], 'String', '1 = 单帧，n = 平均', ...
    'HorizontalAlignment', 'left', 'FontSize', 9, 'ForegroundColor', [0.4 0.4 0.4]);

% ---------- 第三行：模式 + 最远 + 按钮 ----------
uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.03 0.795 0.05 0.045], 'String', '模式', ...
    'HorizontalAlignment', 'left', 'FontSize', 10);
h.popMode = uicontrol(fig, 'Style', 'popupmenu', 'Units', 'normalized', ...
    'Position', [0.085 0.795 0.10 0.05], 'String', {'8T8R', '1T8R'}, ...
    'FontSize', 10, 'Value', 1);

uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.195 0.795 0.06 0.045], 'String', '最远(m)', ...
    'HorizontalAlignment', 'left', 'FontSize', 9);
h.edXMax = uicontrol(fig, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.258 0.795 0.045 0.05], 'String', num2str(st.xMax), ...
    'FontSize', 10, 'Callback', @(~, ~) draw_now());

% ---- 选峰区间 ----
uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.313 0.795 0.062 0.045], 'String', '峰区(m)', ...
    'HorizontalAlignment', 'left', 'FontSize', 9, 'FontWeight', 'bold');
h.edPeakLo = uicontrol(fig, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.378 0.795 0.048 0.05], 'String', sprintf('%.2f', st.peakLo), ...
    'FontSize', 10, 'Callback', @(~, ~) draw_now());
uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.428 0.795 0.015 0.045], 'String', '~', ...
    'HorizontalAlignment', 'center', 'FontSize', 10);
h.edPeakHi = uicontrol(fig, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.445 0.795 0.048 0.05], 'String', sprintf('%.2f', st.peakHi), ...
    'FontSize', 10, 'Callback', @(~, ~) draw_now());

uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.498 0.795 0.04 0.045], 'String', '邻域', ...
    'HorizontalAlignment', 'left', 'FontSize', 9);
h.edHalf = uicontrol(fig, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.538 0.795 0.04 0.05], 'String', num2str(st.peakHalf), ...
    'FontSize', 10, 'Callback', @(~, ~) draw_now());

uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.60 0.795 0.085 0.05], 'String', '画图', ...
    'FontSize', 11, 'FontWeight', 'bold', 'Callback', @(~, ~) draw_now());
uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.695 0.795 0.085 0.05], 'String', '存图', ...
    'FontSize', 10, 'Callback', @(~, ~) save_png());

% ---------- 图 ----------
h.ax = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.095 0.20 0.845 0.545]);
xlabel(h.ax, '距离 (m)'); ylabel(h.ax, '功率 (dB)');
title(h.ax, '选一个 take，点「画图」');
grid(h.ax, 'on'); box(h.ax, 'on');

% ---------- 状态栏 ----------
h.txt = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.032 0.035 0.93 0.13], ...
    'String', {'选一个采集目录和 take，点「画图」（回车也可以）。', ...
               '默认 4 帧平均；帧数填 1 就是单帧。'}, ...
    'HorizontalAlignment', 'left', 'FontSize', 10, ...
    'ForegroundColor', [0.15 0.25 0.45]);

set(fig, 'KeyPressFcn', @(~, ev) onKey(ev));

refresh_takes();
if ~isempty(st.takes)
    draw_now();
end

% ================================================================
    function refresh_takes()
        st.root = strtrim(get(h.edRoot, 'String'));
        st.takes = {};
        if ~isfolder(st.root)
            set(h.popTake, 'String', {'（目录不存在）'}, 'Value', 1);
            return;
        end
        list = dir(st.root);
        list = list([list.isdir]);
        list = list(~ismember({list.name}, {'.', '..'}));
        keep = false(numel(list), 1);
        for i = 1:numel(list)
            keep(i) = isfile(fullfile(st.root, list(i).name, 'radar_adc.mat'));
        end
        list = list(keep);
        [~, ord] = sort([list.datenum], 'descend');
        list = list(ord);
        if isempty(list)
            set(h.popTake, 'String', {'（没有含 radar_adc.mat 的子目录）'}, 'Value', 1);
            return;
        end
        st.takes = {list.name};
        set(h.popTake, 'String', st.takes, 'Value', 1);
        set(h.txt, 'String', sprintf('%s 下 %d 个采集，选一个再点「画图」。', ...
            st.root, numel(st.takes)));
    end

    function onBrowse()
        d = uigetdir(st.root, '选一个采集目录（含 radar_adc.mat 的那层，或它的父目录）');
        if isequal(d, 0), return; end
        set(h.edRoot, 'String', d);
        refresh_takes();
    end

    function onRoot()
        refresh_takes();
    end

    function draw_now()
        if isempty(st.takes)
            refresh_takes();
            if isempty(st.takes), return; end
        end
        idx = get(h.popTake, 'Value');
        if idx < 1 || idx > numel(st.takes), return; end
        takeName = st.takes{idx};
        takeDir = fullfile(st.root, takeName);

        n = str2double(get(h.edFrames, 'String'));
        if ~isfinite(n) || n < 1, n = 1; end
        n = round(n);
        set(h.edFrames, 'String', num2str(n));

        xMax = str2double(get(h.edXMax, 'String'));
        if ~isfinite(xMax) || xMax <= 0, xMax = 6; end

        % 选峰区间：只在这个距离窗口里找局部极大值
        lo = str2double(get(h.edPeakLo, 'String'));
        hi = str2double(get(h.edPeakHi, 'String'));
        if ~isfinite(lo) || lo < 0, lo = 0; end
        if ~isfinite(hi) || hi <= lo, hi = xMax; end
        if hi <= lo, hi = lo + 0.1; end
        set(h.edPeakLo, 'String', sprintf('%.2f', lo));
        set(h.edPeakHi, 'String', sprintf('%.2f', hi));

        % 邻域：前后各几个 bin 都比它低，才算极大值
        hw = str2double(get(h.edHalf, 'String'));
        if ~isfinite(hw) || hw < 1, hw = 5; end
        hw = round(hw);
        set(h.edHalf, 'String', num2str(hw));

        modes = get(h.popMode, 'String');
        md = modes{get(h.popMode, 'Value')};

        set(h.txt, 'String', sprintf('读 %s（%d 帧，峰区 %.2f-%.2f m，邻域 ±%d bin）…', ...
            takeName, n, lo, hi, hw));
        set(fig, 'Pointer', 'watch'); drawnow;
        t0 = tic;
        try
            [~, ~, prof, pk] = plot_range_power(struct( ...
                'takeDir', takeDir, 'nFrames', n, 'mode', md, ...
                'xMax', xMax, 'peakRange', [lo hi], 'peakHalf', hw, 'ax', h.ax));
        catch ex
            set(fig, 'Pointer', 'arrow');
            set(h.txt, 'String', sprintf('出错：%s', ex.message));
            errordlg(ex.message, '算不出来');
            return;
        end
        set(fig, 'Pointer', 'arrow');
        drawnow;

        P = range_params();
        lines = { ...
            sprintf('%s   |   %s   %d 帧平均   |   用时 %.1f s', ...
                takeName, md, n, toc(t0)), ...
            sprintf(['峰区 %.2f-%.2f m，邻域 ±%d bin（前/后都比它低才算极大值）：' ...
                     '找到 %d 个'], pk.range(1), pk.range(2), pk.half, numel(pk.cand))};
        if ~isempty(pk.bin)
            lines{end+1} = sprintf('主峰  bin %d  %.2f m  %.0f kHz  %.1f dB', ...
                pk.bin, pk.rng, pk.kHz, pk.dB);
            others = setdiff(pk.cand(:).', pk.bin);
            if ~isempty(others)
                [~, ord] = sort(prof(others), 'descend');
                c2 = others(ord(1));
                lines{end+1} = sprintf('次强  bin %d  %.2f m  %.1f dB（共 %d 个，图上橙点）', ...
                    c2, (c2-1)*P.dR, 20*log10(prof(c2)/max(prof)), numel(pk.cand));
            else
                lines{end+1} = '区间里只有这一个极大值。';
            end
        else
            lines{end+1} = sprintf('区间内没有 ±%d bin 的局部极大值。', pk.half);
        end
        set(h.txt, 'String', lines);
    end

    function save_png()
        [f, p] = uiputfile('*.png', '存图', fullfile(pwd, 'range_power.png'));
        if isequal(f, 0), return; end
        try
            exportgraphics(fig, fullfile(p, f), 'Resolution', 160);
            set(h.txt, 'String', sprintf('图已存：%s', fullfile(p, f)));
        catch ex
            errordlg(ex.message, '存图失败');
        end
    end

    function onKey(ev)
        if strcmp(ev.Key, 'return')
            draw_now();
        end
    end
end

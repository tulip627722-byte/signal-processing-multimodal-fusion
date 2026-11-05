function view_take(resFile, varargin)
%VIEW_TAKE 三画面播放器：左毫米波点云 / 中深度点云 / 右彩色视频
%
%   view_take                 % 先弹窗列出所有算好的数据，选一个再播
%   view_take('..._res.mat')  % 直接指定
%
% 操作：
%   播放/暂停   ⏵ 按钮，或空格
%   逐帧        ◀ ▶ 按钮，或 ← →
%   速度        下拉框 0.5x / 1x / 2x / 3x / 5x
%   旋转        鼠标拖拽或 rotate3d —— 左右两个点云同步
%   缩放        滚轮或 zoom —— 各自独立，不影响视频
%   锁定视角    勾上后每帧都按当前角度画
%   重置视角    按钮，或按 R
%
% 说明：彩色只做视频看动作，不再往深度点云上贴色（那需要相机标定，先放一放）。
%       中间的点云按距离着色，近亮远暗。

ip = inputParser;
ip.addParameter('frame', 1);
ip.addParameter('save', '');
ip.addParameter('vmin', 0);
ip.parse(varargin{:});
p = ip.Results;

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'algorithm', 'lib'), '-begin');   % pctile 在这里
resDir = paths().takesDir;

if nargin < 1 || isempty(resFile)
    resFile = pick_result(resDir);
    if isempty(resFile)
        fprintf('没有选中数据，退出。\n');
        return;
    end
end

S = load(resFile);
n = numel(S.frames);
k = max(1, min(n, p.frame));

st = struct();
st.k = k; st.n = n; st.playing = false; st.speed = 1.0;
st.lockView = false; st.view = [-32 22];
st.lastView = [nan nan; nan nan];
st.baseInterval = 0.25;
st.xlim = [-3 3]; st.ylim = [0 9]; st.zlim = [-1.4 1.6];st.cmap = lines(20);
st.imgCache = struct('frame', -1, 'img', []);
st.vmin = p.vmin;                 % 雷达最小速度（m/s），0 = 全部显示
st.xShift = 0;                    % 显示层的 X 平移（m），用于快速试偏移量
st.crange = depth_crange(S);      % 固定色标范围，避免播放时颜色闪
st.lim = auto_limits(S);          % 根据数据定范围，画面才不空
st.picks = {};                    % 取点记录（最多两个）
st.curP = []; st.curLab = []; st.curD = [];
st.txtPick = []; st.dcm = []; btnPick = [];

vis = 'off';
if isempty(p.save), vis = 'on'; end

fig = figure('Visible', vis, 'Color', 'w', 'Units', 'normalized', ...
    'Position', [0.03 0.07 0.94 0.86], 'NumberTitle', 'off', ...
    'Name', ['3.0 点云播放器 — ' S.metaOut.takeName], 'MenuBar', 'figure', 'ToolBar', 'figure');

ax1 = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.03 0.20 0.30 0.75]);
ax2 = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.35 0.20 0.30 0.75]);
ax3 = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.68 0.20 0.30 0.75]);

hNoise = []; hClu = gobjects(20, 1); hDep = []; hRadarOverlay = []; hImg = [];
sld = []; btnPlay = []; txtInfo = [];

build_axes();
build_controls();
render(st.k);

if ~isempty(p.save)
    % exportapp 只认绝对路径，先把相对路径转掉
    outPath = p.save;
    try
        outPath = char(java.io.File(p.save).getCanonicalPath());
    catch
        if isempty(fileparts(p.save)), outPath = fullfile(pwd, p.save); end
    end
    try
        exportapp(fig, outPath);           % 图里有 uicontrol，print 不支持
    catch ex
        fprintf(2, 'exportapp 失败：%s，改用 print\n', ex.message);
        print(fig, outPath, '-dpng', '-r100');
    end
    close(fig);
    fprintf('已存图：%s\n', outPath);
    return;
end

tPlay = timer('ExecutionMode', 'fixedSpacing', 'Period', st.baseInterval, ...
              'TimerFcn', @(~, ~) onTick(), 'Name', 'view_take_play');
tSync = timer('ExecutionMode', 'fixedRate', 'Period', 0.08, ...
              'TimerFcn', @(~, ~) sync_view(), 'Name', 'view_take_sync');
start(tSync);
set(fig, 'CloseRequestFcn', @(~, ~) onClose());
set(fig, 'KeyPressFcn', @(src, ev) onKey(ev));
set(fig, 'WindowScrollWheelFcn', @(src, ev) onWheel(ev));

% ================================================================
    function build_axes()
        st.xlim = st.lim{1}; st.ylim = st.lim{2}; st.zlim = st.lim{3};
        set(ax1, 'XLim', st.xlim, 'YLim', st.ylim, 'ZLim', st.zlim, ...
                 'XLimMode', 'manual', 'YLimMode', 'manual', 'ZLimMode', 'manual', ...
                 'DataAspectRatio', [1 1 1], 'DataAspectRatioMode', 'manual', ...
                 'View', st.view, 'Box', 'on', 'GridAlpha', 0.15);
        set(ax2, 'XLim', st.xlim, 'YLim', st.ylim, 'ZLim', st.zlim, ...
                 'XLimMode', 'manual', 'YLimMode', 'manual', 'ZLimMode', 'manual', ...
                 'DataAspectRatio', [1 1 1], 'DataAspectRatioMode', 'manual', ...
                 'View', st.view, 'Box', 'on', 'GridAlpha', 0.15);
        xlabel(ax1, 'X (m)'); ylabel(ax1, 'Y (m)'); zlabel(ax1, 'Z (m)');
        xlabel(ax2, 'X (m)'); ylabel(ax2, 'Y (m)'); zlabel(ax2, 'Z (m)');
        title(ax1, '毫米波点云');
        title(ax2, '深度点云 + 雷达叠加');
        title(ax3, '彩色视频');

        hold(ax1, 'on'); hold(ax2, 'on');
        hNoise = scatter3(NaN, NaN, NaN, 8, [0.75 0.75 0.75], 'filled', 'Parent', ax1);
        for c = 1:20
            hClu(c) = scatter3(NaN, NaN, NaN, 30, st.cmap(c, :), 'filled', 'Parent', ax1);
        end
        hDep = scatter3(NaN, NaN, NaN, 4, [0.5 0.7 0.9], 'filled', 'Parent', ax2);
        set(hDep, 'MarkerFaceColor', 'flat');
        hRadarOverlay = scatter3(NaN, NaN, NaN, 24, [0.9 0.1 0.1], 'o', ...
            'LineWidth', 1.2, 'Parent', ax2);
        colormap(ax2, parula);

        hImg = imshow(zeros(48, 64, 3, 'uint8'), 'Parent', ax3);
        axis(ax3, 'image'); axis(ax3, 'off');
        rotate3d(fig, 'on');
    end

    function build_controls()
        y1 = 0.135;
        uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
            'Position', [0.035 y1 0.045 0.055], 'String', '◀', ...
            'FontSize', 12, 'Callback', @(~, ~) step(-1));
        btnPlay = uicontrol(fig, 'Style', 'togglebutton', 'Units', 'normalized', ...
            'Position', [0.085 y1 0.06 0.055], 'String', '⏵ 播放', ...
            'FontSize', 11, 'Callback', @(~, ~) togglePlay());
        uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
            'Position', [0.15 y1 0.045 0.055], 'String', '▶', ...
            'FontSize', 12, 'Callback', @(~, ~) step(1));

        sld = uicontrol(fig, 'Style', 'slider', 'Units', 'normalized', ...
            'Position', [0.21 y1 0.34 0.055], 'Min', 1, 'Max', st.n, ...
            'Value', st.k, 'SliderStep', [1 / max(st.n - 1, 1) 5 / max(st.n - 1, 1)], ...
            'Callback', @(src, ~) jumpTo(round(get(src, 'Value'))));

        uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.565 y1+0.005 0.035 0.045], 'String', '速度', ...
            'HorizontalAlignment', 'right', 'FontSize', 10);
        uicontrol(fig, 'Style', 'popupmenu', 'Units', 'normalized', ...
            'Position', [0.605 y1 0.06 0.055], ...
            'String', {'0.5x', '1x', '2x', '3x', '5x'}, 'Value', 2, ...
            'Callback', @(src, ~) setSpeed(get(src, 'Value')));

        uicontrol(fig, 'Style', 'checkbox', 'Units', 'normalized', ...
            'Position', [0.685 y1+0.005 0.09 0.045], 'String', '锁定视角', ...
            'FontSize', 10, 'Tag', 'lock', 'Callback', @(src, ~) toggleLock(get(src, 'Value')));

        uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
            'Position', [0.78 y1 0.07 0.055], 'String', '重置视角', ...
            'FontSize', 10, 'Callback', @(~, ~) resetView());

        txtInfo = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.905 y1+0.005 0.085 0.05], 'String', '', ...
            'HorizontalAlignment', 'left', 'FontSize', 9);

        % ---- 取点按钮 ----
        btnPick = uicontrol(fig, 'Style', 'togglebutton', 'Units', 'normalized', ...
            'Position', [0.845 y1 0.055 0.055], 'String', '取点', ...
            'FontSize', 10, 'Callback', @(src, ~) togglePick(get(src, 'Value')));

        % ---- 底部：取点读数 ----
        st.txtPick = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.035 0.012 0.93 0.058], ...
            'String', {'点「取点」后单击图上的点，读数会显示在这里（连点两次可测两点直线距离）'}, ...
            'HorizontalAlignment', 'left', 'FontSize', 10, 'ForegroundColor', [0.15 0.25 0.45]);

        st.dcm = datacursormode(fig);
        set(st.dcm, 'UpdateFcn', @(~, ev) onCursor(ev), 'SnapToDataVertex', 'on', 'Enable', 'off');

        % ---- 第二行：雷达速度门限 ----
        y2 = 0.068;
        uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.035 y2+0.004 0.075 0.042], 'String', '速度门限', ...
            'HorizontalAlignment', 'left', 'FontSize', 10, 'FontWeight', 'bold');
        st.hV = uicontrol(fig, 'Style', 'slider', 'Units', 'normalized', ...
            'Position', [0.115 y2 0.28 0.05], 'Min', 0, 'Max', 1.2, 'Value', st.vmin, ...
            'Callback', @(src, ~) setVmin(get(src, 'Value')));
        st.txtV = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.40 y2+0.004 0.16 0.042], ...
            'String', sprintf('%.2f m/s', st.vmin), ...
            'HorizontalAlignment', 'left', 'FontSize', 10);
        if st.vmin <= 0.01
            set(st.txtV, 'String', '0.00 m/s（全部）');
        end
        uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
            'Position', [0.57 y2 0.06 0.05], 'String', '归零', ...
            'FontSize', 10, 'Callback', @(~, ~) resetVmin());

        % X 平移（左边试偏移量，不改数据）
        uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.655 y2+0.004 0.045 0.042], 'String', 'X 平移', ...
            'HorizontalAlignment', 'left', 'FontSize', 10);
        st.hXs = uicontrol(fig, 'Style', 'slider', 'Units', 'normalized', ...
            'Position', [0.715 y2 0.15 0.05], 'Min', -2, 'Max', 2, 'Value', 0, ...
            'Callback', @(src, ~) setXShift(get(src, 'Value')));
        st.txtXs = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.87 y2+0.004 0.10 0.042], 'String', '0.00 m', ...
            'HorizontalAlignment', 'left', 'FontSize', 10);
    end

    function render(idx)
        idx = max(1, min(st.n, idx));
        if idx ~= st.k, st.picks = {}; refreshReadout(); end   % 换帧就清掉旧的取点
        st.k = idx;

        % ---- 左：毫米波聚类 ----
        P = S.pts{idx};
        lab = S.labels{idx};
        if ~isempty(P) && st.xShift ~= 0
            P(:, 1) = P(:, 1) - st.xShift;
        end
        if ~isempty(P) && st.vmin > 0
            keep = abs(P(:, 6)) >= st.vmin;
            P = P(keep, :);
            lab = lab(keep);
        end
        if isempty(P)
            set(hNoise, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
            for c = 1:numel(hClu)
                set(hClu(c), 'XData', NaN, 'YData', NaN, 'ZData', NaN);
            end
        else
            m = lab == 0;
            if any(m)
                set(hNoise, 'XData', P(m, 1), 'YData', P(m, 2), 'ZData', P(m, 3));
            else
                set(hNoise, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
            end
            labB = zeros(size(lab));
            pos = lab > 0;
            labB(pos) = mod(lab(pos) - 1, numel(hClu)) + 1;
            for c = 1:numel(hClu)
                mm = labB == c;
                if any(mm)
                    set(hClu(c), 'XData', P(mm, 1), 'YData', P(mm, 2), 'ZData', P(mm, 3));
                else
                    set(hClu(c), 'XData', NaN, 'YData', NaN, 'ZData', NaN);
                end
            end
        end
        if st.vmin > 0
            title(ax1, sprintf('毫米波点云   帧 %d   %d 点 / %d 簇   |v|≥%.2f m/s', ...
                S.frames(idx), size(P, 1), numel(unique(lab(lab > 0))), st.vmin));
        else
            title(ax1, sprintf('毫米波点云   帧 %d   %d 点 / %d 簇', ...
                S.frames(idx), size(P, 1), S.nClusters(idx)));
        end

        % ---- 中：深度点云（按距离着色）----
        D = double(S.depthPts{idx});
        if isempty(D)
            set(hDep, 'XData', NaN, 'YData', NaN, 'ZData', NaN, 'CData', NaN);
        else
            set(hDep, 'XData', D(:, 1), 'YData', D(:, 2), 'ZData', D(:, 3), 'CData', D(:, 2));
            caxis(ax2, st.crange);
        end
        if isempty(P)
            set(hRadarOverlay, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
        else
            set(hRadarOverlay, 'XData', P(:, 1), 'YData', P(:, 2), 'ZData', P(:, 3));
        end
        title(ax2, sprintf('同一雷达坐标系：深度 %d 点 + 雷达 %d 点（红圈）', ...
            size(D, 1), size(P, 1)));

        % ---- 右：彩色视频 ----
        img = color_image(idx);
        if ~isempty(img)
            set(hImg, 'CData', img);
        end
        title(ax3, sprintf('彩色视频   帧 %d   Δt %+.1f ms', S.colorFrame(idx), S.depthDt(idx)));

        if st.lockView
            set(ax1, 'View', st.view);
            set(ax2, 'View', st.view);
        end

        % 供"取点"用：记住当前帧实际画出来的点
        st.curP = P; st.curLab = lab; st.curD = D;

        if ~isempty(sld), set(sld, 'Value', idx); end
        if ~isempty(txtInfo)
            set(txtInfo, 'String', sprintf('帧 %d / %d   %.0f ms', idx, st.n, S.ms(idx)));
        end
        drawnow limitrate;
    end

    function C = color_image(idx)
        C = [];
        if idx < 1 || idx > st.n, return; end
        fno = S.colorFrame(idx);
        if fno <= 0, return; end
        if st.imgCache.frame == fno && ~isempty(st.imgCache.img)
            C = st.imgCache.img;
            return;
        end
        pth = fullfile(S.metaOut.takeDir, 'color', sprintf('%06d.png', fno));
        if ~isfile(pth), return; end
        C = imread(pth);
        st.imgCache = struct('frame', fno, 'img', C);
    end

    function setVmin(v)
        st.vmin = v;
        if v <= 0.01
            set(st.txtV, 'String', '0.00 m/s（全部）');
        else
            set(st.txtV, 'String', sprintf('%.2f m/s', v));
        end
        render(st.k);
    end

    function resetVmin()
        st.vmin = 0;
        if isfield(st, 'hV') && isvalid(st.hV), set(st.hV, 'Value', 0); end
        set(st.txtV, 'String', '0.00 m/s（全部）');
        render(st.k);
    end

    function setXShift(v)
        st.xShift = v;
        set(st.txtXs, 'String', sprintf('%+.2f m', v));
        render(st.k);
    end

    function togglePick(on)
        if on
            rotate3d(fig, 'off');
            set(st.dcm, 'Enable', 'on');
            set(gcf, 'Pointer', 'crosshair');
        else
            set(st.dcm, 'Enable', 'off');
            set(gcf, 'Pointer', 'arrow');
            rotate3d(fig, 'on');
        end
    end

    function txt = onCursor(ev)
        axT = ancestor(ev.Target, 'axes');
        pos = ev.Position;
        if isempty(st.curD), st.curD = zeros(0,3); end
        if isempty(st.curP), st.curP = zeros(0,6); end

        if (isequal(axT, ax1) || isequal(ev.Target, hRadarOverlay)) && ~isempty(st.curP)
            d = sum((st.curP(:,1:3) - pos).^2, 2);
            [~, i] = min(d);
            p = st.curP(i, 1:3);
            cid = 0; if ~isempty(st.curLab), cid = st.curLab(i); end
            cen = [NaN NaN NaN]; ncl = 0;
            if cid > 0
                m = (st.curLab == cid);
                cen = mean(st.curP(m, 1:3), 1);
                ncl = nnz(m);
            end
            s = struct('kind', '雷达', 'p', p, 'v', st.curP(i,6), 'clu', cid, ...
                       'rng', st.curP(i,5), 'cen', cen, 'ncl', ncl);
        elseif ~isempty(st.curD)
            d = sum((st.curD(:,1:3) - pos).^2, 2);
            [~, i] = min(d);
            p = st.curD(i, 1:3);
            s = struct('kind', '深度', 'p', p, 'v', NaN, 'clu', NaN, 'rng', norm(p), ...
                       'cen', [NaN NaN NaN], 'ncl', 0);
        else
            txt = '';
            return;
        end

        if numel(st.picks) >= 2, st.picks = {}; end
        st.picks{end+1} = s;
        refreshReadout();
        txt = sprintf('%s  X%+.2f Y%+.2f Z%+.2f', s.kind, p(1), p(2), p(3));
    end

    function refreshReadout()
        if isempty(st.picks)
            set(st.txtPick, 'String', {'点「取点」后单击图上的点，读数会显示在这里'});
            return;
        end
        rd = cell(0, 1);
        for q = 1:numel(st.picks)
            s = st.picks{q};
            head = sprintf('%s   X=%+.3f  Y=%+.3f  Z=%+.3f   距原点 %.2f m', ...
                s.kind, s.p(1), s.p(2), s.p(3), norm(s.p));
            if strcmp(s.kind, '雷达') && isfinite(s.v)
                head = sprintf('%s   速度 %+.2f m/s   簇 %d', head, s.v, s.clu);
            end
            rd{end+1} = head; %#ok<AGROW>
            if isfield(s, 'ncl') && s.ncl > 0
                rd{end+1} = sprintf('     └ 簇心  X=%+.3f  Y=%+.3f  Z=%+.3f （%d 点，距原点 %.2f m）', ...
                    s.cen(1), s.cen(2), s.cen(3), s.ncl, norm(s.cen)); %#ok<AGROW>
            end
        end
        if numel(st.picks) == 2
            a = st.picks{1}.p; b = st.picks{2}.p;
            dv = b - a;
            rd{end+1} = sprintf('Δ = %.3f m   （ΔX=%+.3f  ΔY=%+.3f  ΔZ=%+.3f）', ...
                norm(dv), dv(1), dv(2), dv(3));
            if st.picks{1}.ncl > 0 && st.picks{2}.ncl > 0
                dc = st.picks{2}.cen - st.picks{1}.cen;
                rd{end+1} = sprintf('簇心 Δ = %.3f m   （ΔY 为主：%+.3f m）', norm(dc), dc(2)); %#ok<AGROW>
            end
        end
        set(st.txtPick, 'String', rd);
    end

    function step(d)
        stopIfPlaying();
        render(st.k + d);
    end

    function jumpTo(idx)
        render(idx);
    end

    function togglePlay()
        if st.playing
            stopPlay();
        else
            st.playing = true;
            set(btnPlay, 'String', '⏸ 暂停');
            start(tPlay);
        end
    end

    function stopPlay()
        st.playing = false;
        if isvalid(tPlay) && strcmp(tPlay.Running, 'on'), stop(tPlay); end
        if ~isempty(btnPlay), set(btnPlay, 'String', '⏵ 播放'); end
    end

    function stopIfPlaying()
        if st.playing, stopPlay(); end
    end

    function onTick()
        if ~st.playing, return; end
        nk = st.k + 1;
        if nk > st.n, nk = 1; end
        render(nk);
    end

    function setSpeed(v)
        speeds = [0.5 1 2 3 5];
        st.speed = speeds(v);
        if isvalid(tPlay)
            tPlay.Period = st.baseInterval / st.speed;
        end
    end

    function toggleLock(v)
        st.lockView = logical(v);
        if st.lockView
            st.view = get(ax1, 'View');
            set(ax2, 'View', st.view);
        end
    end

    function resetView()
        st.lockView = false;
        h = findobj(fig, 'Tag', 'lock');
        if ~isempty(h), set(h, 'Value', 0); end
        st.view = [-32 22];
        set(ax1, 'View', st.view); set(ax2, 'View', st.view);
        set(ax1, 'XLim', st.xlim, 'YLim', st.ylim, 'ZLim', st.zlim, ...
                 'DataAspectRatio', [1 1 1], 'DataAspectRatioMode', 'manual');
        set(ax2, 'XLim', st.xlim, 'YLim', st.ylim, 'ZLim', st.zlim, ...
                 'DataAspectRatio', [1 1 1], 'DataAspectRatioMode', 'manual');
    end

    function sync_view()
        if ~ishandle(ax1) || ~ishandle(ax2), return; end
        v1 = get(ax1, 'View');
        v2 = get(ax2, 'View');
        c1 = any(abs(v1 - st.lastView(1, :)) > 1e-6);
        c2 = any(abs(v2 - st.lastView(2, :)) > 1e-6);
        if c1 && ~c2
            set(ax2, 'View', v1); v2 = v1;
        elseif c2 && ~c1
            set(ax1, 'View', v2); v1 = v2;
        end
        st.lastView = [v1; v2];
        if st.lockView, st.view = v1; end
    end

    function onKey(ev)
        switch ev.Key
            case 'space', togglePlay();
            case 'leftarrow', step(-1);
            case 'rightarrow', step(1);
            case 'home', render(1);
            case 'end', render(st.n);
            case 'r', resetView();
        end
    end

    function onWheel(ev)
        pt = get(fig, 'CurrentPoint');
        ax = [];
        p1 = get(ax1, 'Position'); p2 = get(ax2, 'Position');
        if pt(1) >= p1(1) && pt(1) <= p1(1) + p1(3), ax = ax1; end
        if pt(1) >= p2(1) && pt(1) <= p2(1) + p2(3), ax = ax2; end
        if isempty(ax), return; end
        f = 0.90 ^ double(ev.VerticalScrollCount);
        xl = get(ax, 'XLim'); yl = get(ax, 'YLim'); zl = get(ax, 'ZLim');
        cx0 = mean(xl); cy0 = mean(yl); cz0 = mean(zl);
        set(ax, 'XLim', cx0 + (xl - cx0) * f, ...
                'YLim', cy0 + (yl - cy0) * f, ...
                'ZLim', cz0 + (zl - cz0) * f);
    end

    function onClose()
        try
            st.playing = false;
            if isvalid(tPlay), stop(tPlay); delete(tPlay); end
            if isvalid(tSync), stop(tSync); delete(tSync); end
        catch
        end
        delete(fig);
    end
end

% ================================================================
function lim = auto_limits(S)
%用几帧数据定坐标范围：X/Z 取百分位，Y 从 0 开始
lim = {[-3 3], [0 9], [-1.4 1.6]};
try
    n = numel(S.depthPts);
    idx = unique(round(linspace(1, n, min(6, n))));
    P = [];
    for i = idx
        d = double(S.depthPts{i});
        if ~isempty(d)
            step = max(1, floor(size(d, 1) / 3000));
            P = [P; d(1:step:end, :)]; %#ok<AGROW>
        end
        r = S.pts{i};
        if ~isempty(r), P = [P; r(:, 1:3)]; end %#ok<AGROW>
    end
    if size(P, 1) < 200, return; end
    xl = pctile(P(:, 1), [1 99]); yl = pctile(P(:, 2), [0.5 99.5]); zl = pctile(P(:, 3), [1 99]);
    lim = {[xl(1) - 0.3, xl(2) + 0.3], ...
           [min(0, yl(1)), max(yl(2) + 0.3, 3)], ...
           [zl(1) - 0.2, zl(2) + 0.2]};
catch
end
end

% ================================================================
function cr = depth_crange(S)
%取几帧的距离分布，定一个不变的颜色范围，不然每帧重算会闪
cr = [0 8];
try
    n = numel(S.depthPts);
    idx = unique(round(linspace(1, n, min(5, n))));
    vals = [];
    for i = idx
        d = double(S.depthPts{i});
        if ~isempty(d), vals = [vals; d(1:max(1, floor(size(d, 1) / 40)):end, 2)]; end %#ok<AGROW>
    end
    if ~isempty(vals)
        cr = [pctile(vals, 2), pctile(vals, 98)];
        if cr(2) - cr(1) < 0.5, cr = [0 8]; end
    end
catch
end
end

% ================================================================
function resFile = pick_result(resDir)
%启动时扫描 results，弹窗让人选一个
resFile = '';
list = dir(fullfile(resDir, '*_res.mat'));
if isempty(list)
    errordlg('results 里还没有算好的数据，先跑 compute_all。', '没有数据');
    return;
end
[~, ord] = sort([list.datenum], 'descend');
list = list(ord);

names = cell(numel(list), 1);
for i = 1:numel(list)
    nm = strrep(list(i).name, '_res.mat', '');
    names{i} = sprintf('%s   (%.1f MB, %s)', nm, list(i).bytes / 1e6, ...
        datestr(list(i).datenum, 'mm-dd HH:MM'));
end
[idx, ok] = listdlg('PromptString', '选择要播放的数据：', ...
                    'SelectionMode', 'single', ...
                    'ListString', names, ...
                    'ListSize', [560 260], ...
                    'Name', '3.0 点云播放器');
if ~ok || isempty(idx), return; end
resFile = fullfile(list(idx).folder, list(idx).name);
end

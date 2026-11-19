function view_take(resFile, varargin)
%VIEW_TAKE 多模态播放器：毫米波 / 深度 / MID-70 / 三路融合
%
%   view_take                 % 先弹窗列出所有算好的数据，选一个再播
%   view_take('..._res.mat')  % 直接指定，默认从采集中间帧开始
%   view_take('..._res.mat', 'frame', 1) % 明确查看录制开场帧
%   view_take('..._res.mat', 'lidarWindowMs', 300) % LiDAR 累计时间窗
%   view_take('..._res.mat', 'lidarExtrinsicsFile', '...json')
%
% 操作：
%   播放/暂停   ⏵ 按钮，或空格
%   逐帧        ◀ ▶ 按钮，或 ← →
%   速度        下拉框 0.5x / 1x / 2x / 3x / 5x
%   旋转        鼠标拖拽或 rotate3d —— 四个点云视角同步
%   缩放        滚轮或 zoom —— 各自独立，不影响视频
%   锁定视角    勾上后每帧都按当前角度画
%   重置视角    按钮，或按 R
%
% 说明：深度点云和 MID-70 都转换到 ADT7880 坐标后显示；第四窗格为三路
%       统一坐标的 3D 叠加，不使用 RGB 图像或相机工厂标定。

here = fileparts(mfilename('fullpath'));
ip = inputParser;
ip.addParameter('frame', [], @(v) isempty(v) || ...
    (isnumeric(v) && isscalar(v) && isfinite(v)));
ip.addParameter('save', '');
ip.addParameter('vmin', 0);
ip.addParameter('lidarWindowMs', 300, @(v) isnumeric(v) && isscalar(v) && v > 0);
ip.addParameter('maxLidarDisplayPoints', 30000, @is_positive_integer);
ip.addParameter('maxFusionLidarPoints', 12000, @is_positive_integer);
ip.addParameter('maxFusionDepthPoints', 8000, @is_positive_integer);
ip.addParameter('lidarExtrinsicsFile', default_lidar_extrinsics_file(here));
ip.parse(varargin{:});
p = ip.Results;

addpath(fullfile(here, 'algorithm', 'lib'), '-begin');   % pctile 在这里
addpath(fullfile(here, 'lidar'), '-begin');
resDir = paths().takesDir;

if nargin < 1 || isempty(resFile)
    resFile = pick_result(resDir);
    if isempty(resFile)
        fprintf('没有选中数据，退出。\n');
        return;
    end
end

S = load(resFile);
S.metaOut.takeDir = resolve_take_dir(S.metaOut.takeDir, S.metaOut.takeName, resDir);
validate_full_result(S, resFile);
n = numel(S.frames);
if isempty(p.frame)
    % 录制开头常包含人员搬动标定板或目标尚未就位的过渡帧。
    % 保留全部原始帧，但默认打开采集中间，避免把开场位置误当测距错误。
    k = round((n + 1) / 2);
else
    k = max(1, min(n, round(p.frame)));
end

st = struct();
st.k = k; st.n = n; st.playing = false; st.speed = 1.0;
st.lockView = false; st.view = [-32 22];
st.lastView = [nan nan; nan nan];
st.baseInterval = 0.25;
st.xlim = [-3 3]; st.ylim = [0 9]; st.zlim = [-1.4 1.6];st.cmap = lines(20);
st.lidar = lidar_index(S, p.lidarWindowMs);
st.lidar = configure_lidar_extrinsics(st.lidar, p.lidarExtrinsicsFile);
st.fusionAvailable = st.lidar.available && st.lidar.extrinsicsAvailable;
st.maxLidarDisplayPoints = p.maxLidarDisplayPoints;
st.maxFusionLidarPoints = p.maxFusionLidarPoints;
st.maxFusionDepthPoints = p.maxFusionDepthPoints;
st.vmin = p.vmin;                 % 雷达最小速度（m/s），0 = 全部显示
st.xShift = 0;                    % 显示层的 X 平移（m），用于快速试偏移量
st.crange = depth_crange(S);      % 固定色标范围，避免播放时颜色闪
st.lim = auto_limits(S);          % 根据数据定范围，画面才不空
st.picks = {};                    % 取点记录（最多两个）
st.curP = []; st.curLab = []; st.curD = []; st.curL = []; st.curLR = [];
st.txtPick = []; st.dcm = []; btnPick = [];

vis = 'off';
if isempty(p.save), vis = 'on'; end

fig = figure('Visible', vis, 'Color', 'w', 'Units', 'normalized', ...
    'Position', [0.03 0.07 0.94 0.86], 'NumberTitle', 'off', ...
    'Name', ['3.0 点云播放器 — ' S.metaOut.takeName], 'MenuBar', 'figure', 'ToolBar', 'figure');

if st.lidar.available
    ax1 = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.02 0.57 0.46 0.36]);
    ax2 = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.52 0.57 0.46 0.36]);
    ax3 = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.02 0.19 0.46 0.34]);
    ax4 = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.52 0.19 0.46 0.34]);
else
    ax1 = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.03 0.20 0.30 0.75]);
    ax2 = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.35 0.20 0.30 0.75]);
    ax3 = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.68 0.20 0.30 0.75]);
    ax4 = [];
end

hNoise = []; hClu = gobjects(20, 1); hDep = []; hRadarOverlay = []; hLidar = [];
hFusionRadar = []; hFusionDepth = []; hFusionLidar = [];
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
        title(ax2, '深度相机点云（雷达坐标）');
        if st.lidar.available
            set(ax3, 'XLim', st.lidar.lim{1}, 'YLim', st.lidar.lim{2}, 'ZLim', st.lidar.lim{3}, ...
                     'XLimMode', 'manual', 'YLimMode', 'manual', 'ZLimMode', 'manual', ...
                     'DataAspectRatio', [1 1 1], 'DataAspectRatioMode', 'manual', ...
                     'View', [-35 22], 'Box', 'on', 'GridAlpha', 0.15);
            xlabel(ax3, 'X 前 (m)'); ylabel(ax3, 'Y 左 (m)'); zlabel(ax3, 'Z 上 (m)');
            title(ax3, 'MID-70 激光点云（原生坐标）');
            hold(ax3, 'on');
            hLidar = scatter3(NaN, NaN, NaN, 3, NaN, 'filled', 'Parent', ax3, ...
                'MarkerEdgeColor', 'none');
            colormap(ax3, turbo);
            set(ax4, 'XLim', st.xlim, 'YLim', st.ylim, 'ZLim', st.zlim, ...
                'XLimMode', 'manual', 'YLimMode', 'manual', 'ZLimMode', 'manual', ...
                'DataAspectRatio', [1 1 1], 'DataAspectRatioMode', 'manual', ...
                'View', st.view, 'Box', 'on', 'GridAlpha', 0.15);
            xlabel(ax4, 'X (m)'); ylabel(ax4, 'Y (m)'); zlabel(ax4, 'Z (m)');
            hold(ax4, 'on');
            title(ax4, '三路叠加（统一 ADT7880 坐标）');
        else
            title(ax3, 'MID-70 不可用');
        end

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

        if st.lidar.available
            hFusionDepth = scatter3(ax4, NaN, NaN, NaN, 4, [0.10 0.72 0.25], 'filled', ...
                'MarkerFaceAlpha', 0.22, 'MarkerEdgeColor', 'none');
            hFusionLidar = scatter3(ax4, NaN, NaN, NaN, 5, [0.05 0.32 0.95], 'filled', ...
                'MarkerFaceAlpha', 0.48, 'MarkerEdgeColor', 'none');
            hFusionRadar = scatter3(ax4, NaN, NaN, NaN, 42, [0.95 0.05 0.03], 'o', ...
                'LineWidth', 1.5);
            legend(ax4, [hFusionDepth hFusionLidar hFusionRadar], ...
                {'深度', 'MID-70', '毫米波'}, 'Location', 'southoutside', ...
                'Orientation', 'horizontal');
        end
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

        % ---- 深度点云（按距离着色）----
        D = double(S.depthPts{idx});
        if isempty(D)
            set(hDep, 'XData', NaN, 'YData', NaN, 'ZData', NaN, 'CData', NaN);
        else
            set(hDep, 'XData', D(:, 1), 'YData', D(:, 2), 'ZData', D(:, 3), 'CData', D(:, 2));
            caxis(ax2, st.crange);
        end
        set(hRadarOverlay, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
        title(ax2, sprintf('深度相机点云   帧 %d   %d 点', S.depthFrame(idx), size(D, 1)));

        % ---- MID-70：以当前雷达帧为中心累计指定时间窗 ----
        L = zeros(0, 3);
        LR = zeros(0, 3);
        reflectivity = zeros(0, 1);
        if st.lidar.available
            rowIndices = st.lidar.rowsForFrame{idx};
            for rowIndex = rowIndices(:).'
                if isempty(st.lidar.pointsCache{rowIndex})
                    batch = read_livox_bin(st.lidar.files{rowIndex});
                    st.lidar.pointsCache{rowIndex} = double(batch.points);
                    st.lidar.reflectivityCache{rowIndex} = double(batch.reflectivity);
                    if st.lidar.extrinsicsAvailable
                        st.lidar.radarPointsCache{rowIndex} = transform_lidar_points( ...
                            st.lidar.pointsCache{rowIndex}, st.lidar.extrinsics);
                    end
                end
                L = [L; st.lidar.pointsCache{rowIndex}]; %#ok<AGROW>
                reflectivity = [reflectivity; st.lidar.reflectivityCache{rowIndex}]; %#ok<AGROW>
                if st.lidar.extrinsicsAvailable
                    LR = [LR; st.lidar.radarPointsCache{rowIndex}]; %#ok<AGROW>
                end
            end
            if ~isempty(L)
                displayIndices = display_sample_indices(size(L, 1), st.maxLidarDisplayPoints);
                set(hLidar, 'XData', L(displayIndices, 1), 'YData', L(displayIndices, 2), ...
                    'ZData', L(displayIndices, 3), 'CData', reflectivity(displayIndices));
            else
                set(hLidar, 'XData', NaN, 'YData', NaN, 'ZData', NaN, 'CData', NaN);
            end
            if st.lidar.valid(idx)
                deltaText = sprintf('Δt %+.1f ms', st.lidar.dtMs(idx));
            else
                deltaText = sprintf('Δt %+.1f ms（偏大）', st.lidar.dtMs(idx));
            end
            if isempty(rowIndices)
                batchText = '-';
            else
                batchText = sprintf('%d–%d', st.lidar.batches(rowIndices(1)), ...
                    st.lidar.batches(rowIndices(end)));
            end
            title(ax3, sprintf('MID-70  %.0f ms  %d 点  %s', ...
                st.lidar.windowMs, size(L, 1), deltaText));
        end

        if st.lidar.available
            render_fusion_3d(D, P, LR, idx);
        else
            title(ax3, 'MID-70 不可用');
        end

        if st.lockView
            set(ax1, 'View', st.view);
            set(ax2, 'View', st.view);
            if st.lidar.available
                set(ax3, 'View', st.view);
                set(ax4, 'View', st.view);
            end
        end

        % 供"取点"用：记住当前帧实际画出来的点
        st.curP = P; st.curLab = lab; st.curD = D; st.curL = L; st.curLR = LR;

        if ~isempty(sld), set(sld, 'Value', idx); end
        if ~isempty(txtInfo)
            set(txtInfo, 'String', sprintf('帧 %d / %d   %.0f ms', idx, st.n, S.ms(idx)));
        end
        drawnow limitrate;
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
        if isempty(st.curL), st.curL = zeros(0,3); end
        if isempty(st.curLR), st.curLR = zeros(0,3); end

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
        elseif st.lidar.available && isequal(axT, ax3) && ~isempty(st.curL)
            d = sum((st.curL(:,1:3) - pos).^2, 2);
            [~, i] = min(d);
            p = st.curL(i, 1:3);
            s = struct('kind', '激光', 'p', p, 'v', NaN, 'clu', NaN, 'rng', norm(p), ...
                       'cen', [NaN NaN NaN], 'ncl', 0);
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
            if st.lidar.available
                set(ax3, 'View', st.view);
                set(ax4, 'View', st.view);
            end
        end
    end

    function resetView()
        st.lockView = false;
        h = findobj(fig, 'Tag', 'lock');
        if ~isempty(h), set(h, 'Value', 0); end
        st.view = [-32 22];
        set(ax1, 'View', st.view); set(ax2, 'View', st.view);
        if st.lidar.available
            set(ax3, 'View', st.view);
            set(ax4, 'View', st.view);
        end
        set(ax1, 'XLim', st.xlim, 'YLim', st.ylim, 'ZLim', st.zlim, ...
                 'DataAspectRatio', [1 1 1], 'DataAspectRatioMode', 'manual');
        set(ax2, 'XLim', st.xlim, 'YLim', st.ylim, 'ZLim', st.zlim, ...
                 'DataAspectRatio', [1 1 1], 'DataAspectRatioMode', 'manual');
    end

    function sync_view()
        if ~ishandle(ax1) || ~ishandle(ax2), return; end
        axesList = [ax1 ax2];
        if st.lidar.available, axesList = [axesList ax3 ax4]; end
        views = cell2mat(arrayfun(@(a) get(a, 'View'), axesList, 'UniformOutput', false).');
        if size(st.lastView, 1) ~= size(views, 1)
            st.lastView = repmat(views(1,:), size(views,1), 1);
        end
        if any(abs(views(1,:) - st.lastView(1,:)) > 1e-6)
            targetView = views(1,:);
        else
            changed = find(any(abs(views - st.lastView(1:size(views,1),:)) > 1e-6, 2), 1);
            if isempty(changed), targetView = views(1,:); else, targetView = views(changed,:); end
        end
        for axisHandle = axesList
            set(axisHandle, 'View', targetView);
        end
        st.lastView = repmat(targetView, size(views,1), 1);
        if st.lockView, st.view = targetView; end
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
        if st.lidar.available
            p3 = get(ax3, 'Position');
            if pt(1) >= p3(1) && pt(1) <= p3(1) + p3(3), ax = ax3; end
            p4 = get(ax4, 'Position');
            if pt(1) >= p4(1) && pt(1) <= p4(1) + p4(3), ax = ax4; end
        end
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

    function render_fusion_3d(D, P, LR, idx)
        if ~st.fusionAvailable
            set_fusion_cloud(hFusionDepth, zeros(0, 3));
            set_fusion_cloud(hFusionLidar, zeros(0, 3));
            set_fusion_cloud(hFusionRadar, zeros(0, 3));
            title(ax4, '三路叠加：未启用有效 MID-70→雷达外参');
            return;
        end
        depthRadar = D(:, 1:min(3, size(D, 2)));
        radarPoints = P(:, 1:min(3, size(P, 2)));
        depthDisplay = display_sample(depthRadar, st.maxFusionDepthPoints);
        lidarDisplay = display_sample(LR, st.maxFusionLidarPoints);
        set_fusion_cloud(hFusionDepth, depthDisplay);
        set_fusion_cloud(hFusionLidar, lidarDisplay);
        set_fusion_cloud(hFusionRadar, radarPoints);
        title(ax4, sprintf('三路融合  帧 %d   雷达 %d / 深度 %d / MID-70 %d', ...
            S.frames(idx), size(radarPoints, 1), size(depthRadar, 1), size(LR, 1)));
    end

    function set_fusion_cloud(handle, points)
        if isempty(points)
            set(handle, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
        else
            set(handle, 'XData', points(:,1), 'YData', points(:,2), 'ZData', points(:,3));
        end
    end
end

function points = display_sample(points, maximumCount)
indices = display_sample_indices(size(points, 1), maximumCount);
points = points(indices, :);
end

function indices = display_sample_indices(count, maximumCount)
if count <= maximumCount
    indices = (1:count).';
else
    indices = unique(round(linspace(1, count, maximumCount))).';
end
end

function valid = is_positive_integer(value)
valid = isnumeric(value) && isscalar(value) && isfinite(value) && ...
    value >= 1 && value == round(value);
end

% ================================================================
function info = lidar_index(S, windowMs)
%以处理结果中的雷达时间为中心，选择最近 MID-70 批次或短时间窗。
n = numel(S.frames);
info = struct('available', false, 'batchForFrame', zeros(n, 1), ...
    'rowsForFrame', {cell(n, 1)}, 'dtMs', nan(n, 1), 'valid', false(n, 1), ...
    'files', {{}}, 'batches', [], 'windowMs', windowMs, ...
    'pointsCache', {{}}, 'reflectivityCache', {{}}, 'radarPointsCache', {{}}, ...
    'extrinsicsAvailable', false, 'extrinsics', struct(), ...
    'lim', {{[-3 3], [0 9], [-1.4 1.6]}});
try
    takeDir = S.metaOut.takeDir;
    csvPath = fullfile(takeDir, 'lidar_frames.csv');
    lidarDir = fullfile(takeDir, 'lidar');
    if ~isfile(csvPath) || ~isfolder(lidarDir), return; end
    rows = readtable(csvPath, 'TextType', 'string');
    if isempty(rows), return; end
    lidarTime = (double(rows.first_t_rel_ms) + double(rows.last_t_rel_ms)) / 2;
    batches = double(rows.batch);
    if ismember('file', rows.Properties.VariableNames)
        files = cellstr(rows.file);
    else
        files = arrayfun(@(v) fullfile('lidar', sprintf('%06d.bin', v)), ...
            batches, 'UniformOutput', false);
    end
    resolvedFiles = cell(size(files));
    for rowIndex = 1:numel(files)
        path = char(files{rowIndex});
        if ~isfile(path), path = fullfile(takeDir, strrep(path, '/', filesep)); end
        resolvedFiles{rowIndex} = path;
    end
    info.files = resolvedFiles;
    info.batches = batches;
    info.pointsCache = cell(numel(files), 1);
    info.reflectivityCache = cell(numel(files), 1);
    info.radarPointsCache = cell(numel(files), 1);
    for idx = 1:n
        [delta, nearest] = min(abs(lidarTime - double(S.radarT(idx))));
        info.batchForFrame(idx) = batches(nearest);
        info.dtMs(idx) = lidarTime(nearest) - double(S.radarT(idx));
        info.valid(idx) = delta <= 30;
        if windowMs <= 100
            info.rowsForFrame{idx} = nearest;
        else
            windowStart = double(S.radarT(idx)) - windowMs;
            windowEnd = double(S.radarT(idx));
            info.rowsForFrame{idx} = find(lidarTime >= windowStart & lidarTime <= windowEnd).';
        end
        if isempty(info.rowsForFrame{idx}), info.rowsForFrame{idx} = nearest; end
    end
    info.available = all(cellfun(@isfile, info.files));
    if info.available, info.lim = lidar_limits(info.files); end
catch ex
    warning('view_take:LiDARUnavailable', 'LiDAR 未载入：%s', ex.message);
end
end

function takeDir = resolve_take_dir(takeDir, takeName, resultsTakesDir)
takeDir = char(takeDir);
if isfolder(takeDir), return; end
takeDir = '';
try
    projectPaths = paths();
    candidate = fullfile(projectPaths.captureTakes, char(takeName));
    if isfolder(candidate), takeDir = candidate; return; end
catch
end
candidate = fullfile(fileparts(resultsTakesDir), '..', '..', char(takeName));
if isfolder(candidate), takeDir = candidate; end
end

function validate_full_result(S, resFile)
if isfield(S, 'metaOut') && isfield(S.metaOut, 'isPartial') && logical(S.metaOut.isPartial)
    error('view_take:PartialResult', ...
        '拒绝播放抽样结果：%s（%d/%d 帧）。请运行 compute_all 强制完整重算。', ...
        resFile, S.metaOut.processedFrameCount, S.metaOut.fullFrameCount);
end
end

function path = default_lidar_extrinsics_file(matlabDir)
%DEFAULT_LIDAR_EXTRINSICS_FILE Select the newest applicable calibration.
path = '';
resultsRoot = fullfile(matlabDir, '..', 'results');
folders = dir(fullfile(resultsRoot, 'lidar_radar_calibration_*'));
folders = folders([folders.isdir]);
[~, order] = sort([folders.datenum], 'descend');
folders = folders(order);
for index = 1:numel(folders)
    candidate = fullfile(folders(index).folder, folders(index).name, ...
        'lidar_radar_extrinsics_candidate.json');
    if ~isfile(candidate), continue; end
    try
        value = jsondecode(fileread(candidate));
        if isfield(value, 'applicable') && logical(value.applicable)
            path = candidate;
            return;
        end
    catch
    end
end
end

function info = configure_lidar_extrinsics(info, path)
%CONFIGURE_LIDAR_EXTRINSICS Enable fusion only for an applicable candidate.
if isempty(path) || ~info.available
    return;
end
path = char(path);
if ~isfile(path)
    warning('view_take:ExtrinsicsUnavailable', 'LiDAR 外参文件不存在，融合画面降级：%s', path);
    return;
end
try
    extrinsics = load_lidar_radar_extrinsics(path);
    if ~isfield(extrinsics, 'applicable') || ~logical(extrinsics.applicable)
        warning('view_take:ExtrinsicsNotApplicable', ...
            'LiDAR 外参不可用（applicable=false），融合画面降级：%s', path);
        return;
    end
    info.extrinsics = extrinsics;
    info.extrinsicsAvailable = true;
catch ex
    warning('view_take:ExtrinsicsInvalid', 'LiDAR 外参无效，融合画面降级：%s', ex.message);
end
end

function pointsRadar = transform_lidar_points(points, extrinsics)
pointsRadar = (double(extrinsics.lidarToRadarR) * double(points(:,1:3)).').';
pointsRadar = pointsRadar + reshape(double(extrinsics.lidarOriginInRadarM), 1, 3);
end

% ================================================================
function lim = lidar_limits(files)
%抽样若干批次得到稳定显示范围，避免播放时坐标轴闪动。
lim = {[-3 3], [0 9], [-1.4 1.6]};
try
    idx = unique(round(linspace(1, numel(files), min(6, numel(files)))));
    points = zeros(0, 3);
    for fileIndex = idx
        batch = read_livox_bin(files{fileIndex});
        cloud = double(batch.points);
        step = max(1, floor(size(cloud, 1) / 3000));
        points = [points; cloud(1:step:end, :)]; %#ok<AGROW>
    end
    if size(points, 1) < 20, return; end
    x = pctile(points(:, 1), [1 99]);
    y = pctile(points(:, 2), [1 99]);
    z = pctile(points(:, 3), [1 99]);
    lim = {[x(1)-0.2 x(2)+0.2], [y(1)-0.2 y(2)+0.2], [z(1)-0.2 z(2)+0.2]};
catch
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

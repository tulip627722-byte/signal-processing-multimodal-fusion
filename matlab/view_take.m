function view_take(resFile, varargin)
%VIEW_TAKE 多模态播放器：毫米波 / 深度 / MID-70 / 三路融合
%
%   view_take                 % 先弹窗列出所有算好的数据，选一个再播
%   view_take('..._res.mat')  % 直接指定，默认从采集中间帧开始
%   view_take('..._res.mat', 'frame', 1) % 明确查看录制开场帧
%   view_take('..._res.mat', 'lidarWindowMs', 1000)        % LiDAR 累计时间窗，默认 1000 ms
%   view_take('..._res.mat', 'lidarWindowAlign', 'center') % center=以当前帧为中心（离线回放）
%                                                          % trailing=取当前帧之前 W ms（模拟实时）
%   view_take('..._res.mat', 'lidarFgMs', 300)             % 近景高亮窗口，叠加在长窗之上
%   view_take('..._res.mat', 'lidarVoxelM', 0.03)          % LiDAR 体素平均边长(m)，0=关闭
%   view_take('..._res.mat', 'depthFilter', true)          % 显示清理后深度（默认开启）
%   view_take('..._res.mat', 'depthVoxelM', 0.015)         % 去噪时的深度体素边长(m)
%   view_take('..._res.mat', 'depthMinNeighbors', 1)       % 去噪时的体素半径滤波最少邻居数
%   view_take('..._res.mat', 'lidarFgMinDistM', 0.08)      % 背景差分前景阈值(m)
%   view_take('..._res.mat', 'lidarExtrinsicsFile', '...json')
%
% 操作：
%   播放/暂停   ⏵ 按钮，或空格
%   逐帧        ◀ ▶ 按钮，或 ← →
%   速度        下拉框 0.5x / 1x / 2x / 3x / 5x
%   LiDAR窗     下拉框 100~2000 ms，现场切换 MID-70 累计时间窗
%   视角        3D / 俯视 X-Y / 侧视 Y-Z / 正视 X-Z —— 用来检查深度与 LiDAR 在深度方向是否重合
%   近景高亮    勾上后，三路叠加里最近 lidarFgMs 的 MID-70 点用亮蓝色叠在长窗（淡蓝）之上，
%               运动物体看亮色、静止背景看淡色
%   清理后深度  切换单窗显示原始/清理点云；融合窗始终使用清理后的主点云
%   旋转        鼠标拖拽或 rotate3d —— 四个点云视角同步
%   缩放        滚轮或 zoom —— 各自独立，不影响视频
%   锁定视角    勾上后每帧都按当前角度画
%   重置视角    按钮，或按 R
%
% 说明：深度点云和 MID-70 都转换到 ADT7880 坐标后显示；第四窗格为三路
%       统一坐标的 3D 叠加，不使用 RGB 图像或相机工厂标定。
%       MID-70 为非重复扫描，积分时间越长覆盖越完整（0.2 s 约等于 32 线，约 1.5 s 覆盖约 86%），
%       所以默认累计 1000 ms；窗口内的点再做体素平均，降噪并去掉重叠。

here = fileparts(mfilename('fullpath'));
ip = inputParser;
ip.addParameter('frame', [], @(v) isempty(v) || ...
    (isnumeric(v) && isscalar(v) && isfinite(v)));
ip.addParameter('save', '');
ip.addParameter('vmin', 0);
ip.addParameter('lidarWindowMs', 1000, @(v) isnumeric(v) && isscalar(v) && v > 0);
ip.addParameter('lidarWindowAlign', 'center', @(v) ischar(v) || (isstring(v) && isscalar(v)));
ip.addParameter('lidarFgMs', 300, @(v) isnumeric(v) && isscalar(v) && v > 0);
ip.addParameter('lidarVoxelM', 0.03, @(v) isnumeric(v) && isscalar(v) && isfinite(v) && v >= 0);
ip.addParameter('depthFilter', true, @(v) (islogical(v) || isnumeric(v)) && isscalar(v));
ip.addParameter('depthVoxelM', 0.015, @(v) isnumeric(v) && isscalar(v) && isfinite(v) && v >= 0);
ip.addParameter('depthMinNeighbors', 1, @(v) isnumeric(v) && isscalar(v) && isfinite(v) && v >= 0 && v == round(v));
ip.addParameter('lidarFgMinDistM', 0.08, @(v) isnumeric(v) && isscalar(v) && isfinite(v) && v >= 0);
ip.addParameter('lidarCacheBatches', 600, @is_positive_integer);
ip.addParameter('maxLidarDisplayPoints', 50000, @is_positive_integer);
ip.addParameter('maxFusionLidarPoints', 30000, @is_positive_integer);
ip.addParameter('maxFusionDepthPoints', 20000, @is_positive_integer);
ip.addParameter('lidarExtrinsicsFile', default_lidar_extrinsics_file(here));
ip.addParameter('offlineFusionFile', '', @(v) ischar(v) || (isstring(v) && isscalar(v)));
ip.parse(varargin{:});
p = ip.Results;
lidarAlign = lower(char(p.lidarWindowAlign));
if ~ismember(lidarAlign, {'center', 'trailing'})
    error('view_take:BadAlign', 'lidarWindowAlign 只能是 ''center'' 或 ''trailing''。');
end

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

if isfolder(resFile)
    [~, takeNameFromFolder] = fileparts(char(resFile));
    candidateResult = fullfile(resDir, [takeNameFromFolder '_res.mat']);
    if isfile(candidateResult)
        resFile = candidateResult;
    else
        error('view_take:TakeDirectoryOnly', ...
            '传入的是 take 目录，但找不到处理结果：%s。请先运行 compute_take/compute_all。', candidateResult);
    end
end

S = load(resFile);
offlineFusion = load_offline_fusion(p.offlineFusionFile, S.metaOut.takeName, resDir);
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
st.lidar = lidar_index(S, p.lidarWindowMs, p.lidarFgMs, lidarAlign);
st.lidar = configure_lidar_extrinsics(st.lidar, p.lidarExtrinsicsFile);
st.fusionAvailable = st.lidar.available && st.lidar.extrinsicsAvailable;
st.offlineFusion = offlineFusion;
st.maxLidarDisplayPoints = p.maxLidarDisplayPoints;
st.maxFusionLidarPoints = p.maxFusionLidarPoints;
st.maxFusionDepthPoints = p.maxFusionDepthPoints;
st.lidarWindowMs = p.lidarWindowMs;
st.lidarFgMs = p.lidarFgMs;
st.lidarAlign = lidarAlign;
st.lidarVoxel = p.lidarVoxelM;
st.depthFilter = logical(p.depthFilter);
st.depthVoxel = p.depthVoxelM;
st.depthMinNeighbors = p.depthMinNeighbors;
st.lidarFgMinDist = p.lidarFgMinDistM;
st.lidarCacheBatches = p.lidarCacheBatches;
st.winOpts = unique([100 300 500 1000 1500 2000 round(p.lidarWindowMs)]);
st.showFg = true;                 % 三路叠加里是否高亮近景
st.cenText = [];                  % 雷达簇心编号文字句柄
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
    'Name', ['3.1 点云播放器 — ' S.metaOut.takeName], 'MenuBar', 'figure', 'ToolBar', 'figure');

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
hFusionLidarFg = []; hFusionRadarCen = []; hFusionGuidedRadar = [];
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
            hLidar = scatter3(NaN, NaN, NaN, 4, [0.25 0.45 0.90], 'filled', ...
                'Parent', ax3, 'MarkerEdgeColor', 'none', 'MarkerFaceAlpha', 0.55);
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
        % 深度点径不小于 LiDAR；单色半透明显示几何表面，避免彩色噪声掩盖目标。
        hDep = scatter3(NaN, NaN, NaN, 8, [0.10 0.62 0.68], 'filled', ...
            'Parent', ax2, 'MarkerEdgeColor', 'none', 'MarkerFaceAlpha', 0.45);
        hRadarOverlay = scatter3(NaN, NaN, NaN, 24, [0.9 0.1 0.1], 'o', ...
            'LineWidth', 1.2, 'Parent', ax2);

        if st.lidar.available
            % 叠加顺序（先画的在下）：深度 -> LiDAR 长窗(淡蓝) -> LiDAR 近景(亮蓝) -> 雷达点 -> 簇心
            hFusionDepth = scatter3(ax4, NaN, NaN, NaN, 8, [0.10 0.62 0.25], 'filled', ...
                'MarkerFaceAlpha', 0.45, 'MarkerEdgeColor', 'none');
            hFusionLidar = scatter3(ax4, NaN, NaN, NaN, 3, [0.35 0.52 0.92], 'filled', ...
                'MarkerFaceAlpha', 0.30, 'MarkerEdgeColor', 'none');
            hFusionLidarFg = scatter3(ax4, NaN, NaN, NaN, 5, [0.05 0.25 0.90], 'filled', ...
                'MarkerFaceAlpha', 0.80, 'MarkerEdgeColor', 'none');
            hFusionRadar = scatter3(ax4, NaN, NaN, NaN, 12, [0.95 0.45 0.30], 'filled', ...
                'MarkerFaceAlpha', 0.45, 'MarkerEdgeColor', 'none');
            hFusionRadarCen = scatter3(ax4, NaN, NaN, NaN, 150, [0.85 0 0], 'filled', ...
                'Marker', 'p', 'MarkerEdgeColor', 'k');
            hFusionGuidedRadar = scatter3(ax4, NaN, NaN, NaN, 170, [.55 0 .85], 'filled', ...
                'Marker', 'd', 'MarkerEdgeColor', 'k');
            legend(ax4, [hFusionDepth hFusionLidar hFusionLidarFg hFusionRadar hFusionRadarCen hFusionGuidedRadar], ...
                {'深度', 'MID-70 长窗', 'MID-70 近景', '毫米波点', '簇心', '雷达距离约束板面点'}, ...
                'Location', 'southoutside', 'Orientation', 'horizontal');
        end
        rotate3d(fig, 'on');
    end

    function build_controls()
        y1 = 0.135;
        lidarOn = 'off';
        if st.lidar.available, lidarOn = 'on'; end

        uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
            'Position', [0.020 y1 0.040 0.055], 'String', '◀', ...
            'FontSize', 12, 'Callback', @(~, ~) step(-1));
        btnPlay = uicontrol(fig, 'Style', 'togglebutton', 'Units', 'normalized', ...
            'Position', [0.062 y1 0.055 0.055], 'String', '⏵ 播放', ...
            'FontSize', 11, 'Callback', @(~, ~) togglePlay());
        uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
            'Position', [0.119 y1 0.040 0.055], 'String', '▶', ...
            'FontSize', 12, 'Callback', @(~, ~) step(1));

        sld = uicontrol(fig, 'Style', 'slider', 'Units', 'normalized', ...
            'Position', [0.165 y1 0.230 0.055], 'Min', 1, 'Max', st.n, ...
            'Value', st.k, 'SliderStep', [1 / max(st.n - 1, 1) 5 / max(st.n - 1, 1)], ...
            'Callback', @(src, ~) jumpTo(round(get(src, 'Value'))));

        uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.400 y1+0.005 0.030 0.045], 'String', '速度', ...
            'HorizontalAlignment', 'right', 'FontSize', 10);
        uicontrol(fig, 'Style', 'popupmenu', 'Units', 'normalized', ...
            'Position', [0.432 y1 0.050 0.055], ...
            'String', {'0.5x', '1x', '2x', '3x', '5x'}, 'Value', 2, ...
            'Callback', @(src, ~) setSpeed(get(src, 'Value')));

        % ---- LiDAR 累计时间窗 ----
        winStr = arrayfun(@(v) sprintf('%d ms', v), st.winOpts, 'UniformOutput', false);
        winVal = find(st.winOpts == round(st.lidarWindowMs), 1);
        if isempty(winVal), winVal = 1; end
        uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.487 y1+0.005 0.050 0.045], 'String', 'LiDAR窗', ...
            'HorizontalAlignment', 'right', 'FontSize', 10);
        st.hWin = uicontrol(fig, 'Style', 'popupmenu', 'Units', 'normalized', ...
            'Position', [0.538 y1 0.065 0.055], 'String', winStr, 'Value', winVal, ...
            'Enable', lidarOn, ...
            'Callback', @(src, ~) setLidarWindow(get(src, 'Value'), src));

        % ---- 视角预设 ----
        st.hView = uicontrol(fig, 'Style', 'popupmenu', 'Units', 'normalized', ...
            'Position', [0.610 y1 0.085 0.055], ...
            'String', {'3D 视角', '俯视 X-Y', '侧视 Y-Z', '正视 X-Z'}, 'Value', 1, ...
            'Callback', @(src, ~) setViewPreset(get(src, 'Value'), src));

        uicontrol(fig, 'Style', 'checkbox', 'Units', 'normalized', ...
            'Position', [0.700 y1+0.005 0.075 0.045], 'String', '锁定视角', ...
            'FontSize', 10, 'Tag', 'lock', 'Callback', @(src, ~) toggleLock(get(src, 'Value')));

        uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
            'Position', [0.780 y1 0.060 0.055], 'String', '重置视角', ...
            'FontSize', 10, 'Callback', @(~, ~) resetView());

        txtInfo = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.900 y1+0.005 0.095 0.05], 'String', '', ...
            'HorizontalAlignment', 'left', 'FontSize', 9);

        % ---- 取点按钮 ----
        btnPick = uicontrol(fig, 'Style', 'togglebutton', 'Units', 'normalized', ...
            'Position', [0.845 y1 0.050 0.055], 'String', '取点', ...
            'FontSize', 10, 'Callback', @(src, ~) togglePick(get(src, 'Value')));

        % ---- 底部：取点读数 ----
        st.txtPick = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.020 0.012 0.96 0.058], ...
            'String', {'点「取点」后单击图上的点，读数会显示在这里（连点两次可测两点直线距离）'}, ...
            'HorizontalAlignment', 'left', 'FontSize', 10, 'ForegroundColor', [0.15 0.25 0.45]);

        st.dcm = datacursormode(fig);
        set(st.dcm, 'UpdateFcn', @(~, ev) onCursor(ev), 'SnapToDataVertex', 'on', 'Enable', 'off');

        % ---- 第二行：雷达速度门限 / 近景高亮 / 深度去噪 / X 平移 ----
        y2 = 0.068;
        uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.020 y2+0.004 0.075 0.042], 'String', '速度门限', ...
            'HorizontalAlignment', 'left', 'FontSize', 10, 'FontWeight', 'bold');
        st.hV = uicontrol(fig, 'Style', 'slider', 'Units', 'normalized', ...
            'Position', [0.100 y2 0.240 0.05], 'Min', 0, 'Max', 1.2, 'Value', st.vmin, ...
            'Callback', @(src, ~) setVmin(get(src, 'Value')));
        st.txtV = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.345 y2+0.004 0.125 0.042], ...
            'String', sprintf('%.2f m/s', st.vmin), ...
            'HorizontalAlignment', 'left', 'FontSize', 10);
        if st.vmin <= 0.01
            set(st.txtV, 'String', '0.00 m/s（全部）');
        end
        uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
            'Position', [0.475 y2 0.050 0.05], 'String', '归零', ...
            'FontSize', 10, 'Callback', @(~, ~) resetVmin());

        st.hFg = uicontrol(fig, 'Style', 'checkbox', 'Units', 'normalized', ...
            'Position', [0.535 y2+0.004 0.115 0.042], 'String', '近景高亮', ...
            'FontSize', 10, 'Value', double(st.showFg), 'Enable', lidarOn, ...
            'Callback', @(src, ~) toggleFg(get(src, 'Value'), src));

        st.hDepthFilter = uicontrol(fig, 'Style', 'checkbox', 'Units', 'normalized', ...
            'Position', [0.650 y2+0.004 0.085 0.042], 'String', '清理后深度', ...
            'FontSize', 10, 'Value', double(st.depthFilter), ...
            'Callback', @(src, ~) toggleDepthFilter(get(src, 'Value'), src));

        % X 平移（仅用于检查偏移量，不改数据）
        uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.735 y2+0.004 0.040 0.042], 'String', 'X 平移', ...
            'HorizontalAlignment', 'left', 'FontSize', 10);
        st.hXs = uicontrol(fig, 'Style', 'slider', 'Units', 'normalized', ...
            'Position', [0.775 y2 0.125 0.05], 'Min', -2, 'Max', 2, 'Value', 0, ...
            'Callback', @(src, ~) setXShift(get(src, 'Value')));
        st.txtXs = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.905 y2+0.004 0.090 0.042], 'String', '0.00 m', ...
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
        if ~isempty(P)
            keep = cloud_in_limits(P(:, 1:3), st.lim);
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

        % ---- 深度点云：单窗可切原始/清理，融合始终使用清理后主点云 ----
        Dclean = finite_xyz(S.depthPts{idx});
        Draw = Dclean;
        if isfield(S, 'depthPtsRaw') && numel(S.depthPtsRaw) >= idx
            Draw = finite_xyz(S.depthPtsRaw{idx});
        end
        if st.depthFilter, D = Dclean; else, D = Draw; end
        if isempty(D)
            set(hDep, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
        else
            set(hDep, 'XData', D(:, 1), 'YData', D(:, 2), 'ZData', D(:, 3));
        end
        set(hRadarOverlay, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
        if st.depthFilter
            depthMode = '清理后';
        else
            depthMode = '原始审计';
        end
        title(ax2, sprintf('深度相机点云（%s）   帧 %d   %d 点', ...
            depthMode, S.depthFrame(idx), size(D, 1)));

        % ---- MID-70：以当前雷达帧为参考累计时间窗（长窗=背景，短窗=近景）----
        L = zeros(0, 3);          % 体素平均后，MID-70 原生坐标
        LR = zeros(0, 3);         % 体素平均后，雷达坐标（长窗）
        LRfg = zeros(0, 3);       % 体素平均后，雷达坐标（近景短窗前景）
        LRbg = zeros(0, 3);       % 长窗中排除短窗后的静态背景
        reflectivity = zeros(0, 1);
        nLidarRaw = 0;
        if st.lidar.available
            rowIndices = st.lidar.rowsForFrame{idx};
            fgRows = st.lidar.rowsFgForFrame{idx};
            hasLR = st.lidar.extrinsicsAvailable;
            nRows = numel(rowIndices);
            partL = cell(nRows, 1);
            partLR = cell(nRows, 1);
            partR = cell(nRows, 1);
            rowIsFg = false(nRows, 1);
            ri = 0;
            for rowIndex = rowIndices(:).'
                ri = ri + 1;
                if isempty(st.lidar.pointsCache{rowIndex})
                    batch = read_livox_bin(st.lidar.files{rowIndex});
                    st.lidar.pointsCache{rowIndex} = double(batch.points);
                    st.lidar.reflectivityCache{rowIndex} = double(batch.reflectivity(:));
                    if hasLR
                        st.lidar.radarPointsCache{rowIndex} = transform_lidar_points( ...
                            st.lidar.pointsCache{rowIndex}, st.lidar.extrinsics);
                    end
                end
                pc = st.lidar.pointsCache{rowIndex};
                if size(pc, 2) >= 3
                    partL{ri} = pc(:, 1:3);
                else
                    partL{ri} = zeros(0, 3);
                end
                partR{ri} = st.lidar.reflectivityCache{rowIndex};
                if hasLR, partLR{ri} = st.lidar.radarPointsCache{rowIndex}; end
                rowIsFg(ri) = any(rowIndex == fgRows);
            end
            Lall = vertcat(partL{:});
            Rall = vertcat(partR{:});
            if hasLR
                LRall = vertcat(partLR{:});
            else
                LRall = zeros(0, 3);
            end
            counts = cellfun(@(a) size(a, 1), partL);
            ptIsFg = repelem(rowIsFg, counts);
            nLidarRaw = size(Lall, 1);

            % 体素平均：降噪、去重叠；L 与 LR 同一分组，两个窗格保持一致
            [L, LR, reflectivity] = lidar_voxel(Lall, LRall, Rall, st.lidarVoxel);
            if hasLR && any(~ptIsFg)
                [~, LRbg] = lidar_voxel(Lall(~ptIsFg, :), LRall(~ptIsFg, :), ...
                    Rall(~ptIsFg), st.lidarVoxel);
            end
            if st.showFg && nLidarRaw > 0 && any(ptIsFg) && ~all(ptIsFg)
                if hasLR
                    LRsub = LRall(ptIsFg, :);
                else
                    LRsub = zeros(0, 3);
                end
                [~, LRfg] = lidar_voxel(Lall(ptIsFg, :), LRsub, Rall(ptIsFg), st.lidarVoxel);
                LRfg = foreground_from_background(LRfg, LRbg, st.lidarVoxel, st.lidarFgMinDist);
            elseif st.showFg && any(ptIsFg) && isempty(LRbg) && hasLR
                % 没有可用的背景批次时保留短窗，但在标题/颜色上仍视为候选前景。
                LRfg = LR;
            end
            st.lidar = evict_lidar_cache(st.lidar, rowIndices, st.lidarCacheBatches);

            if ~isempty(L)
                displayIndices = display_sample_indices(size(L, 1), st.maxLidarDisplayPoints);
                set(hLidar, 'XData', L(displayIndices, 1), 'YData', L(displayIndices, 2), ...
                    'ZData', L(displayIndices, 3));
            else
                set(hLidar, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
            end
            if st.lidar.valid(idx)
                deltaText = sprintf('Δt %+.1f ms', st.lidar.dtMs(idx));
            else
                deltaText = sprintf('Δt %+.1f ms（偏大）', st.lidar.dtMs(idx));
            end
            if strcmp(st.lidarAlign, 'center')
                alignText = '居中';
            else
                alignText = '尾随';
            end
            title(ax3, sprintf('MID-70  窗 %.0f ms（%s）  %d 点 → 体素后 %d  %s', ...
                st.lidar.windowMs, alignText, nLidarRaw, size(L, 1), deltaText));
        end

        if st.lidar.available
            render_fusion_3d(Dclean, P, lab, LR, LRfg, idx);
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

    function setLidarWindow(v, src)
        %现场切换 MID-70 累计时间窗（不重新读文件，缓存保留）
        st.lidarWindowMs = st.winOpts(v);
        if st.lidar.available
            st.lidar = lidar_assign_windows(st.lidar, S, st.lidarWindowMs, ...
                st.lidarFgMs, st.lidarAlign);
            render(st.k);
        end
        releaseFocus(src);
    end

    function toggleFg(v, src)
        st.showFg = logical(v);
        render(st.k);
        releaseFocus(src);
    end

    function toggleDepthFilter(v, src)
        % 只切换第二窗显示；融合固定使用 compute_take 生成的清理后主点云。
        st.depthFilter = logical(v);
        render(st.k);
        releaseFocus(src);
    end

    function setViewPreset(v, src)
        %四个窗格同时切到预设视角。俯视/侧视用来看深度方向有没有错位或双层。
        presets = [-32 22; 0 90; 90 0; 0 0];
        vw = presets(v, :);
        st.view = vw;
        axesList = [ax1 ax2];
        if st.lidar.available, axesList = [axesList ax3 ax4]; end
        for axisHandle = axesList
            set(axisHandle, 'View', vw);
        end
        st.lastView = repmat(vw, numel(axesList), 1);
        releaseFocus(src);
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
        if isfield(st, 'hView') && ishandle(st.hView), set(st.hView, 'Value', 1); end
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

    function render_fusion_3d(depthClean, P, lab, LR, LRfg, idx)
        if isfield(st, 'offlineFusion') && st.offlineFusion.valid
            fused = st.offlineFusion.demo.fusedPc;
            set_fusion_cloud(hFusionDepth, display_sample(fused.xyz(fused.sensor_id == 1, :), st.maxFusionDepthPoints));
            set_fusion_cloud(hFusionLidar, display_sample(fused.xyz(fused.sensor_id == 2, :), st.maxFusionLidarPoints));
            set_fusion_cloud(hFusionLidarFg, zeros(0, 3));
            set_fusion_cloud(hFusionRadar, display_sample(fused.xyz(fused.sensor_id == 3, :), 1000));
            draw_offline_centroids(st.offlineFusion.demo);
            draw_guided_board_return(st.offlineFusion.demo);
            title(ax4, sprintf('离线三路融合 | Depth %d / MID-70 %d / 雷达 %d', ...
                nnz(fused.sensor_id == 1), nnz(fused.sensor_id == 2), nnz(fused.sensor_id == 3)));
            return;
        end
        if ~st.fusionAvailable
            set_fusion_cloud(hFusionDepth, zeros(0, 3));
            set_fusion_cloud(hFusionLidar, zeros(0, 3));
            set_fusion_cloud(hFusionLidarFg, zeros(0, 3));
            set_fusion_cloud(hFusionRadar, zeros(0, 3));
            draw_radar_centroids([], []);
            title(ax4, '三路叠加：未启用有效 MID-70→雷达外参');
            return;
        end
        depthRadar = depthClean(:, 1:min(3, size(depthClean, 2)));
        radarPoints = P(:, 1:min(3, size(P, 2)));
        depthDisplay = display_sample(depthRadar, st.maxFusionDepthPoints);
        lidarDisplay = display_sample(LR, st.maxFusionLidarPoints);
        useFg = st.showFg && ~isempty(LRfg);
        if useFg
            % 长窗淡色当背景，近景亮色叠在上面
            set(hFusionLidar, 'SizeData', 3, 'MarkerFaceColor', [0.55 0.70 0.98], ...
                'MarkerFaceAlpha', 0.30);
            set_fusion_cloud(hFusionLidarFg, display_sample(LRfg, st.maxFusionLidarPoints));
        else
            set(hFusionLidar, 'SizeData', 4, 'MarkerFaceColor', [0.05 0.32 0.95], ...
                'MarkerFaceAlpha', 0.55);
            set_fusion_cloud(hFusionLidarFg, zeros(0, 3));
        end
        set_fusion_cloud(hFusionDepth, depthDisplay);
        set_fusion_cloud(hFusionLidar, lidarDisplay);
        set_fusion_cloud(hFusionRadar, radarPoints);
        draw_radar_centroids(P, lab);
        title(ax4, sprintf('三路融合  帧 %d   雷达 %d / 深度 %d / MID-70 %d（窗 %.0f ms）', ...
            S.frames(idx), size(radarPoints, 1), size(depthRadar, 1), size(LR, 1), ...
            st.lidar.windowMs));
    end

    function draw_guided_board_return(demo)
        if isempty(hFusionGuidedRadar), return; end
        if isfield(demo,'planeAnalysis') && isfield(demo.planeAnalysis,'guidedRadarBoard') && ...
                demo.planeAnalysis.guidedRadarBoard.valid
            set_fusion_cloud(hFusionGuidedRadar,demo.planeAnalysis.guidedRadarBoard.point);
        else
            set_fusion_cloud(hFusionGuidedRadar,zeros(0,3));
        end
    end

    function draw_offline_centroids(demo)
        if ~isempty(st.cenText), delete(st.cenText(ishandle(st.cenText))); st.cenText = []; end
        if ~isfield(demo, 'associations') || ~isfield(demo.associations, 'cluster') || isempty(demo.associations.cluster)
            set_fusion_cloud(hFusionRadarCen, zeros(0, 3)); return;
        end
        items = demo.associations.cluster; cen = vertcat(items.radar_centroid);
        set_fusion_cloud(hFusionRadarCen, cen);
        th = gobjects(numel(items), 1);
        for q=1:numel(items)
            label=sprintf('  C%d',items(q).cluster_index);
            if isfield(demo.planeAnalysis,'boardCandidateClusterIndices') && any(demo.planeAnalysis.boardCandidateClusterIndices == items(q).cluster_index)
                label=[label ' 板面'];
            end
            th(q)=text(ax4,cen(q,1),cen(q,2),cen(q,3),label,'FontSize',9,'FontWeight','bold','Color',[.7 0 0],'Clipping','on');
        end
        st.cenText=th;
    end

    function draw_radar_centroids(P, lab)
        %雷达每个簇画一个大号簇心标记并标编号；原始雷达点退为淡色小圈
        if ~isempty(st.cenText)
            delete(st.cenText(ishandle(st.cenText)));
            st.cenText = [];
        end
        if isempty(P) || isempty(lab) || ~any(lab > 0)
            if ~isempty(hFusionRadarCen), set_fusion_cloud(hFusionRadarCen, zeros(0, 3)); end
            return;
        end
        ids = unique(lab(lab > 0)).';
        cen = zeros(numel(ids), 3);
        for q = 1:numel(ids)
            cen(q, :) = mean(P(lab == ids(q), 1:3), 1);
        end
        set_fusion_cloud(hFusionRadarCen, cen);
        th = gobjects(numel(ids), 1);
        for q = 1:numel(ids)
            th(q) = text(ax4, cen(q, 1), cen(q, 2), cen(q, 3), sprintf('  %d', ids(q)), ...
                'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.7 0 0], 'Clipping', 'on');
        end
        st.cenText = th;
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

function points = finite_xyz(points)
if isempty(points), points = zeros(0,3); return; end
points = double(points(:,1:3));
points = points(all(isfinite(points),2),:);
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

function keep = cloud_in_limits(points, lim)
%CLOUD_IN_LIMITS Apply the fixed scene ROI to a point cloud.
if isempty(points)
    keep = false(0, 1);
    return;
end
keep = all(isfinite(points(:, 1:3)), 2);
for axisIndex = 1:3
    keep = keep & points(:, axisIndex) >= lim{axisIndex}(1) & ...
        points(:, axisIndex) <= lim{axisIndex}(2);
end
end

function points = prepare_display_cloud(points, lim, voxelM, minNeighbors)
%PREPARE_DISPLAY_CLOUD Crop, voxel-average, and optionally remove isolated voxels.
if isempty(points)
    points = zeros(0, 3);
    return;
end
points = double(points(:, 1:3));
keep = cloud_in_limits(points, lim);
points = points(keep, :);
if isempty(points) || voxelM <= 0
    return;
end
points = voxel_average_xyz(points, voxelM);
if minNeighbors > 0
    points = voxel_radius_filter(points, voxelM, minNeighbors);
end
end

function points = voxel_average_xyz(points, voxelM)
%VOXEL_AVERAGE_XYZ Stable absolute-coordinate voxel reduction for display.
if isempty(points) || voxelM <= 0
    return;
end
q = floor(points ./ voxelM);
[~, ~, group] = unique(q, 'rows');
count = accumarray(group, 1);
points = [accumarray(group, points(:, 1)), ...
          accumarray(group, points(:, 2)), ...
          accumarray(group, points(:, 3))] ./ count;
end

function points = voxel_radius_filter(points, voxelM, minNeighbors)
%VOXEL_RADIUS_FILTER Remove isolated voxels using a 26-neighbour radius.
%   This is a lightweight display-time equivalent of a radius outlier filter.
if isempty(points) || voxelM <= 0 || minNeighbors <= 0
    return;
end
q = floor(points ./ voxelM);
neighbours = zeros(size(q, 1), 1);
for dx = -1:1
    for dy = -1:1
        for dz = -1:1
            if dx == 0 && dy == 0 && dz == 0, continue; end
            neighbours = neighbours + ismember(q + [dx dy dz], q, 'rows');
        end
    end
end
points = points(neighbours >= minNeighbors, :);
end

function foreground = foreground_from_background(shortCloud, background, voxelM, minDistM)
%FOREGROUND_FROM_BACKGROUND Keep short-window voxels not occupied by long-window background.
%   This is deliberately conservative: if no background is available, the short
%   window is returned unchanged. It avoids treating a dense static wall as motion.
foreground = shortCloud;
if isempty(shortCloud) || isempty(background) || voxelM <= 0
    return;
end
shortCloud = double(shortCloud(:, 1:3));
background = double(background(:, 1:3));
shortQ = floor(shortCloud ./ voxelM);
backgroundQ = floor(background ./ voxelM);
radius = max(0, ceil(minDistM / voxelM));
occupied = false(size(shortQ, 1), 1);
for dx = -radius:radius
    for dy = -radius:radius
        for dz = -radius:radius
            shifted = shortQ + [dx dy dz];
            occupied = occupied | ismember(shifted, backgroundQ, 'rows');
        end
    end
end
foreground = shortCloud(~occupied, :);
end

function releaseFocus(h)
%下拉框/复选框点完后让出键盘焦点，空格、方向键才能继续控制播放。
try
    set(h, 'Enable', 'off');
    drawnow;
    set(h, 'Enable', 'on');
catch
end
end

% ================================================================
function [Lv, LRv, Rv] = lidar_voxel(L, LR, refl, voxelM)
%LIDAR_VOXEL 体素平均：同一体素内的点取均值（降噪并去掉重叠点）。
%   L 与 LR 用同一分组，保证 MID-70 原生坐标和雷达坐标两个窗格一一对应。
Lv = L; LRv = LR; Rv = refl;
if isempty(L) || voxelM <= 0, return; end
hasLR = ~isempty(LR) && size(LR, 1) == size(L, 1);
if numel(refl) ~= size(L, 1), refl = zeros(size(L, 1), 1); end
ok = all(isfinite(L), 2);
if ~all(ok)
    L = L(ok, :);
    refl = refl(ok);
    if hasLR, LR = LR(ok, :); end
end
if isempty(L)
    Lv = zeros(0, 3); LRv = zeros(0, 3); Rv = zeros(0, 1);
    return;
end
g = floor((L - min(L, [], 1)) ./ voxelM) + 1;           % 每点所在体素的整数下标
dim = max(g, [], 1);
key = g(:, 1) + dim(1) .* ((g(:, 2) - 1) + dim(2) .* (g(:, 3) - 1));
[~, ~, j] = unique(key);
cnt = accumarray(j, 1);
Lv = [accumarray(j, L(:, 1)), accumarray(j, L(:, 2)), accumarray(j, L(:, 3))] ./ cnt;
if hasLR
    LRv = [accumarray(j, LR(:, 1)), accumarray(j, LR(:, 2)), accumarray(j, LR(:, 3))] ./ cnt;
else
    LRv = zeros(0, 3);
end
Rv = accumarray(j, refl(:)) ./ cnt;
end

function info = evict_lidar_cache(info, keepRows, maxBatches)
%已缓存的批次太多时，清掉不在当前窗口里的，防止内存一直涨。
filled = find(~cellfun(@isempty, info.pointsCache));
if numel(filled) <= maxBatches, return; end
drop = setdiff(filled, keepRows(:).');
info.pointsCache(drop) = {[]};
info.reflectivityCache(drop) = {[]};
info.radarPointsCache(drop) = {[]};
end

% ================================================================
function info = lidar_index(S, windowMs, fgMs, align)
%以处理结果中的雷达时间为参考，选择最近 MID-70 批次或累计时间窗。
n = numel(S.frames);
info = struct('available', false, 'batchForFrame', zeros(n, 1), ...
    'rowsForFrame', {cell(n, 1)}, 'rowsFgForFrame', {cell(n, 1)}, ...
    'dtMs', nan(n, 1), 'valid', false(n, 1), ...
    'files', {{}}, 'batches', [], 'timeMs', [], 'windowMs', windowMs, 'fgMs', fgMs, ...
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
    info.timeMs = lidarTime(:);
    info.pointsCache = cell(numel(files), 1);
    info.reflectivityCache = cell(numel(files), 1);
    info.radarPointsCache = cell(numel(files), 1);
    for idx = 1:n
        [delta, nearest] = min(abs(lidarTime - double(S.radarT(idx))));
        info.batchForFrame(idx) = batches(nearest);
        info.dtMs(idx) = lidarTime(nearest) - double(S.radarT(idx));
        info.valid(idx) = delta <= 30;
    end
    info = lidar_assign_windows(info, S, windowMs, fgMs, align);
    info.available = all(cellfun(@isfile, info.files));
    if info.available, info.lim = lidar_limits(info.files); end
catch ex
    warning('view_take:LiDARUnavailable', 'LiDAR 未载入：%s', ex.message);
end
end

function info = lidar_assign_windows(info, S, windowMs, fgMs, align)
%LIDAR_ASSIGN_WINDOWS 给每个雷达帧分配 MID-70 批次：长窗（背景）和短窗（近景）。
%   align = 'center'   窗口以当前雷达帧时间为中心 [t-W/2, t+W/2]（离线回放，点云时间中心与雷达/深度对齐）
%   align = 'trailing' 窗口取当前帧之前 [t-W, t]（模拟实时预览，无未来数据）
%   窗口 <= 100 ms 时只取最近的一个批次。
n = numel(S.frames);
t = info.timeMs(:);
info.windowMs = windowMs;
info.fgMs = fgMs;
info.rowsForFrame = cell(n, 1);
info.rowsFgForFrame = cell(n, 1);
for idx = 1:n
    t0 = double(S.radarT(idx));
    [~, nearest] = min(abs(t - t0));
    info.rowsForFrame{idx} = rows_in_window(t, t0, windowMs, align, nearest);
    info.rowsFgForFrame{idx} = rows_in_window(t, t0, min(fgMs, windowMs), align, nearest);
end
end

function rows = rows_in_window(t, t0, widthMs, align, nearest)
if widthMs <= 100
    rows = nearest;
    return;
end
if strcmpi(align, 'trailing')
    a = t0 - widthMs;
    b = t0;
else
    a = t0 - widthMs / 2;
    b = t0 + widthMs / 2;
end
rows = find(t >= a & t <= b).';
if isempty(rows), rows = nearest; end
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

function result = load_offline_fusion(path, takeName, resDir)
result = struct('valid',false);
if isempty(path)
    candidate = fullfile(fileparts(resDir),'static_fusion',[char(takeName) '_offline_fusion.mat']);
    if isfile(candidate), path=candidate; end
end
if isempty(path) || ~isfile(path), return; end
try
    loaded=load(path,'demo');
    if isfield(loaded,'demo') && isfield(loaded.demo,'fusedPc')
        result.valid=true; result.demo=loaded.demo; result.path=path;
    end
catch ex
    warning('view_take:OfflineFusionInvalid','离线融合文件无法读取：%s',ex.message);
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
                    'Name', '3.1 点云播放器');
if ~ok || isempty(idx), return; end
resFile = fullfile(list(idx).folder, list(idx).name);
end

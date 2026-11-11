function view_overlay(resFile, varargin)
%VIEW_OVERLAY 雷达点云投影到图像的坐标对齐工具
%
%   view_overlay                 % 弹窗选一个已算好的 take
%   view_overlay('..._res.mat')  % 直接指定
%   view_overlay(resFile, 'frame', 5, 'save', 'out.png')   % 无界面出图
%
% 三个画面：左=彩色照片+投影点，中=深度图(已镜像)+投影点，右=3D 点云
%
% 坐标约定：雷达为原点，X 右 / Y 前(视线方向) / Z 上。
% 默认相机在雷达正上方 0.10 m（t = [0 0 0.10]），姿态无旋转。
%
% 投影链：
%   P_cam = R' * (P_radar - t)          雷达体坐标系 → 相机体坐标系（轴同雷达）
%   → 光学系：x = X, y = -Z, z = Y
%   → u = fx*x/z + cx,  v = fy*y/z + cy
%
% 滑条：tx/ty/tz = 相机在雷达坐标系里的位置（米）
%       yaw/pitch/roll = 相机相对雷达的姿态（度，yaw 绕 Z 上，pitch 绕 X 右，roll 绕 Y 前）
%
% 快捷键：←/→ 逐帧   空格 播放/暂停   u 撤销   r 重置 3D 视角
% 按钮：重置(回标定初值) / 撤销 / 保存参数(写入 results\extrinsics.json)

% ============ 用户标定值（solve_t_all 解的 t） ============
K_USER = [491.4 655.7 319.5 239.5];   % Astra Pro 彩色口径
T0 = struct('tx', 0.000, 'ty', -0.072, 'tz', 0.130, 'yaw', 0, 'pitch', 0, 'roll', 0);
% t = [0.000, -0.072, 0.130] 是 solve_t_all 在 11-4 数据上逐组解后取的中位数
% 下面根据采集批次自动套默认姿态（只影响 yaw/pitch，t 固定）：
%   _13 批次（2m/2.5m/3m）：yaw=-4.3°  pitch=+5.4°
%   _16 批次（4m/5m/6m）：yaw=+1.5°  pitch=+10.0°
% ======================================================================

ip = inputParser;
ip.addParameter('frame', 1);
ip.addParameter('save', '');
ip.parse(varargin{:});
p = ip.Results;

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'algorithm', 'lib'), '-begin');   % pctile
resDir = paths().takesDir;

if nargin < 1 || isempty(resFile)
    resFile = pick_result(resDir);
    if isempty(resFile)
        fprintf('没有选中数据，退出。\n');
        return;
    end
end

S = load(resFile);
meta = S.metaOut;
n = numel(S.frames);
k = max(1, min(n, p.frame));

% ---------------- 内参：K_USER > calib.json > meta.json ----------------
K = K_USER; ksrc = '文件顶部 K_USER（棋盘格标定值）';
if isempty(K)
    cj = fullfile(meta.takeDir, 'calib.json');
    if isfile(cj)
        try
            j = jsondecode(fileread(cj));
            K = double([j.fx j.fy j.cx j.cy]);
            ksrc = 'calib.json（棋盘格标定值）';
        catch
        end
    end
end
if isempty(K)
    K = double([meta.fx meta.fy meta.cx meta.cy]);
    ksrc = 'meta.json（设备 FOV 估计值，非标定值）';
end
W = 640; H = 480;
Kd = K; Kd(3) = (W - 1) - K(3);   % 深度图已镜像，主点跟着翻

% 相机姿态按采集批次区分（11-04 实测）：
%   13:xx 的 2m/2.5m/3m 组：相机俯视约 20°，yaw 约 -10°
%   16:xx 的 4m/5m/6m 组：相机基本水平
nm = meta.takeName;
if contains(nm, '2米') || contains(nm, '3米')
    T0.pitch = 20; T0.yaw = -10;
end

st = struct();
st.k = k; st.n = n; st.playing = false; st.speed = 1.0;
st.K = K; st.Kd = Kd; st.ksrc = ksrc;
st.prm = T0;
st.hist = {T0};                    % 撤销栈：每次松手/重置都记一笔
st.imin = 0;                       % 雷达点强度门限（dB）
st.baseInterval = 0.4;
st.view = [-32 22];
st.lim = {[-3 3], [0 9], [-1.4 1.6]};
st.imgCache = struct('frame', -1, 'img', []);
st.grad = []; st.gradFrame = -1;
st.txtInfo = []; st.sld = [];

vis = 'off';
if isempty(p.save), vis = 'on'; end

fig = figure('Visible', vis, 'Color', 'w', 'Units', 'normalized', ...
    'Position', [0.02 0.06 0.96 0.88], 'NumberTitle', 'off', ...
    'Name', sprintf('雷达-相机投影对齐 — %s   t=[%.2f %.2f %.2f] yaw=%.1f°', ...
        meta.takeName, T0.tx, T0.ty, T0.tz, T0.yaw), 'MenuBar', 'figure', 'ToolBar', 'figure');

ax1 = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.03 0.34 0.30 0.60]);
ax2 = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.36 0.34 0.30 0.60]);
ax3 = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.69 0.34 0.28 0.60]);

hP1 = []; hP2 = []; hRadar = []; hDep = []; hCam = []; hImg1 = []; hImg2 = [];
sldPrm = struct(); txtPrm = struct();

fprintf('take: %s\n内参: fx=%.2f fy=%.2f cx=%.2f cy=%.2f （%s）\n', ...
    meta.takeName, K(1), K(2), K(3), K(4), ksrc);

build_axes();
build_controls();
render(st.k);

if ~isempty(p.save)
    outPath = p.save;
    try
        outPath = char(java.io.File(p.save).getCanonicalPath());
    catch
        if isempty(fileparts(p.save)), outPath = fullfile(pwd, p.save); end
    end
    try
        exportapp(fig, outPath);
    catch ex
        fprintf(2, 'exportapp 失败：%s，改用 print\n', ex.message);
        print(fig, outPath, '-dpng', '-r100');
    end
    close(fig);
    fprintf('已存图：%s\n', outPath);
    return;
end

tPlay = timer('ExecutionMode', 'fixedSpacing', 'Period', st.baseInterval, ...
              'TimerFcn', @(~, ~) onTick(), 'Name', 'view_overlay_play');
set(fig, 'CloseRequestFcn', @(~, ~) onClose());
set(fig, 'KeyPressFcn', @(src, ev) onKey(ev));
set(fig, 'WindowButtonUpFcn', @(~, ~) onUp());
set(fig, 'WindowScrollWheelFcn', @(src, ev) onWheel(ev));

% 回调全部注册完才开旋转（先开会吞掉回调）
rotate3d(fig, 'on');

% ================================================================
    function build_axes()
        % 左：彩色 + 投影（image 句柄只建一次，之后只换 CData，否则会盖住投影点）
        hImg1 = image(ax1, 'CData', zeros(H, W, 3, 'uint8'));
        hold(ax1, 'on');
        hP1 = plot(ax1, NaN, NaN, 'o', 'MarkerSize', 6, 'MarkerFaceColor', [0 1 1], ...
                   'MarkerEdgeColor', [1 0 0], 'LineWidth', 1.2);
        axis(ax1, 'image'); axis(ax1, 'off');
        title(ax1, '彩色 + 雷达投影');

        % 中：深度图（镜像后）+ 投影
        hImg2 = image(ax2, 'CData', zeros(H, W), 'CDataMapping', 'scaled');
        set(ax2, 'CLim', [0.3 8]);
        colormap(ax2, parula);
        hold(ax2, 'on');
        hP2 = plot(ax2, NaN, NaN, 'o', 'MarkerSize', 6, 'MarkerFaceColor', [1 0 1], ...
                   'MarkerEdgeColor', [1 1 1], 'LineWidth', 1.2);
        axis(ax2, 'image'); axis(ax2, 'off');
        title(ax2, '深度图 + 雷达投影');

        % 右：3D 点云
        set(ax3, 'XLim', st.lim{1}, 'YLim', st.lim{2}, 'ZLim', st.lim{3}, ...
                 'XLimMode', 'manual', 'YLimMode', 'manual', 'ZLimMode', 'manual', ...
                 'DataAspectRatio', [1 1 1], 'DataAspectRatioMode', 'manual', ...
                 'View', st.view, 'Box', 'on', 'GridAlpha', 0.15);
        hold(ax3, 'on');
        hDep = scatter3(ax3, NaN, NaN, NaN, 2, [0.7 0.75 0.85], 'filled');
        hRadar = scatter3(ax3, NaN, NaN, NaN, 22, NaN, 'filled');
        hCam = plot3(ax3, NaN, NaN, NaN, 'v', 'MarkerSize', 10, ...
                     'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');
        xlabel(ax3, 'X (m)'); ylabel(ax3, 'Y (m)'); zlabel(ax3, 'Z (m)');
        title(ax3, '3D 点云（色=雷达强度）');
        colormap(ax3, parula);
    end

    function build_controls()
        % ---- 参数滑条：左列 tx/ty/tz，右列 yaw/pitch/roll ----
        rows = [0.235 0.175 0.115];
        mkSlider('tx (m)', 'tx', -0.8, 0.8, rows(1), 0.035);
        mkSlider('ty (m)', 'ty', -0.8, 0.8, rows(2), 0.035);
        mkSlider('tz (m)', 'tz', -0.8, 0.8, rows(3), 0.035);
        mkSlider('yaw (°)', 'yaw', -30, 30, rows(1), 0.51);
        mkSlider('pitch(°)', 'pitch', -45, 45, rows(2), 0.51);
        mkSlider('roll (°)', 'roll', -15, 15, rows(3), 0.51);

        uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.035 0.295 0.9 0.035], 'String', '', ...
            'HorizontalAlignment', 'left', 'FontSize', 9, 'ForegroundColor', [0.3 0.3 0.3], ...
            'Tag', 'ksrc');

        % ---- 底部：帧导航 + 强度门限 + 按钮 ----
        yb = 0.05;
        uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
            'Position', [0.035 yb 0.035 0.06], 'String', '◀', ...
            'FontSize', 11, 'Callback', @(~, ~) step(-1));
        uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
            'Position', [0.075 yb 0.035 0.06], 'String', '▶', ...
            'FontSize', 11, 'Callback', @(~, ~) step(1));
        uicontrol(fig, 'Style', 'togglebutton', 'Units', 'normalized', ...
            'Position', [0.115 yb 0.055 0.06], 'String', '⏵', ...
            'FontSize', 11, 'Callback', @(~, ~) togglePlay());
        st.sld = uicontrol(fig, 'Style', 'slider', 'Units', 'normalized', ...
            'Position', [0.18 yb 0.26 0.06], 'Min', 1, 'Max', st.n, 'Value', st.k, ...
            'SliderStep', [1 / max(st.n - 1, 1), 5 / max(st.n - 1, 1)], ...
            'Callback', @(src, ~) jumpTo(round(get(src, 'Value'))));
        st.txtInfo = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.445 yb 0.09 0.055], 'String', sprintf('帧 %d/%d', st.k, st.n), ...
            'HorizontalAlignment', 'left', 'FontSize', 9);

        uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.545 yb 0.06 0.055], 'String', '强度≥dB', ...
            'HorizontalAlignment', 'right', 'FontSize', 9);
        uicontrol(fig, 'Style', 'slider', 'Units', 'normalized', ...
            'Position', [0.61 yb 0.12 0.06], 'Min', 0, 'Max', 50, 'Value', 0, ...
            'Callback', @(src, ~) setImin(get(src, 'Value')));
        uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.735 yb 0.045 0.055], 'String', '0', 'FontSize', 9, ...
            'Tag', 'iminval');

        uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
            'Position', [0.790 yb 0.062 0.06], 'String', '自动对齐', ...
            'FontSize', 9, 'FontWeight', 'bold', ...
            'Callback', @(~, ~) autoAlign());
        uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
            'Position', [0.855 yb 0.050 0.06], 'String', '重置', ...
            'FontSize', 10, 'Callback', @(~, ~) resetPrm());
        uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
            'Position', [0.908 yb 0.042 0.06], 'String', '撤销', ...
            'FontSize', 10, 'Callback', @(~, ~) undoPrm());
        uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
            'Position', [0.953 yb 0.038 0.06], 'String', '保存', ...
            'FontSize', 10, 'Callback', @(~, ~) savePrm());

        h = findobj(fig, 'Tag', 'ksrc');
        set(h, 'String', sprintf(['内参来源：%s   |   ' ...
            '滑条含义：t=相机在雷达坐标系位置（默认正上方 0.10 m），yaw/pitch/roll=相机姿态   |   ' ...
            'U 撤销 / ←→ 逐帧 / 空格 播放'], ksrc));
    end

    function mkSlider(label, field, lo, hi, y, x0)
        uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [x0 y 0.075 0.05], 'String', label, ...
            'HorizontalAlignment', 'left', 'FontSize', 9, 'FontWeight', 'bold');
        sldPrm.(field) = uicontrol(fig, 'Style', 'slider', 'Units', 'normalized', ...
            'Position', [x0 + 0.08 y 0.26 0.05], 'Min', lo, 'Max', hi, ...
            'Value', st.prm.(field), ...
            'SliderStep', [(hi - lo) / 200, (hi - lo) / 20], ...
            'Callback', @(src, ~) setPrm(field, get(src, 'Value')));
        txtPrm.(field) = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [x0 + 0.35 y 0.09 0.05], 'String', ...
            sprintf('%+.3f', st.prm.(field)), 'HorizontalAlignment', 'left', 'FontSize', 9);
    end

    function setPrm(field, val)
        st.prm.(field) = val;
        set(txtPrm.(field), 'String', sprintf('%+.3f', val));
        render(st.k);
    end

    function applyPrm(np)
        st.prm = np;
        f = fieldnames(np);
        for i = 1:numel(f)
            set(sldPrm.(f{i}), 'Value', np.(f{i}));
            set(txtPrm.(f{i}), 'String', sprintf('%+.3f', np.(f{i})));
        end
        render(st.k);
    end

    function onUp()
        % 松手时把当前参数记进撤销栈
        if ~isequal(st.prm, st.hist{end})
            st.hist{end+1} = st.prm; %#ok<AGROW>
        end
    end

    function undoPrm()
        if numel(st.hist) > 1
            st.hist(end) = [];
            applyPrm(st.hist{end});
        end
    end

    function resetPrm()
        if ~isequal(st.prm, T0)
            st.hist{end+1} = st.prm; %#ok<AGROW>
        end
        applyPrm(T0);
    end

    function savePrm()
        outFile = fullfile(paths().resultsDir, 'extrinsics.json');
        rec = struct('takeName', meta.takeName, 'tx', st.prm.tx, 'ty', st.prm.ty, ...
                     'tz', st.prm.tz, 'yawDeg', st.prm.yaw, 'pitchDeg', st.prm.pitch, ...
                     'rollDeg', st.prm.roll, 'K', st.K, 'Ksource', st.ksrc, ...
                     'frame', S.frames(st.k), 'savedAt', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
        all = struct();
        if isfile(outFile)
            try
                all = jsondecode(fileread(outFile));
                if ~iscell(all), all = {all}; end
            catch
                all = {};
            end
        end
        hit = false;
        for i = 1:numel(all)
            if strcmp(all{i}.takeName, meta.takeName)
                all{i} = rec; hit = true;
            end
        end
        if ~hit, all{end+1} = rec; end %#ok<AGROW>
        fid = fopen(outFile, 'w');
        fwrite(fid, jsonencode(all, 'PrettyPrint', true));
        fclose(fid);
        fprintf('参数已保存：%s\n', outFile);
        set(st.txtInfo, 'String', '已保存 ✓');
    end

    function setImin(v)
        st.imin = v;
        h = findobj(fig, 'Tag', 'iminval');
        set(h, 'String', sprintf('%d', round(v)));
        render(st.k);
    end

    function [u, v, ok] = project(P, prm, Kloc)
        % 雷达体坐标 → 像素。P: N×3
        t = [prm.tx, prm.ty, prm.tz];
        R = Rz(prm.yaw) * Rx(prm.pitch) * Ry(prm.roll);   % 相机体 → 雷达体
        Pc = (P - t) * R;                                 % 相机体坐标（X右 Y前 Z上）
        xo = Pc(:, 1); yo = -Pc(:, 3); zo = Pc(:, 2);     % 光学系：x右 y下 z前
        ok = zo > 0.25;
        u = nan(size(zo)); v = nan(size(zo));
        u(ok) = Kloc(1) .* xo(ok) ./ zo(ok) + Kloc(3);
        v(ok) = Kloc(2) .* yo(ok) ./ zo(ok) + Kloc(4);
        ok = ok & u >= 1 & u <= W - 2 & v >= 1 & v <= H - 2;
        u(~ok) = NaN; v(~ok) = NaN;
    end

    function render(idx)
        idx = max(1, min(st.n, idx));
        st.k = idx;

        % ---- 雷达点（强度过滤）----
        P = S.pts{idx};
        if ~isempty(P) && st.imin > 0
            P = P(P(:, 4) >= st.imin, :);
        end

        % ---- 左：彩色 + 投影 ----
        img = color_image(idx);
        if ~isempty(img)
            set(hImg1, 'CData', img);
            if st.gradFrame ~= S.colorFrame(idx)
                g = double(img); if size(g, 3) > 1, g = mean(g, 3); end
                [gx, gy] = gradient(g);
                st.grad = sqrt(gx.^2 + gy.^2);   % 边缘强度图：强散射点多落在轮廓上
                st.gradFrame = S.colorFrame(idx);
            end
        else
            set(hImg1, 'CData', zeros(H, W, 3, 'uint8'));
            st.grad = []; st.gradFrame = -1;
        end
        if isempty(P)
            set(hP1, 'XData', NaN, 'YData', NaN);
            n1 = 0;
        else
            [u1, v1, ok1] = project(P(:, 1:3), st.prm, st.K);
            set(hP1, 'XData', u1(ok1), 'YData', v1(ok1));
            n1 = nnz(ok1);
        end
        title(ax1, sprintf('彩色 + 雷达投影（%d 点）   帧 %d', n1, S.frames(idx)));

        % ---- 中：深度图 + 投影 ----
        dNo = S.depthFrame(idx);
        dp = fullfile(meta.takeDir, 'depth', sprintf('%06d.png', dNo));
        if isfile(dp)
            D = imread(dp);
            if size(D, 3) > 1, D = D(:, :, 1); end
            D = fliplr(D);                       % 和彩色/雷达 X 方向对齐
            set(hImg2, 'CData', double(D) * 0.001);
            set(ax2, 'CLim', [0.3 8]);
        end
        if isempty(P)
            set(hP2, 'XData', NaN, 'YData', NaN);
            n2 = 0;
        else
            [u2, v2, ok2] = project(P(:, 1:3), st.prm, st.Kd);
            set(hP2, 'XData', u2(ok2), 'YData', v2(ok2));
            n2 = nnz(ok2);
        end
        title(ax2, sprintf('深度图 + 雷达投影（%d 点）   深度帧 %d   Δt %+.0f ms', ...
            n2, dNo, S.depthDt(idx)));

        % ---- 右：3D 点云 ----
        Dc = S.depthPts{idx};
        if ~isempty(Dc)
            % 深度点云是从"相机光轴水平"假设反投影的；按当前姿态转回世界系，
            % 否则改 pitch/yaw 时它不跟着动，会与雷达点错开
            Rm = Rz(st.prm.yaw) * Rx(st.prm.pitch) * Ry(st.prm.roll);
            Dc = double(Dc(:, 1:3)) * Rm' + [st.prm.tx, st.prm.ty, st.prm.tz];
            smp = max(1, floor(size(Dc, 1) / 6000));
            set(hDep, 'XData', Dc(1:smp:end, 1), 'YData', Dc(1:smp:end, 2), ...
                      'ZData', Dc(1:smp:end, 3));
        else
            set(hDep, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
        end
        if isempty(P)
            set(hRadar, 'XData', NaN, 'YData', NaN, 'ZData', NaN, 'CData', NaN);
        else
            set(hRadar, 'XData', P(:, 1), 'YData', P(:, 2), 'ZData', P(:, 3), ...
                        'CData', P(:, 4));
            caxis(ax3, [0, max(pctile(P(:, 4), 98), 30)]);
        end
        set(hCam, 'XData', st.prm.tx, 'YData', st.prm.ty, 'ZData', st.prm.tz);
        title(ax3, sprintf('3D 点云（色=雷达强度）   t=[%+.2f %+.2f %+.2f]  y/p/r=[%.1f %.1f %.1f]°', ...
            st.prm.tx, st.prm.ty, st.prm.tz, st.prm.yaw, st.prm.pitch, st.prm.roll));

        if ~isempty(st.sld), set(st.sld, 'Value', idx); end
        if ~isempty(st.txtInfo)
            set(st.txtInfo, 'String', sprintf('帧 %d/%d', idx, st.n));
        end
        drawnow limitrate;
    end

    function sc = alignScore(prm)
        % 评分：雷达点投影后落在照片边缘图上的平均强度（越大越贴合轮廓）
        sc = -1e9;
        if isempty(st.grad), return; end
        P = S.pts{st.k};
        if isempty(P), return; end
        if st.imin > 0, P = P(P(:, 4) >= st.imin, :); end
        if size(P, 1) < 30, return; end
        [uu, vv, ok] = project(P(:, 1:3), prm, st.K);
        if nnz(ok) < 30, return; end
        gg = interp2(st.grad, uu(ok), vv(ok), 'linear', 0);
        sc = mean(gg(:));
    end

    function best = searchField(prm, fld, lo, hi, step)
        best = prm; bs = alignScore(prm);
        for val = lo:step:hi
            q = prm; q.(fld) = val;
            s = alignScore(q);
            if s > bs, bs = s; best = q; end
        end
    end

    function autoAlign()
        stopIfPlaying();
        if isempty(st.grad)
            set(st.txtInfo, 'String', '无照片，无法自动对齐');
            return;
        end
        set(st.txtInfo, 'String', '自动对齐中…'); drawnow;
        prm = st.prm;
        % 第一轮：全范围粗搜（pitch 最关键，先搜）
        prm = searchField(prm, 'pitch', -45, 45, 3);
        prm = searchField(prm, 'yaw', -25, 25, 2.5);
        prm = searchField(prm, 'tz', -0.7, 0.7, 0.05);
        prm = searchField(prm, 'tx', -0.7, 0.7, 0.05);
        % 第二轮：在最优点附近细化
        for rnd = 1:2
            prm = searchField(prm, 'pitch', prm.pitch - 3, prm.pitch + 3, 0.5);
            prm = searchField(prm, 'yaw', prm.yaw - 3, prm.yaw + 3, 0.5);
            prm = searchField(prm, 'tz', prm.tz - 0.06, prm.tz + 0.06, 0.01);
            prm = searchField(prm, 'tx', prm.tx - 0.06, prm.tx + 0.06, 0.01);
            prm = searchField(prm, 'ty', prm.ty - 0.15, prm.ty + 0.15, 0.02);
        end
        if ~isequal(prm, st.hist{end}), st.hist{end+1} = prm; end %#ok<AGROW>
        applyPrm(prm);
        fprintf('自动对齐：tx=%+.2f ty=%+.2f tz=%+.2f  yaw=%+.1f° pitch=%+.1f° roll=%+.1f°\n', ...
            prm.tx, prm.ty, prm.tz, prm.yaw, prm.pitch, prm.roll);
        set(st.txtInfo, 'String', '自动对齐完成 ✓');
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
        pth = fullfile(meta.takeDir, 'color', sprintf('%06d.png', fno));
        if ~isfile(pth), return; end
        C = imread(pth);
        st.imgCache = struct('frame', fno, 'img', C);
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
            start(tPlay);
        end
    end

    function stopPlay()
        st.playing = false;
        if isvalid(tPlay) && strcmp(tPlay.Running, 'on'), stop(tPlay); end
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

    function onKey(ev)
        switch ev.Key
            case 'space', togglePlay();
            case 'leftarrow', step(-1);
            case 'rightarrow', step(1);
            case 'u', undoPrm();
            case 'r'
                st.view = [-32 22];
                set(ax3, 'View', st.view, ...
                         'XLim', st.lim{1}, 'YLim', st.lim{2}, 'ZLim', st.lim{3});
        end
    end

    function onWheel(ev)
        % 滚轮只缩放 3D 视图
        pt = get(fig, 'CurrentPoint');
        p3 = get(ax3, 'Position');
        if pt(1) < p3(1) || pt(1) > p3(1) + p3(3), return; end
        f = 0.90 ^ double(ev.VerticalScrollCount);
        xl = get(ax3, 'XLim'); yl = get(ax3, 'YLim'); zl = get(ax3, 'ZLim');
        set(ax3, 'XLim', mean(xl) + (xl - mean(xl)) * f, ...
                 'YLim', mean(yl) + (yl - mean(yl)) * f, ...
                 'ZLim', mean(zl) + (zl - mean(zl)) * f);
    end

    function onClose()
        try
            st.playing = false;
            if isvalid(tPlay), stop(tPlay); delete(tPlay); end
        catch
        end
        delete(fig);
    end
end

% ================================================================
function R = Rz(a)
s = sind(a); c = cosd(a);
R = [c -s 0; s c 0; 0 0 1];
end

function R = Ry(a)
s = sind(a); c = cosd(a);
R = [c 0 s; 0 1 0; -s 0 c];
end

function R = Rx(a)
s = sind(a); c = cosd(a);
R = [1 0 0; 0 c -s; 0 s c];
end

% ================================================================
function resFile = pick_result(resDir)
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
[idx, ok] = listdlg('PromptString', '选择要对齐的数据：', ...
                    'SelectionMode', 'single', ...
                    'ListString', names, ...
                    'ListSize', [560 260], ...
                    'Name', '雷达-相机投影对齐');
if ~ok || isempty(idx), return; end
resFile = fullfile(list(idx).folder, list(idx).name);
end

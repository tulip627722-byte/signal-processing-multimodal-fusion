function visualize_alignment()
%VISUALIZE_ALIGNMENT 用 solve_t_all 逐组最优姿态，投影雷达点并按残差着色
%
% 用法：visualize_alignment

here = fileparts(mfilename('fullpath'));
cd(here);
addpath(here, fullfile(here,'tools'), fullfile(here,'algorithm'), fullfile(here,'algorithm','lib'));

P = paths();
K = [491.4 655.7 319.5 239.5];  W = 640; H = 480;
Kd = K; Kd(3) = (W - 1) - K(3);

% 读 solve_t_all 结果
js = jsondecode(fileread(fullfile(P.resultsDir, 'tall_result.json')));
tAvg = js.t_weighted_mean;
tMed = js.t_median;

fprintf('\n==== 标定结果 ====\n');
fprintf('加权平均 t = [%+.4f, %+.4f, %+.4f] m\n', tAvg);
fprintf('中位数   t = [%+.4f, %+.4f, %+.4f] m\n', tMed);
fprintf('参与 %d 组合格\n\n', numel(js.groups));

% 选展示组（所有合格 + 室内最好的 + 所有 res.mat）
files = dir(fullfile(P.takesDir, '*_res.mat'));

% 弹交互式窗口
fig = figure('Visible', 'on', 'Color', 'w', 'Units', 'normalized', ...
    'Position', [0.02 0.06 0.96 0.88], 'MenuBar', 'figure', 'ToolBar', 'figure', ...
    'Name', sprintf('雷达-相机对齐可视化  固定中位数t=[%.3f %.3f %.3f]  姿态逐组解', tMed(1), tMed(2), tMed(3)));

ax1 = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.03 0.34 0.30 0.60]);
ax2 = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.36 0.34 0.30 0.60]);
ax3 = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.69 0.34 0.28 0.60]);

hImg1 = image(ax1, 'CData', zeros(H, W, 3, 'uint8')); hold(ax1, 'on');
hSc1 = scatter(ax1, NaN, NaN, 8, NaN, 'filled'); set(hSc1, 'LineWidth', 0.5, 'MarkerEdgeColor', 'k');
axis(ax1, 'image'); axis(ax1, 'off');

hImg2 = image(ax2, 'CData', zeros(H, W), 'CDataMapping', 'scaled');
set(ax2, 'CLim', [0.3 8]); colormap(ax2, parula); hold(ax2, 'on');
hSc2 = scatter(ax2, NaN, NaN, 8, NaN, 'filled'); set(hSc2, 'LineWidth', 0.5);
axis(ax2, 'image'); axis(ax2, 'off');

set(ax3, 'XLim', [-3 3], 'YLim', [0 9], 'ZLim', [-1.4 1.6], ...
    'DataAspectRatio', [1 1 1], 'View', [-32 22], 'Box', 'on', 'GridAlpha', 0.15); hold(ax3, 'on');
hDep = scatter3(ax3, NaN, NaN, NaN, 2, [0.7 0.75 0.85], 'filled');
hRadar = scatter3(ax3, NaN, NaN, NaN, 12, NaN, 'filled');
hCam = plot3(ax3, NaN, NaN, NaN, 'v', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');
hOrig = plot3(ax3, 0, 0, 0, 's', 'MarkerSize', 14, 'MarkerFaceColor', [1 0.5 0], 'MarkerEdgeColor', 'k');
xlabel(ax3, 'X'); ylabel(ax3, 'Y(前)'); zlabel(ax3, 'Z(上)');

% ---- 顶部：选择下拉框 + 信息 ----
names = cellfun(@(s) strrep(s, '_res.mat', ''), {files.name}, 'UniformOutput', false);
[~, ord] = sort(names); names = names(ord); files = files(ord);

uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.035 0.30 0.12 0.03], 'String', '选数据:', 'FontSize', 9, 'FontWeight', 'bold');
ddl = uicontrol(fig, 'Style', 'popupmenu', 'Units', 'normalized', ...
    'Position', [0.155 0.30 0.45 0.035], 'String', names, 'Value', 1, ...
    'Callback', @(src,~) loadAndRender(get(src,'Value')));

txtInfo = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.62 0.30 0.36 0.03], 'FontSize', 9, 'HorizontalAlignment', 'left', ...
    'ForegroundColor', [0.2 0.2 0.7]);

% ---- 底部帧滑条 ----
yb = 0.05;
uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.035 yb 0.035 0.06], 'String', '◀', 'FontSize', 11, ...
    'Callback', @(~,~) step(-1));
uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.075 yb 0.035 0.06], 'String', '▶', 'FontSize', 11, ...
    'Callback', @(~,~) step(1));
sldF = uicontrol(fig, 'Style', 'slider', 'Units', 'normalized', ...
    'Position', [0.115 yb 0.40 0.06], 'Min', 1, 'Max', 1, 'Value', 1, ...
    'SliderStep', [0.01, 0.05], 'Callback', @(src,~) renderFrame(round(get(src,'Value'))));
uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.52 yb 0.15 0.055], 'FontSize', 9, 'String', '');

% ---- 残差阈值滑条 ----
uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.68 yb 0.13 0.055], 'FontSize', 9, 'HorizontalAlignment', 'right', ...
    'String', '残差阈值cm');
sldT = uicontrol(fig, 'Style', 'slider', 'Units', 'normalized', ...
    'Position', [0.815 yb 0.12 0.06], 'Min', 1, 'Max', 50, 'Value', 15, ...
    'SliderStep', [0.5, 2], 'Callback', @(src,~) renderFrame(st.k));
uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.94 yb 0.05 0.055], 'FontSize', 9, 'String', '15');

% ---- 状态栏 ----
uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.035 0.005 0.93 0.03], 'FontSize', 8, ...
    'ForegroundColor', [0.4 0.4 0.4], ...
    'String', '绿色=残差≤阈值cm  黄色=2×阈值  红色=>2×阈值   |   ← → 翻帧');

% ---- 状态 ----
st = struct('S', [], 'nm', '', 'k', 1, 'n', 0, 'prm', [], 'resid_vals', [], 'mask_ok', [], 'resid', [], 'cmap', []);

set(fig, 'KeyPressFcn', @(src,ev) onKey(ev));

loadAndRender(1);

% ================================================================
    function loadAndRender(idx)
        st.S = load(fullfile(files(idx).folder, files(idx).name));
        st.nm = st.S.metaOut.takeName;
        st.n = numel(st.S.frames);
        st.k = min(st.n, 10);   % 从第 10 帧开始（通常最稳定）

        % 强制用固定中位数 t，只用 solve_t_all 解的 yaw/pitch/roll
        prm = struct('tx', tMed(1), 'ty', tMed(2), 'tz', tMed(3), 'yaw', 0, 'pitch', 0, 'roll', 0);
        found = false;
        for gi = 1:numel(js.groups)
            if strcmp(js.groups(gi).name, st.nm)
                prm.yaw = js.groups(gi).yaw; prm.pitch = js.groups(gi).pitch; prm.roll = js.groups(gi).roll;
                found = true; break;
            end
        end
        if ~found
            if contains(st.nm, '_16')
                prm.yaw = +1.5; prm.pitch = +10.0;
            elseif contains(st.nm, '室外')
                prm.yaw = -1.0; prm.pitch = +3.0;
            else
                prm.yaw = -4.3; prm.pitch = +5.4;
            end
        end
        st.prm = prm;

        fprintf('载入 %s  prm.t=[%.3f %.3f %.3f]  yaw=%.1f pitch=%.1f roll=%.1f\n', ...
            st.nm, prm.tx, prm.ty, prm.tz, prm.yaw, prm.pitch, prm.roll);

        if isvalid(sldF)
            set(sldF, 'Max', st.n, 'Value', st.k, ...
                'SliderStep', [1/max(st.n-1,1), 5/max(st.n-1,1)]);
        end
        renderFrame(st.k);
    end

    function step(d)
        renderFrame(max(1, min(st.n, st.k + d)));
    end

    function onKey(ev)
        if strcmp(ev.Key, 'rightarrow'), step(1); end
        if strcmp(ev.Key, 'leftarrow'), step(-1); end
    end

    function renderFrame(idx)
        idx = max(1, min(st.n, idx));
        st.k = idx;

        S = st.S; prm = st.prm;
        Kthr = get(sldT, 'Value');   % 残差阈值 cm
        findobj(fig, 'Style', 'text', 'String', num2str(Kthr), '-and', ...
            'Position', [0.94 0.05 0.05 0.055]);

        % ---- 雷达点 ----
        Pts = double(S.pts{idx});

        % ---- 深度图 ----
        df = fullfile(S.metaOut.takeDir, 'depth', sprintf('%06d.png', S.depthFrame(idx)));
        D = imread(df);
        if size(D,3) > 1, D = D(:,:,1); end
        D = fliplr(double(D)) * 0.001;
        D(D < 0.3 | D > 9) = NaN;
        set(hImg2, 'CData', D);

        % ---- 投影 ----
        [u, v, ok] = proj(Pts(:,1:3), prm, K);
        ok = ok & u >= 1 & u <= W & v >= 1 & v <= H;

        % ---- 残差：雷达 zo vs 深度图读数 ----
        Rm = Rz(prm.yaw) * Rx(prm.pitch) * Ry(prm.roll);
        Pc = (Pts(:,1:3) - [prm.tx prm.ty prm.tz]) * Rm;
        zo_all = Pc(:, 2);

        resid_cm = nan(size(ok));
        if any(ok)
            dv = interp2(D, u(ok), v(ok), 'linear', NaN);
            resid_cm(ok) = abs(zo_all(ok) - dv) * 100;   % cm
        end

        % ---- 颜色：残差着色 ----
        colors = zeros(numel(resid_cm), 3);
        valid = ok & isfinite(resid_cm) & resid_cm < 100 & zo_all > 0.3;
        thr = Kthr;
        for i = find(valid)'
            r = resid_cm(i);
            if r <= thr
                colors(i,:) = [0 0.85 0];       % 绿
            elseif r <= 2*thr
                t = (r - thr) / thr;           % 黄过渡
                colors(i,:) = [1 t 0];
            else
                colors(i,:) = [1 0 0];         % 红
            end
        end

        % ---- 左：彩色图 + 投影 ----
        cf = fullfile(S.metaOut.takeDir, 'color', sprintf('%06d.png', S.colorFrame(idx)));
        if isfile(cf)
            img = imread(cf);
            set(hImg1, 'CData', img);
        end
        set(hSc1, 'XData', u(valid), 'YData', v(valid), ...
            'CData', resid_cm(valid), 'SizeData', 12 * ones(nnz(valid),1));
        colormap(ax1, [0 0.85 0; 1 1 0; 1 0.5 0; 1 0 0]);
        caxis(ax1, [0 thr*2]);

        % ---- 中：深度图 + 投影 ----
        set(hSc2, 'XData', u(valid), 'YData', v(valid), ...
            'CData', resid_cm(valid), 'SizeData', 12 * ones(nnz(valid),1));

        % ---- 右：3D ----
        Dc = S.depthPts{idx};
        if ~isempty(Dc)
            Rm2 = Rz(prm.yaw) * Rx(prm.pitch) * Ry(prm.roll);
            Dc = double(Dc(:,1:3)) * Rm2' + [prm.tx, prm.ty, prm.tz];
            smp = max(1, floor(size(Dc,1)/6000));
            set(hDep, 'XData', Dc(1:smp:end,1), 'YData', Dc(1:smp:end,2), 'ZData', Dc(1:smp:end,3));
        end
        set(hRadar, 'XData', Pts(valid,1), 'YData', Pts(valid,2), ...
            'ZData', Pts(valid,3), 'CData', resid_cm(valid));
        set(hCam, 'XData', prm.tx, 'YData', prm.ty, 'ZData', prm.tz);

        n_green = nnz(valid & resid_cm <= thr);
        n_yellow = nnz(valid & resid_cm > thr & resid_cm <= 2*thr);
        n_red = nnz(valid & resid_cm > 2*thr);
        md = median(resid_cm(valid & isfinite(resid_cm)));
        mn = mean(resid_cm(valid & isfinite(resid_cm)));

        title(ax1, sprintf('彩色 + 投影  残差 ≤ %.0fcm 绿 / 黄 / 红', thr), 'FontSize', 9);
        title(ax2, sprintf('深度 + 投影  中位 %.1fcm  平均 %.1fcm', md, mn), 'FontSize', 9);
        title(ax3, sprintf('3D  橙=雷达 红=相机 t=[%.3f %.3f %.3f]', prm.tx, prm.ty, prm.tz), 'FontSize', 9);

        info = sprintf(['%s  |  帧 %d/%d  |  t=[%.3f, %.3f, %.3f]  yaw=%.1f pitch=%.1f\n' ...
                        '绿=%d 黄=%d 红=%d  |  中位残差=%.1fcm'], ...
            st.nm, idx, st.n, prm.tx, prm.ty, prm.tz, prm.yaw, prm.pitch, ...
            n_green, n_yellow, n_red, md);
        set(txtInfo, 'String', info);
        drawnow limitrate;
    end

    function [u, v, ok] = proj(P, prm, Kloc)
        R = Rz(prm.yaw) * Rx(prm.pitch) * Ry(prm.roll);
        Pc = (P - [prm.tx, prm.ty, prm.tz]) * R;
        xo = Pc(:,1); yo = -Pc(:,3); zo = Pc(:,2);
        ok = zo > 0.25;
        u = nan(size(zo)); v = nan(size(zo));
        u(ok) = Kloc(1) .* xo(ok) ./ zo(ok) + Kloc(3);
        v(ok) = Kloc(2) .* yo(ok) ./ zo(ok) + Kloc(4);
    end
end

function R = Rz(a), s=sind(a); c=cosd(a); R=[c -s 0; s c 0; 0 0 1]; end
function R = Ry(a), s=sind(a); c=cosd(a); R=[c 0 s; 0 1 0; -s 0 c]; end
function R = Rx(a), s=sind(a); c=cosd(a); R=[1 0 0; 0 c -s; 0 s c]; end

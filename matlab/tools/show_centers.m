function show_centers()
%SHOW_CENTERS 在原图上同时标出"棋盘中心"与"雷达中心"，连线并标注偏差
%  棋盘中心：detectCheckerboardPoints 角点阵列中心（红+）
%  雷达中心：贴板距离层内窄条带的质心投影（青+）
%  参数：t=[0.008 -0.048 0.130]，az 补偿 3.2096°，yaw/pitch/roll 取像素级重解值
%
% 用法：show_centers          在 MATLAB 里直接弹窗
%      show_centers('save')   只导出 PNG

cd('G:\3.2\matlab'); addpath('tools','algorithm','algorithm/lib');
P = paths(); K = [491.4 655.7 319.5 239.5]; W = 640; H = 480;
% 最新 t：读 tall_result.json 的加权平均（11-5 批次室外数据已纳入）
js = jsondecode(fileread(fullfile(P.resultsDir, 'tall_result.json')));
t = js.t_weighted_mean(:).';
AZDEG = 3.2096;
fprintf('采用最新 t（加权平均） = [%+.4f, %+.4f, %+.4f] m\n', t);

% 姿态：yaw 像素级重解（yaw_solve），pitch 俯角补偿后重解（pitch_solve），roll 取 solve_t_all
PRM = { '20261104_132932_室内2米5_1',  1.10,  8.15,  1.00, 2.5;
        '20261104_134227_室内2米_2',   2.55, 10.15, -2.00, 2.0;
        '20261104_134904_室内3米_1',  -0.85, 11.50,  1.75, 3.0;
        '20261104_134957_室内3米_2',   4.80,  7.35, -4.75, 3.0;
        '20261104_162154_室内4米',    -0.70,  6.20,  5.00, 4.0;
        '20261104_162348_室内5米',     1.60,  9.30, -3.00, 5.0 };

outDir = fullfile(P.resultsDir,'centers'); if ~exist(outDir,'dir'), mkdir(outDir); end

fprintf('\n%-24s %14s %14s | %8s %8s | %7s %7s | %s\n', ...
    '组','棋盘中心(u,v)','雷达中心(u,v)','du','dv','dx_cm','dz_cm','点数');
fprintf('%s\n', repmat('-',1,112));

for i = 1:size(PRM,1)
    nm = PRM{i,1}; yaw = PRM{i,2}; pitch = PRM{i,3}; roll = PRM{i,4}; dNom = PRM{i,5};
    S = load(fullfile(P.takesDir, [nm '_res.mat']));
    k = min(numel(S.frames),10);

    cf = fullfile(S.metaOut.takeDir,'color',sprintf('%06d.png', S.colorFrame(k)));
    img = imread(cf);

    % ---- 棋盘中心 ----
    corners = [];
    try, corners = detectCheckerboardPoints(img); catch, end
    if isempty(corners) || size(corners,1) < 4
        fprintf('%-24s 棋盘未检测到，跳过\n', nm); continue;
    end
    bx = mean(corners(:,1)); by = mean(corners(:,2));

    % ---- 雷达中心：贴板层 + 窄条带 + 质心 ----
    Pts = double(S.pts{k});
    Pts(:,1) = Pts(:,1) - Pts(:,2)*tand(AZDEG);
    mL = Pts(:,2)>dNom-0.15 & Pts(:,2)<dNom+0.15 & abs(Pts(:,1))<0.30 & Pts(:,3)>-0.6 & Pts(:,3)<0.6;
    Pl = Pts(mL,1:3);
    if size(Pl,1) < 5, fprintf('%-24s 层点不足(%d)\n', nm, size(Pl,1)); continue; end
    edges = -0.30:0.02:0.30; [cnt,~] = histcounts(Pl(:,1), edges); [~,pk] = max(cnt);
    stripX = (edges(pk)+edges(pk+1))/2;
    Ps = Pl(abs(Pl(:,1)-stripX)<0.03,:);
    if size(Ps,1) < 3, fprintf('%-24s 条带点不足\n', nm); continue; end

    % 条带质心（三维）投影
    cen = mean(Ps,1);
    Rm = Rz(yaw)*Rx(pitch)*Ry(roll);
    Pc = (cen - t) * Rm;
    rx = K(1)*Pc(1)/Pc(2)+K(3);
    ry = K(2)*(-Pc(3))/Pc(2)+K(4);

    du = rx - bx; dv = ry - by;
    dx_cm = du * dNom * 100 / K(1);
    dz_cm = -dv * dNom * 100 / K(2);

    fprintf('%-24s (%6.1f,%6.1f) (%6.1f,%6.1f) | %+8.1f %+8.1f | %+7.1f %+7.1f | %d\n', ...
        nm, bx, by, rx, ry, du, dv, dx_cm, dz_cm, size(Ps,1));

    % ---- 画图 ----
    fig = figure('Visible','on','Color','w','Position',[60 60 1100 820]);
    ax = axes('Parent',fig,'Units','normalized','Position',[0.04 0.10 0.92 0.82]);
    imshow(img, 'Parent', ax); hold(ax,'on');

    % 全部条带点投影（淡青小点，作为背景证据）
    PcP = (Ps - t) * Rm;
    zo = PcP(:,2); okp = zo>0.25;
    uu = K(1)*PcP(okp,1)./zo(okp)+K(3); vv = K(2)*(-PcP(okp,3))./zo(okp)+K(4);
    plot(ax, uu, vv, '.', 'Color', [0.55 0.9 1], 'MarkerSize', 8);

    % 棋盘角点与中心
    plot(ax, corners(:,1), corners(:,2), 'gx', 'MarkerSize', 7, 'LineWidth', 1);
    plot(ax, bx, by, 'r+', 'MarkerSize', 30, 'LineWidth', 3);
    plot(ax, bx, by, 'ro', 'MarkerSize', 26, 'LineWidth', 2);

    % 雷达中心
    plot(ax, rx, ry, 'p', 'MarkerSize', 24, 'MarkerFaceColor', [0 0.85 0.9], ...
        'MarkerEdgeColor','k', 'LineWidth', 1.5);

    % 连线
    plot(ax, [bx rx], [by ry], 'm-', 'LineWidth', 2);

    % 标注数值
    text(ax, (bx+rx)/2, (by+ry)/2 - 12, ...
        sprintf('Δu=%+.1fpx  Δv=%+.1fpx\\rightarrowΔx=%+.1fcm  Δz=%+.1fcm', du, dv, dx_cm, dz_cm), ...
        'Color','m','FontSize',11,'FontWeight','bold','BackgroundColor','w', ...
        'HorizontalAlignment','center');

    legend(ax, {'雷达条带点','棋盘角点','棋盘中心','','雷达中心',''}, ...
        'Location','northeast','Color',[0.12 0.12 0.12],'TextColor',[0.95 0.95 0.95]);

    title(ax, sprintf('%s   帧 %d   yaw=%+.2f° pitch=%+.2f°   t=[%.3f %.3f %.3f]', ...
        nm, k, yaw, pitch, t(1), t(2), t(3)), 'Interpreter','none','FontSize',12);

    exportgraphics(fig, fullfile(outDir, [nm '_centers.png']), 'Resolution', 150);
    if strcmp(computer,'PCWIN64') && ~batchStartupOptionUsed()
        % 交互运行时保留窗口，供连续查看
    else
        close(fig);
    end
end
fprintf('\n输出图: %s\n', outDir);

    function R = Rz(a), s=sind(a); c=cosd(a); R=[c -s 0; s c 0; 0 0 1]; end
    function R = Ry(a), s=sind(a); c=cosd(a); R=[c 0 s; 0 1 0; -s 0 c]; end
    function R = Rx(a), s=sind(a); c=cosd(a); R=[1 0 0; 0 c -s; 0 s c]; end
    function tf = batchStartupOptionUsed()
        tf = ~usejava('desktop');
    end
end

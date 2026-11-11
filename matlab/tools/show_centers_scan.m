function show_centers_scan(prefix)
%SHOW_CENTERS_SCAN 对指定前缀的采集组自动解 yaw/pitch 并标出雷达中心与棋盘中心
%  仅依赖彩色图 + 雷达点，不依赖深度图，因此室外深度退化场景同样适用
%
% 用法：show_centers_scan('20261105')     % 扫 11-5 全部组（室外 3~7m）
%      show_centers_scan('2026110')      % 扫 11-4 + 11-5

if nargin < 1 || isempty(prefix), prefix = '20261105'; end

cd('G:\3.2\matlab'); addpath('tools','algorithm','algorithm/lib');
P = paths(); K = [491.4 655.7 319.5 239.5];
js = jsondecode(fileread(fullfile(P.resultsDir,'tall_result.json')));
t = js.t_weighted_mean(:).';
AZDEG = 3.2096;

outDir = fullfile(P.resultsDir, ['centers_' prefix]);
if ~exist(outDir,'dir'), mkdir(outDir); end

files = dir(fullfile(P.takesDir, [prefix '*_res.mat']));
if isempty(files), fprintf('未找到 %s*_res.mat\n', prefix); return; end

fprintf('\nΔr=3.66 cm/bin  rangeOffset=-0.19 m\nt=[%+.4f %+.4f %+.4f]  az=%+.4f°\n\n', t, AZDEG);
fprintf('%-26s %6s %8s %8s | %8s %8s | %8s %8s | %s\n', ...
    '组','距离','yaw','pitch','du(px)','dv(px)','dx(cm)','dz(cm)','点数');
fprintf('%s\n', repmat('-',1,108));

rows = {};
for f = 1:numel(files)
    nm = strrep(files(f).name,'_res.mat','');
    tok = regexp(nm,'(\d+(?:\.\d+)?)米','tokens');
    if isempty(tok), fprintf('%-26s 距离解析失败\n',nm); continue; end
    dNom = str2double(tok{1}{1});

    S = load(fullfile(P.takesDir,[nm '_res.mat']));
    k = min(numel(S.frames),10);
    cf = fullfile(S.metaOut.takeDir,'color',sprintf('%06d.png',S.colorFrame(k)));
    if ~exist(cf,'file'), fprintf('%-26s 彩色帧缺失\n',nm); continue; end
    img = imread(cf);

    corners = []; ok = false;
    try
        corners = detectCheckerboardPoints(img);
        ok = ~isempty(corners) && size(corners,1) >= 6;
    catch
    end
    if ~ok, fprintf('%-26s 棋盘未检测到（可能太远/太斜）\n',nm); continue; end
    bx = mean(corners(:,1)); by = mean(corners(:,2));

    % ---- 雷达条带 ----
    Pts = double(S.pts{k});
    Pts(:,1) = Pts(:,1) - Pts(:,2)*tand(AZDEG);
    mL = Pts(:,2)>dNom-0.15 & Pts(:,2)<dNom+0.15 & abs(Pts(:,1))<0.35 & Pts(:,3)>-0.7 & Pts(:,3)<0.7;
    Pl = Pts(mL,1:3);
    if size(Pl,1) < 5, fprintf('%-26s 贴板层点不足(%d)\n',nm,size(Pl,1)); continue; end
    edges = -0.35:0.02:0.35; [cnt,~] = histcounts(Pl(:,1),edges); [~,pk] = max(cnt);
    stripX = (edges(pk)+edges(pk+1))/2;
    Ps = Pl(abs(Pl(:,1)-stripX)<0.03,:);
    if size(Ps,1) < 3, fprintf('%-26s 条带点不足\n',nm); continue; end
    cen = mean(Ps,1);

    % ---- 一维搜索 yaw（像素判据） ----
    bestY = struct('v',-1,'cost',inf,'u',NaN);
    for yw = -15:0.05:15
        Rm = Rz(yw)*Rx(0)*Ry(0);
        Pc = (cen - t)*Rm;
        if Pc(2) <= 0.25, continue; end
        uu = K(1)*Pc(1)/Pc(2)+K(3);
        c = abs(uu - bx);
        if c < bestY.cost, bestY.v = yw; bestY.cost = c; bestY.u = uu; end
    end
    yaw = bestY.v;

    % ---- 固定 yaw，搜索 pitch ----
    bestP = struct('v',0,'cost',inf,'vv',NaN);
    for pp = -30:0.05:35
        Rm = Rz(yaw)*Rx(pp)*Ry(0);
        Pc = (cen - t)*Rm;
        if Pc(2) <= 0.25, continue; end
        vv = K(2)*(-Pc(3))/Pc(2)+K(4);
        c = abs(vv - by);
        if c < bestP.cost, bestP.v = pp; bestP.cost = c; bestP.vv = vv; end
    end
    pitch = bestP.v;

    % ---- 最终投影（整条带） ----
    Rm = Rz(yaw)*Rx(pitch)*Ry(0);
    PcA = (Ps - t)*Rm;
    good = PcA(:,2) > 0.25;
    uu = K(1)*PcA(good,1)./PcA(good,2)+K(3);
    vv = K(2)*(-PcA(good,3))./PcA(good,2)+K(4);
    du = bestY.u - bx;  dv = bestP.vv - by;   % 用解算后的最终值重算
    Rm2 = Rz(yaw)*Rx(pitch)*Ry(0); Pc2 = (cen - t)*Rm2;
    rx = K(1)*Pc2(1)/Pc2(2)+K(3); ry = K(2)*(-Pc2(3))/Pc2(2)+K(4);
    du = rx - bx; dv = ry - by;
    dx_cm = du * dNom * 100 / K(1);
    dz_cm = -dv * dNom * 100 / K(2);

    fprintf('%-26s %5.1fm %+8.2f %+8.2f | %+8.1f %+8.1f | %+8.1f %+8.1f | %5d\n', ...
        nm, dNom, yaw, pitch, du, dv, dx_cm, dz_cm, size(Ps,1));

    rows{end+1} = struct('name',nm,'dist_m',dNom,'yaw',yaw,'pitch',pitch, ...
        'du_px',du,'dv_px',dv,'dx_cm',dx_cm,'dz_cm',dz_cm,'npts',size(Ps,1));

    % ---- 出图 ----
    fig = figure('Visible','on','Color','w','Position',[50 50 1100 820]);
    ax = axes('Parent',fig,'Units','normalized','Position',[0.04 0.10 0.92 0.82]);
    imshow(img,'Parent',ax); hold(ax,'on');
    plot(ax, uu, vv, '.', 'Color',[0.55 0.9 1], 'MarkerSize',8);
    plot(ax, corners(:,1), corners(:,2), 'gx','MarkerSize',7);
    plot(ax, bx, by, 'r+','MarkerSize',30,'LineWidth',3);
    plot(ax, bx, by, 'ro','MarkerSize',26,'LineWidth',2);
    plot(ax, rx, ry, 'p','MarkerSize',24,'MarkerFaceColor',[0 0.85 0.9],'MarkerEdgeColor','k','LineWidth',1.5);
    plot(ax, [bx rx],[by ry],'m-','LineWidth',2);
    text(ax,(bx+rx)/2,(by+ry)/2-14, ...
        sprintf('Δu=%+.1fpx Δv=%+.1fpx → Δx=%+.1fcm Δz=%+.1fcm',du,dv,dx_cm,dz_cm), ...
        'Color','m','FontSize',11,'FontWeight','bold','BackgroundColor','w','HorizontalAlignment','center');
    title(ax, sprintf('%s  %d帧  距离%.1fm  yaw=%+.2f° pitch=%+.2f°', nm, k, dNom, yaw, pitch), ...
        'Interpreter','none','FontSize',12);
    exportgraphics(fig, fullfile(outDir,[nm '_centers.png']),'Resolution',150);
    if ~usejava('desktop'), close(fig); end
end

if ~isempty(rows)
    T = struct2table(vertcat(rows{:}));
    writetable(T, fullfile(outDir,'centers_scan.csv'));
    fprintf('\n水平 |Δu| 中位 %.1f px，最大 %.1f px\n', median(abs(T.du_px)), max(abs(T.du_px)));
    fprintf('垂直 |Δv| 中位 %.1f px，最大 %.1f px\n', median(abs(T.dv_px)), max(abs(T.dv_px)));
    fprintf('yaw   %+.2f ~ %+.2f°，中位 %+.2f°\n', min(T.yaw), max(T.yaw), median(T.yaw));
    fprintf('pitch %+.2f ~ %+.2f°，中位 %+.2f°\n', min(T.pitch), max(T.pitch), median(T.pitch));
end
fprintf('输出: %s\n', outDir);

    function R = Rz(a), s=sind(a); c=cosd(a); R=[c -s 0; s c 0; 0 0 1]; end
    function R = Ry(a), s=sind(a); c=cosd(a); R=[c 0 s; 0 1 0; -s 0 c]; end
    function R = Rx(a), s=sind(a); c=cosd(a); R=[1 0 0; 0 c -s; 0 s c]; end
end

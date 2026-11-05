function pitch_solve()
%PITCH_SOLVE 俯角补偿：固定 t / yaw / roll，一维搜索 pitch 使雷达中心 v 对上棋盘中心 v
%  t      = tall_result.json 的加权平均（Δr=3.66 + rangeOffset −0.19 已在 pts 内）
%  yaw    = pixel 判据重解值（yaw_solve.csv）
%  roll   = solve_t_all 值
%  搜索目标：|雷达条带质心投影 v − 棋盘中心 v| 最小

cd('G:\3.2\matlab'); addpath('tools','algorithm','algorithm/lib');
P = paths(); K = [491.4 655.7 319.5 239.5];
js = jsondecode(fileread(fullfile(P.resultsDir,'tall_result.json')));
t = js.t_weighted_mean(:).';
AZDEG = 3.2096;

% 组名, yaw(像素级), 旧pitch, roll, 标称距离
PRM = { '20261104_132932_室内2米5_1',  1.10,  3.90,  1.00, 2.5;
        '20261104_134227_室内2米_2',   2.55,  4.40, -2.00, 2.0;
        '20261104_134904_室内3米_1',  -0.85,  3.40,  1.75, 3.0;
        '20261104_134957_室内3米_2',   4.80,  4.40, -4.75, 3.0;
        '20261104_162154_室内4米',    -0.70, 12.50,  5.00, 4.0;
        '20261104_162348_室内5米',     1.60, 14.75, -3.00, 5.0 };

outDir = fullfile(P.resultsDir,'pitch_solve'); if ~exist(outDir,'dir'), mkdir(outDir); end

fprintf('\nΔr = 3.66 cm/bin, rangeOffset = -0.19 m（已在 pts 内）\n');
fprintf('t = [%+.4f, %+.4f, %+.4f]，az 补偿 %+.4f°\n\n', t, AZDEG);
fprintf('%-24s %8s %8s %8s | %8s %8s | %10s %8s\n', ...
    '组','旧pitch','新pitch','差','旧dv','新dv','板距离','标称');
fprintf('%s\n', repmat('-',1,104));

res = {};
for i = 1:size(PRM,1)
    nm = PRM{i,1}; yaw = PRM{i,2}; pitch0 = PRM{i,3}; roll = PRM{i,4}; dNom = PRM{i,5};
    S = load(fullfile(P.takesDir,[nm '_res.mat'])); k = min(numel(S.frames),10);

    cf = fullfile(S.metaOut.takeDir,'color',sprintf('%06d.png',S.colorFrame(k)));
    img = imread(cf);
    corners = [];
    try, corners = detectCheckerboardPoints(img); catch, end
    if isempty(corners) || size(corners,1) < 4, fprintf('%-24s 棋盘未检测\n',nm); continue; end
    bx = mean(corners(:,1)); by = mean(corners(:,2));

    Pts = double(S.pts{k});
    Pts(:,1) = Pts(:,1) - Pts(:,2)*tand(AZDEG);
    mL = Pts(:,2)>dNom-0.15 & Pts(:,2)<dNom+0.15 & abs(Pts(:,1))<0.30 & Pts(:,3)>-0.6 & Pts(:,3)<0.6;
    Pl = Pts(mL,1:3);
    if size(Pl,1) < 5, fprintf('%-24s 层点不足\n',nm); continue; end
    edges = -0.30:0.02:0.30; [cnt,~] = histcounts(Pl(:,1),edges); [~,pk] = max(cnt);
    stripX = (edges(pk)+edges(pk+1))/2;
    Ps = Pl(abs(Pl(:,1)-stripX)<0.03,:);
    if size(Ps,1) < 3, fprintf('%-24s 条带点不足\n',nm); continue; end
    cen = mean(Ps,1);

    % ---- 一维搜索 pitch ----
    best = struct('p',pitch0,'adv',inf,'v',NaN);
    for pi_ = -25 : 0.05 : 30
        Rm = Rz(yaw)*Rx(pi_)*Ry(roll);
        Pc = (cen - t) * Rm;
        if Pc(2) <= 0.25, continue; end
        vv = K(2)*(-Pc(3))/Pc(2)+K(4);
        adv = abs(vv - by);
        if adv < best.adv, best.p = pi_; best.adv = adv; best.v = vv; end
    end

    % 旧 pitch 的 dv
    Rm0 = Rz(yaw)*Rx(pitch0)*Ry(roll);
    Pc0 = (cen - t) * Rm0;
    v0 = K(2)*(-Pc0(3))/Pc0(2)+K(4);

    dvOld = v0 - by; dvNew = best.v - by;
    dzNew = -dvNew * dNom * 100 / K(2);

    fprintf('%-24s %+8.2f %+8.2f %+8.2f | %+8.1f %+8.1f | %9.3f %8.1f\n', ...
        nm, pitch0, best.p, best.p-pitch0, dvOld, dvNew, median(Ps(:,2)), dNom);

    res{end+1} = struct('name',nm,'pitch_old',pitch0,'pitch_new',best.p,'dvOld',dvOld,'dvNew',dvNew,'dz_cm',dzNew,'dNom',dNom);

    % ---- 出图对照（旧 pitch 左 / 新 pitch 右） ----
    fig = figure('Visible','on','Color','w','Position',[40 40 1400 780]);
    for side = 1:2
        ax = axes('Parent',fig,'Units','normalized','Position',[0.02+(side-1)*0.49 0.06 0.47 0.86]);
        imshow(img,'Parent',ax); hold(ax,'on');
        if side==1, pp = pitch0; else, pp = best.p; end
        Rm = Rz(yaw)*Rx(pp)*Ry(roll);
        PcP = (Ps - t)*Rm; ok = PcP(:,2)>0.25;
        uu = K(1)*PcP(ok,1)./PcP(ok,2)+K(3);
        vv = K(2)*(-PcP(ok,3))./PcP(ok,2)+K(4);
        plot(ax, uu, vv, '.', 'Color',[0.55 0.9 1], 'MarkerSize',7);
        PcC = (cen - t)*Rm;
        rx = K(1)*PcC(1)/PcC(2)+K(3); ry = K(2)*(-PcC(3))/PcC(2)+K(4);
        plot(ax, corners(:,1), corners(:,2), 'gx','MarkerSize',6);
        plot(ax, bx, by, 'r+','MarkerSize',26,'LineWidth',2.5);
        plot(ax, rx, ry, 'p','MarkerSize',20,'MarkerFaceColor',[0 0.85 0.9],'MarkerEdgeColor','k');
        plot(ax, [bx rx],[by ry],'m-','LineWidth',1.5);
        title(ax, sprintf('%s  pitch %+.2f°  dv=%+.1fpx  Δz=%+.1fcm', nm, pp, ry-by, -(ry-by)*dNom*100/K(2)), ...
            'Interpreter','none','FontSize',10);
    end
    exportgraphics(fig, fullfile(outDir,[nm '_pitchcmp.png']),'Resolution',120);
    if ~usejava('desktop'), close(fig); end
end

if ~isempty(res)
    T = struct2table(vertcat(res{:}));
    writetable(T, fullfile(outDir,'pitch_solve.csv'));
    fprintf('\n新 pitch: 中位 %.2f°  均值 %.2f°  σ %.2f°\n', median(T.pitch_new), mean(T.pitch_new), std(T.pitch_new));
    fprintf('剩余 dv:  中位 %.1f px  |  最大 %.1f px\n', median(abs(T.dvNew)), max(abs(T.dvNew)));
end
fprintf('输出: %s\n', outDir);

    function R = Rz(a), s=sind(a); c=cosd(a); R=[c -s 0; s c 0; 0 0 1]; end
    function R = Ry(a), s=sind(a); c=cosd(a); R=[c 0 s; 0 1 0; -s 0 c]; end
    function R = Rx(a), s=sind(a); c=cosd(a); R=[1 0 0; 0 c -s; 0 s c]; end
end

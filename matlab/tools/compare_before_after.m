function compare_before_after()
%COMPARE_BEFORE_AFTER 室内 / 室外 分开生成校正前后对比图 + 自动弹出
%
% 对每组数据都用彩色图+雷达点云独立跑像素判据（不读深度）：
%   先搜 yaw = argmin |median(u_strip) − boardU|
%   再搜 pitch = argmin |v_cen − boardV|
% 固定 tFix + az补偿
%
% 运行：MATLAB GUI 里敲
%   cd('G:\3.2\matlab'); addpath('tools','algorithm','algorithm/lib'); compare_before_after

close all; clc;
cd('G:\3.2\matlab'); addpath('tools','algorithm','algorithm/lib');
P = paths();
K = [491.4 655.7 319.5 239.5];
AZDEG = 3.2096;

js = jsondecode(fileread(fullfile(P.resultsDir,'tall_result.json')));
tFix = js.t_weighted_mean(:).';

indoor = { ...
    '20261104_132932_室内2米5_1', 2.5; ...
    '20261104_134227_室内2米_2',   2.0; ...
    '20261104_134904_室内3米_1',   3.0; ...
    '20261104_134957_室内3米_2',   3.0; ...
    '20261104_162154_室内4米',     4.0; ...
    '20261104_162348_室内5米',     5.0 };

outdoor = { ...
    '20261105_152723_室外3米',     3.0; ...
    '20261105_153539_室外4米',     4.0 };

outDir = fullfile(P.resultsDir, 'before_after');
if ~exist(outDir, 'dir'), mkdir(outDir); end

% ---- 只跑室外 ----
[newParams, oldParams] = solveAllPixel(outdoor, js, tFix, AZDEG, K, P);

fig = buildCompareFigure(outdoor, outdoor, newParams, oldParams, K, AZDEG, tFix, ...
    '室外组 - 校正前（左）vs 校正后（右）', P);
p = fullfile(outDir, 'outdoor_compare.png');
exportgraphics(fig, p, 'Resolution', 180);
close(fig);
fprintf('室外已保存: %s\n', p);
winopen(p);
end  % --- 主函数 ---


% =========================================================================
%  对所有组跑像素判据：先 yaw 后 pitch
%  输入: groups cell array {name, dNom}
%  输出: newParams(nm) = [yawNew, pitchNew]
%        oldParams(nm) = [tOld(1:3), yawOld, pitchOld, rollOld]
% =========================================================================
function [newParams, oldParams] = solveAllPixel(groups, js, tFix, AZDEG, K, P)
newParams = containers.Map('KeyType','char','ValueType','any');
oldParams = containers.Map('KeyType','char','ValueType','any');

fprintf('\n像素判据（先 yaw 后 pitch，固定 tFix + az %.4f°）\n', AZDEG);
fprintf('%-28s %8s %8s %8s | %8s %8s | %8s %8s\n', ...
    '组名','旧yaw','新yaw','dyaw','旧pch','新pch','dpch','板检测');
fprintf('%s\n', repmat('-',1,100));

for i = 1:size(groups,1)
    nm   = groups{i,1};
    dNom = groups{i,2};

    gi = find(strcmp({js.groups.name}, nm), 1);
    tOld    = js.groups(gi).t(:).';
    yawOld  = js.groups(gi).yaw;
    pitchOld= js.groups(gi).pitch;
    rollOld = js.groups(gi).roll;
    oldParams(nm) = [tOld, yawOld, pitchOld, rollOld];

    % ---- 加载 ----
    S  = load(fullfile(P.takesDir, [nm '_res.mat']));
    k  = min(numel(S.frames), 10);
    cf = fullfile(S.metaOut.takeDir, 'color', sprintf('%06d.png', S.colorFrame(k)));
    img = imread(cf);

    boardU = NaN; boardV = NaN;
    try
        corners = detectCheckerboardPoints(img);
        if ~isempty(corners) && size(corners,1) >= 4
            boardU = mean(corners(:,1));
            boardV = mean(corners(:,2));
        end
    catch
    end
    hasBoard = ~isnan(boardU);

    Pts = double(S.pts{k});
    PtsC = Pts; PtsC(:,1) = PtsC(:,1) - PtsC(:,2)*tand(AZDEG);   % az 补偿

    % ---- 雷达条带 ----
    mL = PtsC(:,2)>dNom-0.15 & PtsC(:,2)<dNom+0.15 & abs(PtsC(:,1))<0.30 & ...
         PtsC(:,3)>-0.6 & PtsC(:,3)<0.6;
    Pl = PtsC(mL,1:3);
    if size(Pl,1) < 5
        fprintf('%-28s 层点不足 (%d)\n', nm, size(Pl,1));
        newParams(nm) = [yawOld, pitchOld];
        continue;
    end
    edges = -0.30:0.02:0.30; [cnt,~] = histcounts(Pl(:,1),edges); [~,pk] = max(cnt);
    stripX = (edges(pk)+edges(pk+1))/2;
    Ps = Pl(abs(Pl(:,1)-stripX)<0.03,:);
    if size(Ps,1) < 3
        fprintf('%-28s 条带点不足 (%d)\n', nm, size(Ps,1));
        newParams(nm) = [yawOld, pitchOld];
        continue;
    end
    cen = mean(Ps,1);

    if ~hasBoard
        fprintf('%-28s 板检测失败\n', nm);
        newParams(nm) = [yawOld, pitchOld];
        continue;
    end

    % ---- 1) 搜 yaw（固定 tFix + rollOld，pitch 先用 pitchOld）----
    bestY = struct('yaw',yawOld,'adu',inf,'u',NaN);
    for yw = -12 : 0.05 : 12
        Rm = Rz(yw)*Rx(pitchOld)*Ry(rollOld);
        Pc = (Ps - tFix) * Rm;
        zo = Pc(:,2); okp = zo>0.25;
        if nnz(okp) < 3, continue; end
        u = K(1).*Pc(okp,1)./zo(okp)+K(3);
        inb = u>=1 & u<=640;
        if nnz(inb) < 3, continue; end
        adu = abs(median(u(inb)) - boardU);
        if adu < bestY.adu, bestY.yaw=yw; bestY.adu=adu; bestY.u=median(u(inb)); end
    end
    yawNew = bestY.yaw;

    % ---- 2) 搜 pitch（固定 tFix + rollOld + yawNew）----
    bestP = struct('p',pitchOld,'adv',inf,'v',NaN);
    for pi_ = -25 : 0.05 : 30
        Rm = Rz(yawNew)*Rx(pi_)*Ry(rollOld);
        Pc = (cen - tFix) * Rm;
        if Pc(2) <= 0.25, continue; end
        vv = K(2)*(-Pc(3))/Pc(2)+K(4);
        adv = abs(vv - boardV);
        if adv < bestP.adv, bestP.p=pi_; bestP.adv=adv; bestP.v=vv; end
    end
    pitchNew = bestP.p;

    newParams(nm) = [yawNew, pitchNew];
    fprintf('%-28s %+8.2f %+8.2f %+8.2f | %+8.2f %+8.2f %+8.2f | OK\n', ...
        nm, yawOld, yawNew, yawNew-yawOld, pitchOld, pitchNew, pitchNew-pitchOld);
end
end  % --- solveAllPixel ---


% =========================================================================
%  画一张子 figure
% =========================================================================
function fig = buildCompareFigure(groups, allGroups, newParams, oldParams, K, AZDEG, tFix, mainTitle, P)
N = size(groups,1);

subW = 640; subH = 480;
gapT = 50; gapB = 12; gapL = 12; gapR = 12; gapRow = 6; gapCol = 6;
figW = gapL + 2*subW + gapCol + gapR;
figH = gapT + N*subH + (N-1)*gapRow + gapB;

fig = figure('Visible','off','Color','w', ...
    'Position',[60 40 figW figH], 'Name', mainTitle, ...
    'PaperUnits','normalized','PaperPosition',[0 0 1 1]);

annotation(fig,'textbox',[0.01 0.982 0.98 0.022], ...
    'String', sprintf('%s   az=+%.4f deg   t=[%+.4f %+.4f %+.4f]', mainTitle, AZDEG, tFix), ...
    'FontSize',16,'FontWeight','bold','HorizontalAlignment','center', ...
    'EdgeColor','none','BackgroundColor','none');

for i = 1:N
    nm   = groups{i,1};
    dNom = groups{i,2};

    % ---- 旧参数 ----
    op = oldParams(nm);
    tOld    = op(1:3);
    yawOld  = op(4);
    pitchOld= op(5);
    rollOld = op(6);

    % ---- 新参数（像素判据）----
    np = newParams(nm);
    yawNew   = np(1);
    pitchNew = np(2);

    S  = load(fullfile(P.takesDir, [nm '_res.mat']));
    k  = min(numel(S.frames), 10);
    cf = fullfile(S.metaOut.takeDir, 'color', sprintf('%06d.png', S.colorFrame(k)));
    img = imread(cf);

    corners = detectCheckerboardPoints(img);
    if isempty(corners) || size(corners,1) < 6
        bx = NaN; by = NaN; corners = [];
    else
        bx = mean(corners(:,1));
        by = mean(corners(:,2));
    end

    Pts  = double(S.pts{k});
    Praw = Pts;
    Pcor = Pts; Pcor(:,1) = Pcor(:,1) - Pcor(:,2)*tand(AZDEG);

    [cenA, ~] = stripCenter(Praw, dNom);
    [cenB, ~] = stripCenter(Pcor, dNom);

    % 统一用 tFix（旧 t 解对室外有问题），只换姿态参数做对比
    % before: tFix + az补偿cen + 旧 yaw/pitch/roll（体现"未姿态校正"的误差）
    rcA = projectCenter(cenB, tFix, yawOld, pitchOld, rollOld, K);
    % after:  tFix + az补偿cen + 新 yaw/pitch/roll（像素判据重解后）
    rcB = projectCenter(cenB, tFix, yawNew, pitchNew, rollOld, K);

    if isnan(bx) || isnan(rcA(1)) || isnan(rcB(1))
        pxResA = NaN; pxResB = NaN; mResA = NaN; mResB = NaN;
    else
        duA = rcA(1)-bx; dvA = rcA(2)-by;
        duB = rcB(1)-bx; dvB = rcB(2)-by;
        pxResA = sqrt(duA^2+dvA^2); mResA = pxResA * dNom / K(1);
        pxResB = sqrt(duB^2+dvB^2); mResB = pxResB * dNom / K(1);
    end

    yAnchor = 1 - (gapT + i*subH + (i-1)*gapRow)/figH;

    posA = [gapL/figW, yAnchor, subW/figW, subH/figH];
    axA = axes('Parent', fig, 'Position', posA);
    drawPanel(axA, img, corners, bx, by, rcA, ...
        sprintf('%.1fm before | %.1fpx | %.2fdeg | %.3fm', dNom, pxResA, yawOld, mResA), 'before');

    posB = [(gapL + subW + gapCol)/figW, yAnchor, subW/figW, subH/figH];
    axB = axes('Parent', fig, 'Position', posB);
    drawPanel(axB, img, corners, bx, by, rcB, ...
        sprintf('%.1fm after | %.1fpx | %.2fdeg | %.3fm', dNom, pxResB, yawNew, mResB), 'after');

    fprintf('  %-26s before %5.1fpx | after %4.1fpx\n', nm, pxResA, pxResB);
end
end  % --- buildCompareFigure ---


% ================= 子函数 =================

function [cen, Pstrip] = stripCenter(Pts, dNom)
    mL = Pts(:,2)>dNom-0.15 & Pts(:,2)<dNom+0.15 & abs(Pts(:,1))<0.35 & Pts(:,3)>-0.7 & Pts(:,3)<0.7;
    Pl = Pts(mL,1:3);
    if size(Pl,1) < 5
        cen = mean(Pl,1); Pstrip = Pl; return;
    end
    edges = -0.35:0.02:0.35;
    [cnt,~] = histcounts(Pl(:,1),edges); [~,pk] = max(cnt);
    stripX = (edges(pk)+edges(pk+1))/2;
    Pstrip = Pl(abs(Pl(:,1)-stripX)<0.03,:);
    if isempty(Pstrip), Pstrip = Pl; end
    cen = mean(Pstrip,1);
end

function rc = projectCenter(cen, t, yaw, pitch, roll, K)
    Rm = Rz(yaw)*Rx(pitch)*Ry(roll);
    Pcc = (cen - t)*Rm;
    % 不过滤 Pc(2)：哪怕深度错了也投影出来，让 before 的"离谱位置"可见
    rc = [K(1)*Pcc(1)/Pcc(2)+K(3),  K(2)*(-Pcc(3))/Pcc(2)+K(4)];
end

function drawPanel(ax, img, corners, bx, by, rc, ttl, side)
    axes(ax); cla;
    imshow(img, 'Parent', ax);
    hold(ax, 'on');
    ax.Clipping = 'off';   % 关键：允许画面外的点也显示出来

    if ~isempty(corners)
        plot(ax, corners(:,1), corners(:,2), 'g+', 'MarkerSize', 12, 'LineWidth', 1.5);
        n = size(corners,1); sideLen = round(sqrt(n));
        nRows = sideLen; nCols = ceil(n/sideLen);
        for r = 1:nRows
            idx = (r-1)*nCols + (1:min(nCols, n-(r-1)*nCols));
            plot(ax, corners(idx,1), corners(idx,2), 'g-', 'LineWidth', 0.8);
        end
        for c = 1:nCols
            idx = c:nCols:n;
            plot(ax, corners(idx,1), corners(idx,2), 'g-', 'LineWidth', 0.8);
        end
    end

    if ~isnan(bx)
        plot(ax, bx, by, 'r+', 'MarkerSize', 28, 'LineWidth', 3);
        plot(ax, bx, by, 'ro', 'MarkerSize', 22, 'LineWidth', 1.5);
    end

    if ~isnan(rc(1))
        if strcmpi(side, 'before')
            col = [0.95 0.45 0.15];
        else
            col = [0.10 0.75 0.85];
        end
        plot(ax, rc(1), rc(2), 'p', 'MarkerSize', 22, ...
            'MarkerFaceColor', col, 'MarkerEdgeColor', 'k', 'LineWidth', 1.8);
        if ~isnan(bx)
            plot(ax, [bx rc(1)], [by rc(2)], 'm-', 'LineWidth', 2.5);
        end
    end

    title(ax, ttl, 'FontSize', 13, 'FontWeight', 'bold', 'Interpreter', 'none');
    hold(ax, 'off');
end

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

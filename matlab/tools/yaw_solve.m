function yaw_solve()
%YAW_SOLVE 用像素级横向偏差（条带u vs 板心u）重解各组 yaw
%  固定 t=[0.008 -0.048 0.130] 与 pitch/roll（solve_t_all 解），一维搜索 yaw
%  az 系统偏差 +3° 先补偿：X_corr = X - Y*tand(3)  （候选，按仓库 -3.2096° 口径）

cd('G:\3.2\matlab'); addpath('tools','algorithm','algorithm/lib');
P = paths(); K = [491.4 655.7 319.5 239.5];
js = jsondecode(fileread(fullfile(P.resultsDir,'tall_result.json')));
t = [0.008 -0.048 0.130];
AZDEG = 3.2096;   % 仓库实验分支补偿值；0 = 不补偿

outDir = fullfile(P.resultsDir,'yaw_solve'); if ~exist(outDir,'dir'), mkdir(outDir); end
res = {};

fprintf('\nAZ 补偿: %.4f°\n', AZDEG);
fprintf('%-24s %8s %8s %8s | %8s %8s\n', '组', '旧yaw', '新yaw', 'yaw差', 'du新(px)', 'dx新(cm)');
fprintf('%s\n', repmat('-',1,80));

files = dir(fullfile(P.takesDir, '*_res.mat'));
for i = 1:numel(files)
    nm = erase(files(i).name, '_res.mat');
    if contains(nm,'室外') || contains(nm,'无板'), continue; end
    tk = regexp(nm, '室内(\d+)米', 'tokens');
    if isempty(tk), continue; end
    dNom = str2double(tk{1}{1});
    if contains(nm,'2米5'), dNom = 2.5; end

    S = load(fullfile(P.takesDir, files(i).name));
    k = min(numel(S.frames),10);
    yaw0=0; pitch=0; roll=0;
    for gi=1:numel(js.groups)
        if strcmp(js.groups(gi).name,nm)
            yaw0=js.groups(gi).yaw; pitch=js.groups(gi).pitch; roll=js.groups(gi).roll; break;
        end
    end

    cf = fullfile(S.metaOut.takeDir,'color',sprintf('%06d.png',S.colorFrame(k)));
    img = imread(cf);
    boardU = NaN;
    try
        corners = detectCheckerboardPoints(img);
        if ~isempty(corners) && size(corners,1) >= 4, boardU = mean(corners(:,1)); end
    catch
    end
    if isnan(boardU), fprintf('%-24s 板检测失败\n', nm); continue; end

    Pts = double(S.pts{k});
    Pts(:,1) = Pts(:,1) - Pts(:,2)*tand(AZDEG);   % az 补偿
    mL = Pts(:,2)>dNom-0.15 & Pts(:,2)<dNom+0.15 & abs(Pts(:,1))<0.30 & Pts(:,3)>-0.6 & Pts(:,3)<0.6;
    Pl = Pts(mL,1:3);
    if size(Pl,1) < 5, fprintf('%-24s 层点不足\n', nm); continue; end
    edges = -0.30:0.02:0.30; [cnt,~] = histcounts(Pl(:,1), edges); [~,pk] = max(cnt);
    stripX = (edges(pk)+edges(pk+1))/2;
    Ps = Pl(abs(Pl(:,1)-stripX)<0.03,:);
    if size(Ps,1) < 3, continue; end

    % --- 一维搜索 yaw ∈ [-12, +12]，步长 0.05°（固定区间，避免贴边界） ---
    best = struct('yaw', yaw0, 'adu', inf);
    for yw = -12 : 0.05 : 12
        Rm = Rz(yw)*Rx(pitch)*Ry(roll);
        Pc = (Ps - t) * Rm;
        zo = Pc(:,2); okp = zo>0.25;
        u = K(1).*Pc(okp,1)./zo(okp)+K(3); v = K(2).*(-Pc(okp,3))./zo(okp)+K(4);
        inb = u>=1 & u<=640 & v>=1 & v<=480;
        if nnz(inb) < 3, continue; end
        adu = abs(median(u(inb)) - boardU);
        if adu < best.adu, best.yaw = yw; best.adu = adu; best.u = median(u(inb)); end
    end
    dx_cm = (best.u - boardU) * dNom * 100 / K(1) * sign(best.u - boardU) ;
    dx_cm = (best.u - boardU) * dNom * 100 / K(1);
    fprintf('%-24s %+8.2f %+8.2f %+8.2f | %8.1f %+8.1f\n', nm, yaw0, best.yaw, best.yaw-yaw0, best.u-boardU, dx_cm);
    res{end+1} = struct('name',nm,'yaw_old',yaw0,'yaw_new',best.yaw,'du',best.u-boardU,'dx_cm',dx_cm,'pitch',pitch,'roll',roll);

    % --- 标注图 ---
    Rm = Rz(best.yaw)*Rx(pitch)*Ry(roll);
    Pc = (Ps - t) * Rm; zo = Pc(:,2); okp = zo>0.25;
    u = K(1).*Pc(okp,1)./zo(okp)+K(3); v = K(2).*(-Pc(okp,3))./zo(okp)+K(4);
    fg = figure('Visible','off','Color','w','Position',[100 100 960 720]);
    imshow(img); hold on;
    plot(u, v, '.', 'Color', [0 0.75 0.9], 'MarkerSize', 10);
    xline(boardU, 'r--', 'LineWidth', 1.5); xline(best.u, 'm-', 'LineWidth', 2);
    title(sprintf('%s  yaw %+.2f→%+.2f°  du=%+.1fpx → dx=%+.1fcm', nm, yaw0, best.yaw, best.u-boardU, dx_cm), 'Interpreter','none');
    exportgraphics(fg, fullfile(outDir, [nm '_yaw.png']), 'Resolution', 150); close(fg);
end
writetable(struct2table(vertcat(res{:})), fullfile(outDir,'yaw_solve.csv'));
fprintf('\n输出: %s\n', outDir);

    function R = Rz(a), s=sind(a); c=cosd(a); R=[c -s 0; s c 0; 0 0 1]; end
    function R = Ry(a), s=sind(a); c=cosd(a); R=[c 0 s; 0 1 0; -s 0 c]; end
    function R = Rx(a), s=sind(a); c=cosd(a); R=[1 0 0; 0 c -s; 0 s c]; end
end

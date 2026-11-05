function pixel_verify2()
%PIXEL_VERIFY2 修正版像素级验证：只用"贴板距离层 + 横向窄条带"判定水平对齐
%  修正点：
%   1) Y 窗收窄到 标称距离±0.15m，排除板后百叶窗/桌沿回波
%   2) 指标改为"竖条带横向位置 vs 板心横向像素"，不再用全窗质心
%   3) Z 向延伸（条带物理形态）不参与水平判定，另报"穿板率"作参考

cd('G:\3.2\matlab'); addpath('tools','algorithm','algorithm/lib');
P = paths(); K = [491.4 655.7 319.5 239.5];
js = jsondecode(fileread(fullfile(P.resultsDir,'tall_result.json')));
t = [0.008 -0.048 0.130];

outDir = fullfile(P.resultsDir,'pixel_verify2'); if ~exist(outDir,'dir'), mkdir(outDir); end
fid = fopen(fullfile(outDir,'pixel_verify2.csv'),'w');
fprintf(fid,'group,dNom,nLayer,stripX,nStrip,stripU,boardU,du_px,dx_cm,vOverlap,oldDu\n');

fprintf('\n%-24s %4s %6s %6s %6s | %7s %7s | %7s %7s | %6s | %s\n', ...
    '组','层点','条带X','条带点','条带u','板心u','du_px','dx_cm','穿板','旧质心du');
fprintf('%s\n', repmat('-',1,118));

files = dir(fullfile(P.takesDir, '*_res.mat'));
for i = 1:numel(files)
    nm = erase(files(i).name, '_res.mat');
    if contains(nm,'室外') || contains(nm,'无板'), continue; end
    tk = regexp(nm, '室内(\d+)米(_(\d))?', 'tokens');
    if isempty(tk), continue; end
    dNom = str2double(tk{1}{1});
    if contains(nm,'2米5'), dNom = 2.5; end

    S = load(fullfile(P.takesDir, files(i).name));
    k = min(numel(S.frames),10);

    yaw=0; pitch=0; roll=0;
    for gi=1:numel(js.groups)
        if strcmp(js.groups(gi).name,nm)
            yaw=js.groups(gi).yaw; pitch=js.groups(gi).pitch; roll=js.groups(gi).roll; break;
        end
    end

    cf = fullfile(S.metaOut.takeDir,'color',sprintf('%06d.png',S.colorFrame(k)));
    img = imread(cf);
    [boardU,boardV,boardUr,boardVr,corners] = deal(NaN,NaN,[],[],[]);
    try
        corners = detectCheckerboardPoints(img);
        if ~isempty(corners) && size(corners,1) >= 4
            boardU = mean(corners(:,1)); boardV = mean(corners(:,2));
            boardUr = [min(corners(:,1)) max(corners(:,1))];
            boardVr = [min(corners(:,2)) max(corners(:,2))];
        end
    catch
    end

    Pts = double(S.pts{k});
    % --- 贴板距离层 ---
    mL = Pts(:,2)>dNom-0.15 & Pts(:,2)<dNom+0.15 & abs(Pts(:,1))<0.30 & Pts(:,3)>-0.6 & Pts(:,3)<0.6;
    Pl = Pts(mL,1:3);
    if size(Pl,1) < 5, fprintf('%-24s 层点不足\n', nm); continue; end

    % --- X 直方图找窄条带（2cm 分箱） ---
    edges = -0.30:0.02:0.30;
    [cnt,~,bin] = histcounts(Pl(:,1), edges);
    [~,pk] = max(cnt);
    stripX = (edges(pk)+edges(pk+1))/2;
    mS = abs(Pl(:,1) - stripX) < 0.03;
    Ps = Pl(mS,:);
    if size(Ps,1) < 3, fprintf('%-24s 条带点不足\n', nm); continue; end

    % --- 投影 ---
    Rm = Rz(yaw)*Rx(pitch)*Ry(roll);
    Pc = (Ps - t) * Rm;
    zo = Pc(:,2); okp = zo>0.25;
    u = K(1).*Pc(okp,1)./zo(okp)+K(3); v = K(2).*(-Pc(okp,3))./zo(okp)+K(4);
    inb = u>=1 & u<=640 & v>=1 & v<=480;
    u = u(inb); v = v(inb);
    if isempty(u), fprintf('%-24s 投影出图\n', nm); continue; end
    stripU = median(u);

    du = stripU - boardU;
    dx_cm = du * dNom * 100 / K(1);

    % 穿板率：条带点 v 落在板 v 范围内的比例
    vOv = NaN;
    if ~isempty(boardVr), vOv = 100*nnz(v>=boardVr(1) & v<=boardVr(2))/numel(v); end

    % 旧质心法对照
    mO = Pts(:,2)>dNom-0.35 & Pts(:,2)<dNom+0.35 & abs(Pts(:,1))<0.45 & Pts(:,3)>-1.0 & Pts(:,3)<0.8;
    oldDu = NaN;
    if nnz(mO) >= 3
        cenc = (mean(Pts(mO,1:3),1) - t) * Rm;
        oldDu = K(1)*cenc(1)/cenc(2)+K(3) - boardU;
    end

    fprintf('%-24s %4d %+6.3f %6d %6.1f | %7.1f %+7.1f | %+7.1f | %5.0f%% | %+.1f\n', ...
        nm, size(Pl,1), stripX, size(Ps,1), stripU, boardU, du, dx_cm, vOv, oldDu);
    fprintf(fid,'%s,%.2f,%d,%.4f,%d,%.2f,%.2f,%.2f,%.2f,%.1f,%.2f\n', ...
        nm, dNom, size(Pl,1), stripX, size(Ps,1), stripU, boardU, du, dx_cm, vOv, oldDu);

    % --- 标注图 ---
    fg = figure('Visible','off','Color','w','Position',[100 100 960 720]);
    imshow(img); hold on;
    if ~isempty(corners)
        plot(corners(:,1), corners(:,2), 'gx', 'MarkerSize', 7, 'LineWidth', 1);
        plot(boardU, boardV, 'r+', 'MarkerSize', 22, 'LineWidth', 3);
        xline(boardU, 'r--', 'LineWidth', 1);
    end
    plot(u, v, '.', 'Color', [0 0.75 0.9], 'MarkerSize', 10);
    xline(stripU, 'm-', 'LineWidth', 2);
    title(sprintf('%s  条带u=%.1f vs 板心u=%.1f  du=%+.1fpx → dx=%+.1fcm  穿板率%.0f%%  (层%d点/条带%d点)', ...
        nm, stripU, boardU, du, dx_cm, vOv, size(Pl,1), size(Ps,1)), 'Interpreter','none');
    exportgraphics(fg, fullfile(outDir, [nm '_pv2.png']), 'Resolution', 150);
    close(fg);
end
fclose(fid);
fprintf('\n输出: %s\n', outDir);

    function R = Rz(a), s=sind(a); c=cosd(a); R=[c -s 0; s c 0; 0 0 1]; end
    function R = Ry(a), s=sind(a); c=cosd(a); R=[c 0 s; 0 1 0; -s 0 c]; end
    function R = Rx(a), s=sind(a); c=cosd(a); R=[1 0 0; 0 c -s; 0 s c]; end
end

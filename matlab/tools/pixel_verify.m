function pixel_verify()
%PIXEL_VERIFY 像素级独立验证：雷达板区点簇质心投影 vs 棋盘格板心像素
%  不使用深度残差，不做阈值染色，直接输出像素偏差与物理偏差

cd('G:\3.2\matlab'); addpath('tools','algorithm','algorithm/lib');
P = paths(); K = [491.4 655.7 319.5 239.5];
js = jsondecode(fileread(fullfile(P.resultsDir,'tall_result.json')));
t = [0.008 -0.048 0.130];   % 固定中位数 t

targets = {'20261104_134227_室内2米_2', 2.0;
           '20261104_134957_室内3米_2', 3.0;
           '20261104_162154_室内4米',   4.0};

outDir = fullfile(P.resultsDir,'pixel_verify'); if ~exist(outDir,'dir'), mkdir(outDir); end
fid = fopen(fullfile(outDir,'pixel_verify.csv'),'w');
fprintf(fid,'group,npts,boardU,boardV,centU,centV,du,dv,dx_cm,dz_cm,detOK,cenX,cenY,cenZ\n');

fprintf('\n%-26s %5s | %14s %14s | %6s %6s | %7s %7s | %s\n', ...
    '组','板区点数','板心(u,v)','质心投影(u,v)','du','dv','dx_cm','dz_cm','检测');
fprintf('%s\n', repmat('-',1,120));

for i = 1:size(targets,1)
    nm = targets{i,1}; dNom = targets{i,2};
    S = load(fullfile(P.takesDir,[nm '_res.mat']));
    k = min(numel(S.frames),10);

    yaw=0; pitch=0; roll=0;
    for gi=1:numel(js.groups)
        if strcmp(js.groups(gi).name,nm)
            yaw=js.groups(gi).yaw; pitch=js.groups(gi).pitch; roll=js.groups(gi).roll; break;
        end
    end

    cf = fullfile(S.metaOut.takeDir,'color',sprintf('%06d.png',S.colorFrame(k)));
    img = imread(cf);

    % --- 自动检测棋盘格内角点，板心=角点阵列中心 ---
    [boardU,boardV,detOK,corners] = deal(NaN,NaN,false,[]);
    try
        corners = detectCheckerboardPoints(img);
        if ~isempty(corners) && size(corners,1) >= 4
            boardU = mean(corners(:,1)); boardV = mean(corners(:,2)); detOK = true;
        end
    catch ME
        fprintf('[%s] detectCheckerboardPoints 不可用: %s\n', nm, ME.message);
    end

    % --- 板区雷达点（按标称距离截取） ---
    Pts = double(S.pts{k});
    m = Pts(:,2)>dNom-0.35 & Pts(:,2)<dNom+0.35 & abs(Pts(:,1))<0.45 & Pts(:,3)>-1.0 & Pts(:,3)<0.8;
    Pb = Pts(m,1:3);
    if size(Pb,1) < 3
        fprintf('%-26s 板区点不足(%d)\n', nm, size(Pb,1)); continue;
    end
    cen = mean(Pb,1);

    % --- 投影（与 visualize_alignment 同一链路） ---
    Rm = Rz(yaw)*Rx(pitch)*Ry(roll);
    Pc = (Pb - t) * Rm;
    xo=Pc(:,1); yo=-Pc(:,3); zo=Pc(:,2);
    okp = zo>0.25;
    u = K(1).*xo(okp)./zo(okp)+K(3); v = K(2).*yo(okp)./zo(okp)+K(4);
    inb = okp; inb(okp) = u>=1 & u<=640 & v>=1 & v<=480;
    uu = K(1).*xo(inb)./zo(inb)+K(3); vv = K(2).*yo(inb)./zo(inb)+K(4);

    cenc = (cen - t) * Rm;
    uc = K(1)*cenc(1)/cenc(2)+K(3); vc = K(2)*(-cenc(3))/cenc(2)+K(4);

    du = uc - boardU; dv = vc - boardV;
    dx_cm = du * dNom * 100 / K(1);
    dz_cm = -dv * dNom * 100 / K(2);

    fprintf('%-26s %5d | (%6.1f,%6.1f) (%6.1f,%6.1f) | %+6.1f %+6.1f | %+7.1f %+7.1f | %s\n', ...
        nm, size(Pb,1), boardU, boardV, uc, vc, du, dv, dx_cm, dz_cm, mat2str(detOK));
    fprintf(fid,'%s,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.3f,%.3f,%.3f\n', ...
        nm, size(Pb,1), boardU, boardV, uc, vc, du, dv, dx_cm, dz_cm, detOK, cen);

    % --- 标注图 ---
    fg = figure('Visible','off','Color','w','Position',[100 100 960 720]);
    imshow(img); hold on;
    if detOK && ~isempty(corners)
        plot(corners(:,1), corners(:,2), 'gx', 'MarkerSize', 8, 'LineWidth', 1.2);
        plot(boardU, boardV, 'r+', 'MarkerSize', 22, 'LineWidth', 3);
    end
    plot(uu, vv, '.', 'Color', [0 0.8 0.9], 'MarkerSize', 8);
    plot(uc, vc, 'm+', 'MarkerSize', 26, 'LineWidth', 3);
    legend({'棋盘角点','板心(检测)','雷达点簇','点簇质心投影'}, 'Location','northeast','Color',[0.15 0.15 0.15]);
    title(sprintf('%s  Δu=%+.1fpx Δv=%+.1fpx  →  Δx=%+.1fcm Δz=%+.1fcm  (板区%d点)', ...
        nm, du, dv, dx_cm, dz_cm, size(Pb,1)), 'Interpreter','none');
    exportgraphics(fg, fullfile(outDir, [nm '_pixel.png']), 'Resolution', 150);
    close(fg);
end
fclose(fid);
fprintf('\n输出: %s\n', outDir);

    function R = Rz(a), s=sind(a); c=cosd(a); R=[c -s 0; s c 0; 0 0 1]; end
    function R = Ry(a), s=sind(a); c=cosd(a); R=[c 0 s; 0 1 0; -s 0 c]; end
    function R = Rx(a), s=sind(a); c=cosd(a); R=[1 0 0; 0 c -s; 0 s c]; end
end

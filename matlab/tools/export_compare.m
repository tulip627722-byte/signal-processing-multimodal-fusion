function export_compare()
%EXPORT_COMPARE 导出成功 vs 不成功的投影对比图

cd('G:\3.2\matlab'); addpath('tools','algorithm','algorithm/lib','tools');
P = paths();
K = [491.4 655.7 319.5 239.5];
js = jsondecode(fileread(fullfile(P.resultsDir, 'tall_result.json')));
outDir = fullfile(P.resultsDir, 'good_vs_bad');
if ~isfolder(outDir), mkdir(outDir); end

files = dir(fullfile(P.takesDir, '*_res.mat'));
targets = {'20261104_134227_室内2米_2', '20261105_152723_室外3米', ...
           '20261104_162348_室内5米', '20261104_132932_室内2米5_1'};
Kthr = 15;

for ti = 1:numel(targets)
    nm = targets{ti};
    fi = find(contains({files.name}, nm));
    if isempty(fi), continue; end

    S = load(fullfile(P.takesDir, files(fi).name));
    k = min(numel(S.frames), 10);

    prm = struct('tx', 0.008, 'ty', -0.048, 'tz', 0.130, 'yaw', 0, 'pitch', 0, 'roll', 0);
    for gi = 1:numel(js.groups)
        if strcmp(js.groups(gi).name, nm)
            prm.tx = js.groups(gi).t(1); prm.ty = js.groups(gi).t(2); prm.tz = js.groups(gi).t(3);
            prm.yaw = js.groups(gi).yaw; prm.pitch = js.groups(gi).pitch; prm.roll = js.groups(gi).roll;
            break;
        end
    end

    Pts = double(S.pts{k});
    df = fullfile(S.metaOut.takeDir, 'depth', sprintf('%06d.png', S.depthFrame(k)));
    D = imread(df); if size(D,3)>1, D=D(:,:,1); end
    D = fliplr(double(D)) * 0.001; D(D < 0.3 | D > 9) = NaN;

    [u, v, ok] = proj(Pts(:,1:3), prm, K);
    ok = ok & u >= 1 & u <= 640 & v >= 1 & v <= 480;
    Rm = Rz(prm.yaw) * Rx(prm.pitch) * Ry(prm.roll);
    Pc = (Pts(:,1:3) - [prm.tx prm.ty prm.tz]) * Rm;
    zo_all = Pc(:, 2);
    resid_cm = nan(size(ok));
    if any(ok)
        dv = interp2(D, u(ok), v(ok), 'linear', NaN);
        resid_cm(ok) = abs(zo_all(ok) - dv) * 100;
    end
    valid = ok & isfinite(resid_cm) & resid_cm < 100 & zo_all > 0.3;

    cf = fullfile(S.metaOut.takeDir, 'color', sprintf('%06d.png', S.colorFrame(k)));
    img = imread(cf);

    fig = figure('Visible', 'off', 'Position', [0 0 1600 450], 'Color', 'w');

    ax1 = axes('Position', [0.02 0.08 0.30 0.85]);
    imshow(img); hold on;
    scatter(u(valid), v(valid), 12, resid_cm(valid), 'filled', 'LineWidth', 0.5);
    colormap(ax1, [0 0.85 0; 1 1 0; 1 0.5 0; 1 0 0]); caxis([0 Kthr*2]);
    md = median(resid_cm(valid & isfinite(resid_cm)));
    title(ax1, sprintf('%s  中位残差=%.1fcm', nm, md), 'Interpreter', 'none', 'FontSize', 9);

    ax2 = axes('Position', [0.35 0.08 0.30 0.85]);
    imagesc(D); colormap(ax2, parula); caxis([0.3 8]); axis image off; hold on;
    scatter(u(valid), v(valid), 12, resid_cm(valid), 'filled', 'LineWidth', 0.5);
    colormap(ax2, [0 0.85 0; 1 1 0; 1 0.5 0; 1 0 0]); caxis([0 Kthr*2]);
    title(ax2, '深度 + 雷达投影', 'FontSize', 9);

    ax3 = axes('Position', [0.68 0.08 0.30 0.85]);
    Dc = S.depthPts{k};
    if ~isempty(Dc)
        Rm2 = Rz(prm.yaw) * Rx(prm.pitch) * Ry(prm.roll);
        Dc = double(Dc(:,1:3)) * Rm2' + [prm.tx, prm.ty, prm.tz];
        smp = max(1, floor(size(Dc,1)/6000));
        scatter3(Dc(1:smp:end,1), Dc(1:smp:end,2), Dc(1:smp:end,3), 2, [0.7 0.75 0.85], 'filled');
        hold on;
    end
    scatter3(Pts(valid,1), Pts(valid,2), Pts(valid,3), 12, resid_cm(valid), 'filled');
    plot3(prm.tx, prm.ty, prm.tz, 'v', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');
    plot3(0, 0, 0, 's', 'MarkerSize', 14, 'MarkerFaceColor', [1 0.5 0], 'MarkerEdgeColor', 'k');
    view(-25, 22); grid on; axis equal;
    xlabel('X'); ylabel('Y(前)'); zlabel('Z(上)');
    title(ax3, sprintf('3D 红=相机 橙=雷达 t=[%.3f %.3f %.3f]', prm.tx, prm.ty, prm.tz), 'FontSize', 8);
    colormap(ax3, [0 0.85 0; 1 1 0; 1 0.5 0; 1 0 0]); caxis([0 Kthr*2]);

    out = fullfile(outDir, sprintf('%s_compare.png', nm));
    print(fig, out, '-dpng', '-r110');
    close(fig);

    fprintf('[%d] %-30s 中位残差=%.1fcm  绿=%3d 黄=%3d 红=%3d → %s\n', ...
        ti, nm, md, nnz(valid & resid_cm <= Kthr), ...
        nnz(valid & resid_cm > Kthr & resid_cm <= 2*Kthr), ...
        nnz(valid & resid_cm > 2*Kthr), out);
end
fprintf('\n完成，全部在 %s\n', outDir);

    function [u, v, ok] = proj(P, prm, Kloc)
        R = Rz(prm.yaw) * Rx(prm.pitch) * Ry(prm.roll);
        Pc = (P - [prm.tx, prm.ty, prm.tz]) * R;
        xo = Pc(:,1); yo = -Pc(:,3); zo = Pc(:,2);
        ok = zo > 0.25;
        u = nan(size(zo)); v = nan(size(zo));
        u(ok) = Kloc(1) .* xo(ok) ./ zo(ok) + Kloc(3);
        v(ok) = Kloc(2) .* yo(ok) ./ zo(ok) + Kloc(4);
    end
    function R = Rz(a), s=sind(a); c=cosd(a); R=[c -s 0; s c 0; 0 0 1]; end
    function R = Ry(a), s=sind(a); c=cosd(a); R=[c 0 s; 0 1 0; -s 0 c]; end
    function R = Rx(a), s=sind(a); c=cosd(a); R=[1 0 0; 0 c -s; 0 s c]; end
end

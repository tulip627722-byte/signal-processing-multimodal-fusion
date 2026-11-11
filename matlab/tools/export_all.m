function export_all()
%EXPORT_ALL 优先用 tall_result.json 里逐组解的姿态+t，没有就用默认批次值

K = [491.4 655.7 319.5 239.5];          % Astra Pro 彩色口径

W = 640; H = 480;
takesDir = 'G:\3.2\results\takes';
outDir = 'G:\3.2\results\overlay_check';
if ~isfolder(outDir), mkdir(outDir); end

% 读 tall_result.json 里的逐组姿态+解的 t
poseMap = containers.Map();
defaultT = struct('tx', 0.004, 'ty', -0.044, 'tz', 0.129);  % solve_t_all 加权平均
jsonFile = fullfile(takesDir, '..', 'tall_result.json');
if isfile(jsonFile)
    js = jsondecode(fileread(jsonFile));
    if isfield(js, 't_weighted_mean')
        defaultT.tx = js.t_weighted_mean(1);
        defaultT.ty = js.t_weighted_mean(2);
        defaultT.tz = js.t_weighted_mean(3);
    end
    if isfield(js, 'groups')
        for g = js.groups(:)'
            if g.resid < 0.5   % 只用残差合格的组
                poseMap(g.name) = struct('yaw', g.yaw, 'pitch', g.pitch, 'roll', g.roll, ...
                    'tx', g.t(1), 'ty', g.t(2), 'tz', g.t(3));
            end
        end
    end
    fprintf('载入 tall_result.json：%d 组有合格解的姿态，默认 t=[%.3f, %.3f, %.3f]\n', ...
        poseMap.Count, defaultT.tx, defaultT.ty, defaultT.tz);
else
    fprintf('未找到 tall_result.json，用硬编码默认值\n');
end

files = dir(fullfile(takesDir, '*_res.mat'));
fprintf('共 %d 组\n', numel(files));

for i = 1:numel(files)
    f = fullfile(takesDir, files(i).name);
    S = load(f);
    m = S.metaOut;
    k = max(1, min(numel(S.frames), 10));

    nm = m.takeName;
    if poseMap.isKey(nm)
        pm = poseMap(nm);
        prm = struct('tx', pm.tx, 'ty', pm.ty, 'tz', pm.tz, ...
                     'yaw', pm.yaw, 'pitch', pm.pitch, 'roll', pm.roll);
    else
        % 默认批次姿态
        if contains(nm, '_16')
            yawD = 1.5; pitchD = 10.0;
        elseif contains(nm, '室外')
            yawD = -1.0; pitchD = 3.0;
        else
            yawD = -4.3; pitchD = 5.4;
        end
        prm = struct('tx', defaultT.tx, 'ty', defaultT.ty, 'tz', defaultT.tz, ...
                     'yaw', yawD, 'pitch', pitchD, 'roll', 0);
    end

    % 图像
    cf = fullfile(m.takeDir, 'color', sprintf('%06d.png', S.colorFrame(k)));
    img = imread(cf);
    df = fullfile(m.takeDir, 'depth', sprintf('%06d.png', S.depthFrame(k)));
    dd = double(imread(df)) * 0.001;
    if size(dd, 3) > 1, dd = dd(:, :, 1); end
    dd = fliplr(dd); dd(dd < 0.3 | dd > 9) = NaN;

    P = double(S.pts{k});
    [u, v, ok] = proj(P(:, 1:3), prm, K, W, H);
    ok = ok & u >= 1 & u <= W & v >= 1 & v <= H;

    fig = figure('Visible', 'off', 'Position', [0 0 1500 420], 'Color', 'w');

    subplot(1, 3, 1);
    imshow(img); hold on;
    plot(u(ok), v(ok), 'o', 'MarkerSize', 6, 'MarkerFaceColor', [0 1 1], ...
         'MarkerEdgeColor', [1 0 0], 'LineWidth', 1.1);
    title(sprintf('%s 帧%d 点数%d', m.takeName, k, nnz(ok)), 'Interpreter', 'none', 'FontSize', 8);

    subplot(1, 3, 2);
    D2 = dd; D2(isnan(D2)) = 8;
    imagesc(D2); axis image off; colormap(gca, parula); caxis([0.3 8]); hold on;
    plot(u(ok), v(ok), 'o', 'MarkerSize', 6, 'MarkerFaceColor', [1 0 1], ...
         'MarkerEdgeColor', 'w', 'LineWidth', 1.1);
    title('深度 + 投影', 'FontSize', 8);

    subplot(1, 3, 3);
    Dc = S.depthPts{k};
    if ~isempty(Dc)
        Rm = Rz(prm.yaw) * Rx(prm.pitch) * Ry(prm.roll);
        Dc = double(Dc(:, 1:3)) * Rm' + [prm.tx, prm.ty, prm.tz];
        smp = max(1, floor(size(Dc, 1) / 5000));
        plot3(Dc(1:smp:end, 1), Dc(1:smp:end, 2), Dc(1:smp:end, 3), '.', ...
              'Color', [0.72 0.75 0.80], 'MarkerSize', 5); hold on;
    end
    inten = P(:, 4);
    c = inten;
    scatter3(P(ok, 1), P(ok, 2), P(ok, 3), 12, c(ok), 'filled');
    plot3(prm.tx, prm.ty, prm.tz, '^', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');
    view(-25, 18); grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('3D（色=强度）', 'FontSize', 8);
    set(gca, 'FontSize', 7);

    out = fullfile(outDir, sprintf('%s_f%d.png', m.takeName, k));
    print(fig, out, '-dpng', '-r110');
    close(fig);
    fprintf('[%2d/%2d] %s 姿态 yaw=%.1f pitch=%.1f  点数=%d\n', ...
        i, numel(files), m.takeName, prm.yaw, prm.pitch, nnz(ok));
end
fprintf('\n输出目录：%s\n', outDir);
end

function [u, v, ok] = proj(P, prm, K, W, H)
R = Rz(prm.yaw) * Rx(prm.pitch) * Ry(prm.roll);
Pc = (P - [prm.tx prm.ty prm.tz]) * R;
xo = Pc(:, 1); yo = -Pc(:, 3); zo = Pc(:, 2);
ok = zo > 0.25;
u = nan(size(zo)); v = nan(size(zo));
u(ok) = K(1) .* xo(ok) ./ zo(ok) + K(3);
v(ok) = K(2) .* yo(ok) ./ zo(ok) + K(4);
end

function R = Rz(a), s = sind(a); c = cosd(a); R = [c -s 0; s c 0; 0 0 1]; end
function R = Ry(a), s = sind(a); c = cosd(a); R = [c 0 s; 0 1 0; -s 0 c]; end
function R = Rx(a), s = sind(a); c = cosd(a); R = [1 0 0; 0 c -s; 0 s c]; end

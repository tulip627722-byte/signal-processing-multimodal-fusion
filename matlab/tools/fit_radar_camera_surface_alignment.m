function result = fit_radar_camera_surface_alignment(opts)
%FIT_RADAR_CAMERA_SURFACE_ALIGNMENT 用同步深度表面拟合受约束平移并留组验证。
%
% 固定轴映射 R；联合估计安装平移 t 与独立的雷达径向偏差 b：
%   p_radar_corrected = p_radar * (norm(p)-b)/norm(p)
%   p_radar_corrected = R*p_camera + t

if nargin < 1, opts = struct(); end
here = fileparts(mfilename('fullpath'));
matlabDir = fileparts(here);
root32 = fileparts(matlabDir);
o = defaults(opts, root32);
addpath(matlabDir, '-begin');
E = load_radar_camera_extrinsics();

names = { ...
    '20261104_132932_室内2米5_1', '20261104_133030_室内2米5_2', ...
    '20261104_133930_室内2米_1', '20261104_134227_室内2米_2', ...
    '20261104_134904_室内3米_1', '20261104_134957_室内3米_2'};
rng(o.randomSeed, 'twister');
holdoutIndex = randi(numel(names));
fitIndex = setdiff(1:numel(names), holdoutIndex);

sets = cell(size(names));
for i = 1:numel(names)
    sets{i} = collect_samples(names{i}, o);
end

lower = [-0.10, -0.10, 0.05, 0.05];
upper = [ 0.10,  0.10, 0.15, 0.30];
start = [0, 0, 0.10, 0.16];
q0 = inverse_logistic((start - lower) ./ (upper - lower));
objective = @(q) training_loss(decode(q, lower, upper), sets(fitIndex), E.cameraToRadarR);
options = optimset('Display', 'off', 'MaxIter', 160, 'MaxFunEvals', 350, ...
    'TolX', 2e-4, 'TolFun', 1e-5);
[q, loss] = fminsearch(objective, q0, options);
p = decode(q, lower, upper);
t = p(1:3); rangeBias = p(4);

fitMetrics = evaluate_sets(sets(fitIndex), E.cameraToRadarR, t, rangeBias);
holdoutMetrics = evaluate_sets(sets(holdoutIndex), E.cameraToRadarR, t, rangeBias);
[example, plotData] = validation_example(sets{holdoutIndex}, E.cameraToRadarR, t, rangeBias);
plot_result(plotData, names{holdoutIndex}, t, rangeBias, holdoutMetrics, o.outputPath);

result = struct('R', E.cameraToRadarR, 't', t, 'rangeBiasM', rangeBias, ...
    'objective', loss, 'takeNames', {names}, 'fitIndex', fitIndex, ...
    'holdoutIndex', holdoutIndex, 'holdoutTake', names{holdoutIndex}, ...
    'fitMetrics', fitMetrics, 'holdoutMetrics', holdoutMetrics, ...
    'example', example, 'randomSeed', o.randomSeed, 'outputPath', o.outputPath, ...
    'bounds', struct('lower', lower, 'upper', upper));
save(o.resultPath, 'result');

fprintf('\n最终受约束估计（随机留出整组未参与拟合）\n');
fprintf('  R = [1 0 0; 0 0 1; 0 -1 0]\n');
fprintf('  t = [%+.4f %+.4f %+.4f] m\n', t);
fprintf('  雷达径向偏差 b = %.4f m（单独处理，不混入 t）\n', rangeBias);
fprintf('  拟合集：|轴向残差| 中位 %.3f m，P90 %.3f m，<=20cm %.1f%%\n', ...
    fitMetrics.medianAbsM, fitMetrics.p90AbsM, 100*fitMetrics.within20cmRatio);
fprintf('  留出组：%s\n', names{holdoutIndex});
fprintf('  验证：|轴向残差| 中位 %.3f m，P90 %.3f m，<=20cm %.1f%%\n', ...
    holdoutMetrics.medianAbsM, holdoutMetrics.p90AbsM, 100*holdoutMetrics.within20cmRatio);
fprintf('  示例雷达坐标     [%+.3f %+.3f %+.3f] m\n', example.radarPoint);
fprintf('  示例相机表面->雷达[%+.3f %+.3f %+.3f] m\n', example.cameraSurfaceInRadar);
fprintf('  示例差值          [%+.3f %+.3f %+.3f] m，三维 %.3f m\n', ...
    example.delta, norm(example.delta));
fprintf('  验证图：%s\n  结果：%s\n\n', o.outputPath, o.resultPath);
end

function data = collect_samples(name, o)
resPath = fullfile(o.resultsRoot, [name '_res.mat']);
takeDir = fullfile(o.dataRoot, name);
S = load(resPath, 'pts', 'depthFrame', 'metaOut');
meta = jsondecode(fileread(fullfile(takeDir, 'meta.json')));
K = meta.camera.depthIntrinsics;
scale = double(meta.depth.depthScaleMetersPerUnit);
framePick = unique(round(linspace(1, numel(S.pts), min(o.framesPerTake, numel(S.pts)))));
samples = repmat(struct('radar', [], 'depth', [], 'fx', 0, 'fy', 0, 'cx', 0, 'cy', 0), numel(framePick), 1);
for j = 1:numel(framePick)
    k = framePick(j);
    radar = double(S.pts{k}(:, 1:3));
    if size(radar, 1) > o.pointsPerFrame
        radar = radar(round(linspace(1, size(radar, 1), o.pointsPerFrame)), :);
    end
    depthPath = fullfile(takeDir, 'depth', sprintf('%06d.png', S.depthFrame(k)));
    D = imread(depthPath);
    if size(D, 3) > 1, D = D(:, :, 1); end
    D = fliplr(double(D) * scale);
    samples(j).radar = radar;
    samples(j).depth = D;
    samples(j).fx = double(K.fx); samples(j).fy = double(K.fy);
    samples(j).cx = (size(D, 2) - 1) - double(K.cx);
    samples(j).cy = double(K.cy);
end
data = struct('name', name, 'samples', samples);
end

function loss = training_loss(p, sets, R)
metrics = evaluate_sets(sets, R, p(1:3), p(4));
loss = metrics.trimmedAbsM + 0.20*metrics.p90CappedM;
end

function metrics = evaluate_sets(sets, R, t, rangeBias)
allResidual = [];
for i = 1:numel(sets)
    frames = sets{i}.samples;
    for j = 1:numel(frames)
        [residual, ~, ~] = frame_residual(frames(j), R, t, rangeBias);
        allResidual = [allResidual; residual]; %#ok<AGROW>
    end
end
absResidual = abs(allResidual);
if isempty(absResidual)
    metrics = struct('count', 0, 'medianSignedM', inf, 'medianAbsM', inf, ...
        'trimmedAbsM', inf, 'p90AbsM', inf, 'p90CappedM', inf, 'within20cmRatio', 0);
    return;
end
capped = min(absResidual, 1.0);
sorted = sort(capped);
keep = sorted(1:max(1, round(0.75*numel(sorted))));
metrics = struct('count', numel(allResidual), 'medianSignedM', median(allResidual), ...
    'medianAbsM', median(absResidual), 'trimmedAbsM', mean(keep), ...
    'p90AbsM', prctile(absResidual, 90), 'p90CappedM', prctile(capped, 90), ...
    'within20cmRatio', mean(absResidual <= 0.20));
end

function [residual, correctedRadar, surfaceRadar] = frame_residual(frame, R, t, rangeBias)
radar = frame.radar;
range = vecnorm(radar, 2, 2);
correctedRadar = radar .* max(range - rangeBias, 0.1) ./ range;
camera = (R.' * (correctedRadar - t).').';
inFront = camera(:, 3) > 0.3;
camera = camera(inFront, :);
correctedRadar = correctedRadar(inFront, :);
u = frame.fx * camera(:, 1) ./ camera(:, 3) + frame.cx + 1;
v = frame.fy * camera(:, 2) ./ camera(:, 3) + frame.cy + 1;
inside = u >= 1 & u <= size(frame.depth, 2) & v >= 1 & v <= size(frame.depth, 1);
camera = camera(inside, :); correctedRadar = correctedRadar(inside, :);
u = u(inside); v = v(inside);
measured = interp2(frame.depth, u, v, 'linear', 0);
valid = measured > 0.3 & measured < 12;
camera = camera(valid, :); correctedRadar = correctedRadar(valid, :);
measured = measured(valid);
residual = camera(:, 3) - measured;
surfaceCamera = [camera(:,1).*measured./camera(:,3), ...
    camera(:,2).*measured./camera(:,3), measured];
surfaceRadar = (R * surfaceCamera.').'+t;
end

function [example, plotData] = validation_example(set, R, t, rangeBias)
best = struct('score', inf, 'radar', [], 'surface', [], 'residual', []);
for j = 1:numel(set.samples)
    [residual, radar, surface] = frame_residual(set.samples(j), R, t, rangeBias);
    score = median(abs(residual));
    if score < best.score
        best = struct('score', score, 'radar', radar, 'surface', surface, 'residual', residual);
    end
end
[~, order] = sort(abs(best.residual));
middle = order(max(1, round(numel(order)/2)));
example = struct('radarPoint', best.radar(middle,:), ...
    'cameraSurfaceInRadar', best.surface(middle,:), ...
    'delta', best.radar(middle,:) - best.surface(middle,:));
plotData = best;
end

function plot_result(data, takeName, t, bias, metrics, outputPath)
count = size(data.radar, 1);
pick = 1:max(1, floor(count/350)):count;
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1450 650]);
ax1 = subplot(1, 2, 1);
scatter3(ax1, data.surface(pick,1), data.surface(pick,2), data.surface(pick,3), ...
    16, [0.15 0.55 0.9], 'filled', 'DisplayName', '同步深度表面（转到雷达系）');
hold(ax1, 'on');
scatter3(ax1, data.radar(pick,1), data.radar(pick,2), data.radar(pick,3), ...
    28, [0.9 0.15 0.1], 'o', 'LineWidth', 1.1, 'DisplayName', '雷达点（已校正径向偏差）');
for k = pick(1:max(1, floor(numel(pick)/40)):end)
    plot3(ax1, [data.surface(k,1), data.radar(k,1)], ...
        [data.surface(k,2), data.radar(k,2)], [data.surface(k,3), data.radar(k,3)], ...
        '-', 'Color', [0.65 0.65 0.65], 'HandleVisibility', 'off');
end
axis(ax1, 'equal'); grid(ax1, 'on'); view(ax1, -35, 24);
xlabel(ax1, 'X / m'); ylabel(ax1, 'Y / m'); zlabel(ax1, 'Z / m');
title(ax1, sprintf('随机留出：%s', takeName), 'Interpreter', 'none');
legend(ax1, 'Location', 'best');

ax2 = subplot(1, 2, 2);
histogram(ax2, data.residual, -1:0.04:1, 'FaceColor', [0.2 0.55 0.85]);
grid(ax2, 'on'); xlabel(ax2, '雷达轴向距离 - 深度表面 / m'); ylabel(ax2, '点数');
title(ax2, sprintf(['t=[%+.3f,%+.3f,%+.3f] m, b=%.3f m\n' ...
    '|残差|中位 %.3f m，P90 %.3f m，<=20cm %.1f%%'], ...
    t, bias, metrics.medianAbsM, metrics.p90AbsM, 100*metrics.within20cmRatio));
exportgraphics(fig, outputPath, 'Resolution', 160);
close(fig);
end

function p = decode(q, lower, upper)
s = 1 ./ (1 + exp(-q));
p = lower + (upper - lower).*s;
end

function q = inverse_logistic(s)
s = min(max(s, 1e-6), 1-1e-6);
q = log(s ./ (1-s));
end

function o = defaults(o, root32)
d = struct('dataRoot', fullfile(root32, '..', '11-4'), ...
    'resultsRoot', fullfile(root32, 'results', 'takes'), ...
    'outputPath', fullfile(root32, 'results', 'radar_camera_surface_validation.png'), ...
    'resultPath', fullfile(root32, 'results', 'radar_camera_surface_alignment.mat'), ...
    'randomSeed', 1104, 'framesPerTake', 8, 'pointsPerFrame', 220);
fields = fieldnames(d);
for i = 1:numel(fields)
    if ~isfield(o, fields{i}), o.(fields{i}) = d.(fields{i}); end
end
end

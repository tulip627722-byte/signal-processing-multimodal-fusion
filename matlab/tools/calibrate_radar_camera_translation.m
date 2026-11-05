function result = calibrate_radar_camera_translation(opts)
%CALIBRATE_RADAR_CAMERA_TRANSLATION 用有板/无板数据拟合相机到雷达的平移。
%
% 固定坐标轴映射 R，只拟合数据能够约束的 t：
%   p_radar = R * p_camera + t
% 随机留出一组仅用于验证，不参与拟合。

if nargin < 1, opts = struct(); end
here = fileparts(mfilename('fullpath'));
matlabDir = fileparts(here);
root32 = fileparts(matlabDir);
o = defaults(opts, root32);
addpath(matlabDir, '-begin');

E = load_radar_camera_extrinsics();
R = E.cameraToRadarR;
groups = { ...
    '2.5m-1', '20261104_132932_室内2米5_1', '20261104_133249_室内2米5_3_（无板子）'; ...
    '2.5m-2', '20261104_133030_室内2米5_2', '20261104_133249_室内2米5_3_（无板子）'; ...
    '2.0m-1', '20261104_133930_室内2米_1',  '20261104_134333_室内2米_3（无板子）'; ...
    '2.0m-2', '20261104_134227_室内2米_2',  '20261104_134333_室内2米_3（无板子）'; ...
    '3.0m-1', '20261104_134904_室内3米_1',  '20261104_135142_室内3米_3（无板子）'; ...
    '3.0m-2', '20261104_134957_室内3米_2',  '20261104_135142_室内3米_3（无板子）'};

samples = repmat(struct('name', '', 'withTake', '', 'camera', nan(1, 3), ...
    'cameraInRadarAxes', nan(1, 3), 'radar', nan(1, 3), ...
    'translation', nan(1, 3), 'depthPixels', 0, 'radarWeight', 0), size(groups, 1), 1);

for i = 1:size(groups, 1)
    withDir = fullfile(o.dataRoot, groups{i, 2});
    noDir = fullfile(o.dataRoot, groups{i, 3});
    [cameraPoint, depthPixels] = depth_board_center(withDir, noDir);
    cameraInRadarAxes = (R * cameraPoint.').';
    radarPoint = radar_board_center(o.resultsRoot, groups{i, 2}, groups{i, 3}, ...
        cameraInRadarAxes, o);
    samples(i).name = groups{i, 1};
    samples(i).withTake = groups{i, 2};
    samples(i).camera = cameraPoint;
    samples(i).cameraInRadarAxes = cameraInRadarAxes;
    samples(i).radar = radarPoint.point;
    samples(i).translation = radarPoint.point - cameraInRadarAxes;
    samples(i).depthPixels = depthPixels;
    samples(i).radarWeight = radarPoint.weight;
end

valid = find(all(isfinite(vertcat(samples.translation)), 2));
if numel(valid) < 3
    error('只有 %d 组有效板子对应点，无法可靠拟合并留组验证。', numel(valid));
end
rng(o.randomSeed, 'twister');
holdout = valid(randi(numel(valid)));
fitIndex = setdiff(valid, holdout);
translations = vertcat(samples.translation);
t = median(translations(fitIndex, :), 1);

predicted = vertcat(samples.cameraInRadarAxes) + t;
observed = vertcat(samples.radar);
errors = observed - predicted;
errorNorm = vecnorm(errors, 2, 2);

fprintf('\n有板/无板坐标对应（单位 m）\n');
fprintf('%-8s  %-23s  %-23s  %-23s\n', '组', 'R*p_camera', 'p_radar', 'p_radar-R*p_camera');
for i = valid(:).'
    marker = ' ';
    if i == holdout, marker = '*'; end
    fprintf('%s%-7s  [%+.3f %+.3f %+.3f]  [%+.3f %+.3f %+.3f]  [%+.3f %+.3f %+.3f]\n', ...
        marker, samples(i).name, samples(i).cameraInRadarAxes, samples(i).radar, samples(i).translation);
end
fprintf('* = 随机留出验证组（未参与拟合）\n');
fprintf('拟合 t = [%+.4f %+.4f %+.4f] m\n', t);
fprintf('验证组 %s：预测 [%+.3f %+.3f %+.3f]，雷达 [%+.3f %+.3f %+.3f]\n', ...
    samples(holdout).name, predicted(holdout, :), observed(holdout, :));
fprintf('验证误差 Δ = [%+.3f %+.3f %+.3f] m，三维 %.3f m\n\n', ...
    errors(holdout, :), errorNorm(holdout));

plot_validation(samples, fitIndex, holdout, predicted, errors, t, o.outputPath);
result = struct('R', R, 't', t, 'samples', samples, 'fitIndex', fitIndex, ...
    'holdoutIndex', holdout, 'predictedRadar', predicted, 'errors', errors, ...
    'errorNorm', errorNorm, 'outputPath', o.outputPath, 'randomSeed', o.randomSeed);
save(o.resultPath, 'result');
fprintf('验证图：%s\n结果：%s\n', o.outputPath, o.resultPath);
end

function [point, count] = depth_board_center(withDir, noDir)
[withDepth, meta] = mean_depth(withDir);
[noDepth, ~] = mean_depth(noDir);
withDepth = fliplr(withDepth);
noDepth = fliplr(noDepth);
meta.cx = (size(withDepth, 2) - 1) - meta.cx;
delta = noDepth - withDepth;
[height, width] = size(withDepth);
[U, V] = meshgrid(0:width-1, 0:height-1);
mask = withDepth > 0.5 & noDepth > 0.5 & delta > 0.12 & delta < 5;
mask = mask & U > 0.12 * width & U < 0.88 * width & V > 0.08 * height & V < 0.92 * height;

if exist('bwconncomp', 'file') == 2
    components = bwconncomp(mask, 8);
    if components.NumObjects > 0
        scores = zeros(components.NumObjects, 1);
        for j = 1:components.NumObjects
            idx = components.PixelIdxList{j};
            [row, col] = ind2sub(size(mask), idx);
            centrality = hypot((median(col) - width/2) / width, (median(row) - height/2) / height);
            scores(j) = numel(idx) * max(median(delta(idx)), 0) / (1 + 3 * centrality);
        end
        [~, best] = max(scores);
        selected = false(size(mask));
        selected(components.PixelIdxList{best}) = true;
        mask = selected;
    end
end

if nnz(mask) < 80
    point = nan(1, 3); count = nnz(mask); return;
end
Z = withDepth(mask);
X = (U(mask) - meta.cx) .* Z / meta.fx;
Y = (V(mask) - meta.cy) .* Z / meta.fy;
point = [median(X), median(Y), median(Z)];
count = numel(Z);
end

function [M, camera] = mean_depth(takeDir)
meta = jsondecode(fileread(fullfile(takeDir, 'meta.json')));
K = meta.camera.depthIntrinsics;
camera = struct('fx', double(K.fx), 'fy', double(K.fy), ...
    'cx', double(K.cx), 'cy', double(K.cy));
scale = double(meta.depth.depthScaleMetersPerUnit);
files = dir(fullfile(takeDir, 'depth', '*.png'));
pick = unique(round(linspace(1, numel(files), min(16, numel(files)))));
stack = [];
for j = pick
    D = imread(fullfile(files(j).folder, files(j).name));
    if size(D, 3) > 1, D = D(:, :, 1); end
    Z = double(D) * scale;
    if isempty(stack), stack = zeros([size(Z), numel(pick)]); end
    stack(:, :, find(pick == j, 1)) = Z; %#ok<FNDSB>
end
stack(stack <= 0) = NaN;
M = median(stack, 3, 'omitnan');
M(~isfinite(M)) = 0;
end

function out = radar_board_center(resultsRoot, withName, noName, expected, o)
withData = load(fullfile(resultsRoot, [withName '_res.mat']), 'pts');
noData = load(fullfile(resultsRoot, [noName '_res.mat']), 'pts');
withPoints = vertcat(withData.pts{:});
noPoints = vertcat(noData.pts{:});

guess = expected + [0, o.initialRangeBiasM, o.initialHeightM];
xEdges = guess(1) + (-o.roiX: o.voxelM: o.roiX);
yEdges = guess(2) + (-o.roiY: o.voxelM: o.roiY);
zEdges = guess(3) + (-o.roiZ: o.voxelM: o.roiZ);
if numel(xEdges) < 3 || numel(yEdges) < 3 || numel(zEdges) < 3
    error('雷达标定 ROI 或 voxelM 设置不合法。');
end

withDensity = voxel_counts(withPoints, xEdges, yEdges, zEdges) / numel(withData.pts);
noDensity = voxel_counts(noPoints, xEdges, yEdges, zEdges) / numel(noData.pts);
density = withDensity - noDensity;
density(density < 0) = 0;

[~, order] = sort(density(:), 'descend');
positive = order(density(order) > 0);
positive = positive(1:min(o.topVoxels, numel(positive)));
if isempty(positive)
    out = struct('point', nan(1, 3), 'weight', 0); return;
end
[ix, iy, iz] = ind2sub(size(density), positive);
xCenter = (xEdges(1:end-1) + xEdges(2:end)) / 2;
yCenter = (yEdges(1:end-1) + yEdges(2:end)) / 2;
zCenter = (zEdges(1:end-1) + zEdges(2:end)) / 2;
centers = [xCenter(ix).', yCenter(iy).', zCenter(iz).'];
weights = density(positive);

peak = centers(1, :);
nearPeak = vecnorm(centers - peak, 2, 2) <= o.peakRadiusM;
centers = centers(nearPeak, :);
weights = weights(nearPeak);
point = sum(centers .* weights, 1) / sum(weights);
out = struct('point', point, 'weight', sum(weights));
end

function counts = voxel_counts(points, xEdges, yEdges, zEdges)
shape = [numel(xEdges)-1, numel(yEdges)-1, numel(zEdges)-1];
ix = floor((points(:, 1) - xEdges(1)) / (xEdges(2) - xEdges(1))) + 1;
iy = floor((points(:, 2) - yEdges(1)) / (yEdges(2) - yEdges(1))) + 1;
iz = floor((points(:, 3) - zEdges(1)) / (zEdges(2) - zEdges(1))) + 1;
valid = ix >= 1 & ix <= shape(1) & iy >= 1 & iy <= shape(2) & iz >= 1 & iz <= shape(3);
linear = sub2ind(shape, ix(valid), iy(valid), iz(valid));
counts = reshape(accumarray(linear, 1, [prod(shape), 1]), shape);
end

function plot_validation(samples, fitIndex, holdout, predicted, errors, t, outputPath)
observed = vertcat(samples.radar);
cameraAxes = vertcat(samples.cameraInRadarAxes);
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1450 650]);

ax1 = subplot(1, 2, 1);
scatter3(ax1, cameraAxes(fitIndex, 1), cameraAxes(fitIndex, 2), cameraAxes(fitIndex, 3), ...
    75, [0.2 0.45 0.9], 'filled', 'DisplayName', '相机点（仅转轴）');
hold(ax1, 'on');
scatter3(ax1, observed(fitIndex, 1), observed(fitIndex, 2), observed(fitIndex, 3), ...
    75, [0.9 0.2 0.15], 'filled', 'DisplayName', '雷达点');
for i = fitIndex(:).'
    plot3(ax1, [cameraAxes(i,1), observed(i,1)], [cameraAxes(i,2), observed(i,2)], ...
        [cameraAxes(i,3), observed(i,3)], ':', 'Color', [0.4 0.4 0.4]);
end
axis(ax1, 'equal'); grid(ax1, 'on'); view(ax1, -35, 22);
xlabel(ax1, 'X / m'); ylabel(ax1, 'Y / m'); zlabel(ax1, 'Z / m');
title(ax1, sprintf('拟合数据：t = [%+.3f, %+.3f, %+.3f] m', t));
legend(ax1, 'Location', 'best');

ax2 = subplot(1, 2, 2);
scatter3(ax2, predicted(holdout, 1), predicted(holdout, 2), predicted(holdout, 3), ...
    140, [0.15 0.65 0.25], 'filled', 'DisplayName', '相机经 R,t 预测');
hold(ax2, 'on');
scatter3(ax2, observed(holdout, 1), observed(holdout, 2), observed(holdout, 3), ...
    140, [0.9 0.15 0.1], 'filled', 'DisplayName', '雷达实测');
plot3(ax2, [predicted(holdout,1), observed(holdout,1)], ...
    [predicted(holdout,2), observed(holdout,2)], ...
    [predicted(holdout,3), observed(holdout,3)], 'k-', 'LineWidth', 2);
axis(ax2, 'equal'); grid(ax2, 'on'); view(ax2, -35, 22);
xlabel(ax2, 'X / m'); ylabel(ax2, 'Y / m'); zlabel(ax2, 'Z / m');
title(ax2, sprintf('随机留出 %s：Δ=[%+.3f,%+.3f,%+.3f] m，|Δ|=%.3f m', ...
    samples(holdout).name, errors(holdout,:), norm(errors(holdout,:))));
legend(ax2, 'Location', 'best');

exportgraphics(fig, outputPath, 'Resolution', 160);
close(fig);
end

function o = defaults(o, root32)
d = struct('dataRoot', fullfile(root32, '..', '11-4'), ...
    'resultsRoot', fullfile(root32, 'results', 'takes'), ...
    'outputPath', fullfile(root32, 'results', 'radar_camera_translation_validation.png'), ...
    'resultPath', fullfile(root32, 'results', 'radar_camera_translation_calibration.mat'), ...
    'randomSeed', 1104, 'initialRangeBiasM', 0.16, 'initialHeightM', 0.10, ...
    'voxelM', 0.10, 'roiX', 0.70, 'roiY', 0.70, 'roiZ', 0.70, ...
    'topVoxels', 30, 'peakRadiusM', 0.28);
fields = fieldnames(d);
for i = 1:numel(fields)
    if ~isfield(o, fields{i}), o.(fields{i}) = d.(fields{i}); end
end
end

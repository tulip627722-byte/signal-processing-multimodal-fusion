function result = visualize_multimodal_frame(takeDir, lidarExtrinsicsFile, opts)
%VISUALIZE_MULTIMODAL_FRAME Show radar, MID-70 and depth-camera clouds together.
% All three clouds are displayed in ADT7880 coordinates: X right, Y forward, Z up.

if nargin < 3, opts = struct(); end
opts = defaults(opts);
if ~isfolder(takeDir), error('Cannot find take directory: %s', takeDir); end
if ~isfile(lidarExtrinsicsFile), error('Cannot find LiDAR-to-radar extrinsics: %s', lidarExtrinsicsFile); end

here = fileparts(fileparts(mfilename('fullpath')));
addpath(here);
if isempty(opts.resFile)
    [~, takeName] = fileparts(takeDir);
    projectPaths = paths();
    opts.resFile = fullfile(projectPaths.takesDir, [takeName '_res.mat']);
end
if ~isfile(opts.resFile)
    if ~opts.computeIfMissing
        error('Missing processed radar/camera result: %s', opts.resFile);
    end
    fprintf('Processed result is missing; running compute_take first.\n');
    opts.resFile = compute_take(takeDir, opts.computeOptions);
end

processed = load(opts.resFile, 'frames', 'radarT', 'pts', 'depthPts', 'depthFrame', 'depthDt', 'metaOut');
if isempty(processed.frames), error('Processed result has no radar frames: %s', opts.resFile); end
frameIndex = select_frame(processed.frames, opts);
radarFrame = double(processed.frames(frameIndex));

pairs = pair_lidar_radar_frames(takeDir, opts.maxDeltaMs);
candidateRows = find(pairs.radar_frame == radarFrame);
if isempty(candidateRows)
    [~, nearest] = min(abs(pairs.radar_t_ms - processed.radarT(frameIndex)));
    candidateRows = nearest;
end
[~, localNearest] = min(abs(pairs.dt_ms(candidateRows)));
pairRow = candidateRows(localNearest);
if ~pairs.valid(pairRow) && ~opts.allowLargeTimeDelta
    error('Nearest LiDAR batch is %.2f ms from radar frame %d, exceeding %.2f ms.', ...
        pairs.dt_ms(pairRow), radarFrame, opts.maxDeltaMs);
end

lidarBatchNumber = double(pairs.lidar_batch(pairRow));
lidarPath = find_lidar_file(takeDir, lidarBatchNumber);
lidarBatch = read_livox_bin(lidarPath);
lidarExtrinsics = load_lidar_radar_extrinsics(lidarExtrinsicsFile);
lidarRadar = (lidarExtrinsics.lidarToRadarR * double(lidarBatch.points).').';
lidarRadar = lidarRadar + lidarExtrinsics.lidarOriginInRadarM;
radarPoints = double(processed.pts{frameIndex});
radarPoints = radarPoints(:, 1:min(3, size(radarPoints, 2)));
cameraRadar = double(processed.depthPts{frameIndex});

[lidarPlot, lidarReflectivity] = sample_points(lidarRadar, double(lidarBatch.reflectivity), opts.maxLidarPoints);
[cameraPlot, ~] = sample_points(cameraRadar, [], opts.maxCameraPoints);
[radarPlot, ~] = sample_points(radarPoints, [], opts.maxRadarPoints);
limits = cloud_limits([lidarPlot; cameraPlot; radarPlot]);

figureHandle = figure('Color', 'w', 'Name', sprintf('三路雷达坐标融合 — 帧 %d', radarFrame), ...
    'NumberTitle', 'off', 'Visible', opts.visible);
layout = tiledlayout(figureHandle, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

radarAxes = nexttile(layout, 1);
scatter_cloud(radarAxes, radarPlot, [], [0.90 0.15 0.12], opts.radarPointSize);
title(radarAxes, sprintf('ADT7880｜帧 %d｜%d 点', radarFrame, size(radarPoints, 1)));

lidarAxes = nexttile(layout, 2);
scatter_cloud(lidarAxes, lidarPlot, lidarReflectivity, [0.08 0.42 0.90], opts.lidarPointSize);
title(lidarAxes, sprintf('MID-70｜批次 %d｜%d 点｜Δt %+.1f ms', ...
    lidarBatchNumber, size(lidarRadar, 1), pairs.dt_ms(pairRow)));

cameraAxes = nexttile(layout, 3);
scatter_cloud(cameraAxes, cameraPlot, cameraPlot(:, 2), [0.15 0.72 0.30], opts.cameraPointSize);
title(cameraAxes, sprintf('深度相机｜帧 %d｜%d 点｜Δt %+.1f ms', ...
    processed.depthFrame(frameIndex), size(cameraRadar, 1), processed.depthDt(frameIndex)));

overlayAxes = nexttile(layout, 4);
hold(overlayAxes, 'on');
scatter_cloud(overlayAxes, cameraPlot, [], [0.20 0.72 0.30], opts.cameraPointSize);
scatter_cloud(overlayAxes, lidarPlot, [], [0.10 0.38 0.92], opts.lidarPointSize);
scatter_cloud(overlayAxes, radarPlot, [], [0.95 0.10 0.08], opts.radarPointSize);
legend(overlayAxes, {'深度相机', 'MID-70', 'ADT7880'}, 'Location', 'best');
title(overlayAxes, '三路叠加｜统一 ADT7880 雷达坐标');

axesList = [radarAxes, lidarAxes, cameraAxes, overlayAxes];
for axisHandle = axesList
    configure_axes(axisHandle, limits, opts.view);
end
linkprop(axesList, {'CameraPosition','CameraTarget','CameraUpVector','CameraViewAngle','XLim','YLim','ZLim'});
title(layout, sprintf('%s｜LiDAR外参 %s｜X右 / Y前 / Z上', ...
    processed.metaOut.takeName, extrinsics_status(lidarExtrinsics)), 'Interpreter', 'none');

if ~isempty(opts.savePath)
    exportgraphics(figureHandle, opts.savePath, 'Resolution', 180);
end
result = struct('figure', figureHandle, 'frameIndex', frameIndex, 'radarFrame', radarFrame, ...
    'lidarBatch', lidarBatchNumber, 'depthFrame', processed.depthFrame(frameIndex), ...
    'lidarRadar', lidarRadar, 'radarPoints', radarPoints, 'cameraRadar', cameraRadar, ...
    'lidarRadarDtMs', pairs.dt_ms(pairRow), 'cameraRadarDtMs', processed.depthDt(frameIndex), ...
    'lidarExtrinsicsStatus', extrinsics_status(lidarExtrinsics), 'resFile', opts.resFile);
fprintf('Visualized radar frame %d, LiDAR batch %d (%+.2f ms), depth frame %d (%+.2f ms).\n', ...
    result.radarFrame, result.lidarBatch, result.lidarRadarDtMs, result.depthFrame, result.cameraRadarDtMs);
end

function frameIndex = select_frame(frames, opts)
if ~isempty(opts.radarFrame)
    frameIndex = find(double(frames) == double(opts.radarFrame), 1);
    if isempty(frameIndex), error('Radar frame %d is not present in processed result.', opts.radarFrame); end
else
    frameIndex = max(1, min(numel(frames), round(opts.frameIndex)));
end
end

function path = find_lidar_file(takeDir, batchNumber)
candidate = fullfile(takeDir, 'lidar', sprintf('%06d.bin', batchNumber));
if isfile(candidate), path = candidate; return; end
files = dir(fullfile(takeDir, 'lidar', '*.bin'));
if batchNumber < 1 || batchNumber > numel(files), error('Cannot locate LiDAR batch %d.', batchNumber); end
path = fullfile(files(batchNumber).folder, files(batchNumber).name);
end

function [sampled, sampledValues] = sample_points(points, values, maximum)
if isempty(points), sampled = zeros(0, 3); sampledValues = []; return; end
count = size(points, 1);
if count > maximum, indices = round(linspace(1, count, maximum)); else, indices = 1:count; end
sampled = points(indices, 1:3);
if isempty(values), sampledValues = []; else, sampledValues = values(indices); end
end

function scatter_cloud(axisHandle, points, values, fallbackColor, pointSize)
if isempty(points)
    scatter3(axisHandle, NaN, NaN, NaN, pointSize, fallbackColor, 'filled');
elseif isempty(values)
    scatter3(axisHandle, points(:,1), points(:,2), points(:,3), pointSize, fallbackColor, 'filled');
else
    scatter3(axisHandle, points(:,1), points(:,2), points(:,3), pointSize, values, 'filled');
    colorbar(axisHandle);
end
end

function configure_axes(axisHandle, limits, cameraView)
axis(axisHandle, 'equal'); grid(axisHandle, 'on'); box(axisHandle, 'on');
xlabel(axisHandle, 'X 右 / m'); ylabel(axisHandle, 'Y 前 / m'); zlabel(axisHandle, 'Z 上 / m');
xlim(axisHandle, limits(1,:)); ylim(axisHandle, limits(2,:)); zlim(axisHandle, limits(3,:));
view(axisHandle, cameraView); rotate3d(ancestor(axisHandle, 'figure'), 'on');
end

function limits = cloud_limits(points)
if isempty(points), limits = [-3 3; 0 8; -2 2]; return; end
points = points(all(isfinite(points),2),:);
if isempty(points), limits = [-3 3; 0 8; -2 2]; return; end
limits = zeros(3,2);
for dimension = 1:3
    values = sort(points(:,dimension));
    low = values(max(1, floor(0.01*numel(values))));
    high = values(min(numel(values), ceil(0.99*numel(values))));
    padding = max(0.25, 0.08*max(high-low, 0.1));
    limits(dimension,:) = [low-padding, high+padding];
end
end

function value = extrinsics_status(extrinsics)
if isfield(extrinsics, 'status'), value = char(string(extrinsics.status)); else, value = 'unspecified'; end
if isfield(extrinsics, 'applicable') && ~extrinsics.applicable, value = [value ' / NOT APPLICABLE']; end
end

function opts = defaults(opts)
values = struct('frameIndex',1,'radarFrame',[],'resFile','','computeIfMissing',true, ...
    'computeOptions',struct(),'maxDeltaMs',30,'allowLargeTimeDelta',false, ...
    'maxLidarPoints',40000,'maxCameraPoints',40000,'maxRadarPoints',10000, ...
    'lidarPointSize',4,'cameraPointSize',3,'radarPointSize',28, ...
    'view',[-32 22],'visible','on','savePath','');
names = fieldnames(values);
for index = 1:numel(names)
    if ~isfield(opts,names{index}) || isempty(opts.(names{index})), opts.(names{index}) = values.(names{index}); end
end
end

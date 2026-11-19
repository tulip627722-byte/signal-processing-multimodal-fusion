function report = inspect_lidar_radar_take(takeDir, opts)
%INSPECT_LIDAR_RADAR_TAKE Validate one MID-70 + ADT7880 acquisition.
%
% report = inspect_lidar_radar_take(takeDir)
% report = inspect_lidar_radar_take(takeDir, struct('processRadar', true))
%
% The MID-70 files already contain Cartesian XYZ from the device SDK. This
% function reads those XYZ values directly; it does not reconstruct them
% from range/angle. ADT7880 points are different: they are generated from
% radar_adc.mat by the existing FFT/CFAR/angle pipeline when processRadar
% is enabled.

if nargin < 2, opts = struct(); end
opts = defaults(opts);
takeDir = char(string(takeDir));
if ~isfolder(takeDir), error('Take directory does not exist: %s', takeDir); end

required = {'meta.json','manifest.csv','radar_frames.csv','radar_adc.mat'};
for index = 1:numel(required)
    if ~isfile(fullfile(takeDir, required{index}))
        error('Missing required file %s in %s', required{index}, takeDir);
    end
end
lidarDir = fullfile(takeDir, 'lidar');
lidarFiles = dir(fullfile(lidarDir, '*.bin'));
if isempty(lidarFiles), error('No lidar/*.bin files in %s', takeDir); end
[~, order] = sort({lidarFiles.name});
lidarFiles = lidarFiles(order);

meta = jsondecode(fileread(fullfile(takeDir, 'meta.json')));
manifest = readtable(fullfile(takeDir, 'manifest.csv'), 'TextType', 'string');
radarFrames = readtable(fullfile(takeDir, 'radar_frames.csv'), 'TextType', 'string');
lidarRows = read_lidar_rows(takeDir, manifest, lidarFiles);
pairs = pair_lidar_radar_frames(takeDir, opts.maxDeltaMs);

sampleIndices = unique(round(linspace(1, numel(lidarFiles), min(opts.maxBatchesToRead, numel(lidarFiles)))));
sampledPoints = cell(numel(sampleIndices), 1);
sampledReflectivity = cell(numel(sampleIndices), 1);
sampledVersions = zeros(numel(sampleIndices), 1);
sampledCounts = zeros(numel(sampleIndices), 1);
invalidPoints = 0;
for outputIndex = 1:numel(sampleIndices)
    fileIndex = sampleIndices(outputIndex);
    batch = read_livox_bin(fullfile(lidarFiles(fileIndex).folder, lidarFiles(fileIndex).name));
    sampledVersions(outputIndex) = batch.version;
    sampledCounts(outputIndex) = size(batch.points, 1);
    valid = all(isfinite(batch.points), 2);
    invalidPoints = invalidPoints + sum(~valid);
    points = double(batch.points(valid, :));
    intensity = double(batch.reflectivity(valid));
    if size(points, 1) > opts.maxPointsPerBatch
        pick = round(linspace(1, size(points, 1), opts.maxPointsPerBatch));
        points = points(pick, :);
        intensity = intensity(pick);
    end
    sampledPoints{outputIndex} = points;
    sampledReflectivity{outputIndex} = intensity;
end
points = vertcat(sampledPoints{:});
reflectivity = vertcat(sampledReflectivity{:});
range = sqrt(sum(points.^2, 2));

received = double(radarFrames.received);
missing = double(radarFrames.missing);
expectedRadarPackets = sum(received + missing);
radarLossRate = sum(missing) / max(1, expectedRadarPackets);
radarLossRateExcludingTail = get_nested_number(meta, {'radar','lossRateExcludingTail'}, NaN);
if ~isfinite(radarLossRateExcludingTail)
    core = true(height(radarFrames), 1);
    if ~isempty(core) && missing(end) > 0, core(end) = false; end
    radarLossRateExcludingTail = sum(missing(core)) / max(1, sum(received(core) + missing(core)));
end
completeRadarFrames = sum(missing == 0 & received > 0);
pairAbs = abs(double(pairs.dt_ms));
validPair = logical(pairs.valid);

expectedLidarPoints = NaN;
if ismember('points', lidarRows.Properties.VariableNames)
    expectedLidarPoints = sum(double(lidarRows.points));
elseif isfield(meta, 'lidar') && isfield(meta.lidar, 'points')
    expectedLidarPoints = double(meta.lidar.points);
end
sampledOnly = numel(sampleIndices) < numel(lidarFiles);
source = get_nested_text(meta, {'lidar','source'}, 'unknown');
if all(sampledVersions == 3)
    coordinateFrame = 'MID-70 sensor: X forward / Y left / Z up';
else
    coordinateFrame = get_nested_text(meta, {'lidar','coordinateFrame'}, ...
        'legacy LIVOXBIN coordinate frame; see meta.json');
end

checks = strings(0, 1);
checks(end+1) = check_line(numel(lidarFiles) == height(lidarRows), 'PASS', 'FAIL', ...
    sprintf('LiDAR files=%d, timeline rows=%d', numel(lidarFiles), height(lidarRows)));
checks(end+1) = check_line(all(ismember(sampledVersions, [1 2 3])), 'PASS', 'FAIL', ...
    sprintf('LIVOXBIN versions sampled: %s', mat2str(unique(sampledVersions).')));
checks(end+1) = check_line(invalidPoints == 0 && ~isempty(points), 'PASS', 'FAIL', ...
    sprintf('Native XYZ sampled=%d, invalid=%d', size(points, 1), invalidPoints));
checks(end+1) = check_line(all(reflectivity >= 0 & reflectivity <= 255), 'PASS', 'FAIL', ...
    sprintf('Reflectivity range %.0f..%.0f', min_or_nan(reflectivity), max_or_nan(reflectivity)));
checks(end+1) = check_line(all(diff(double(lidarRows.first_t_rel_ms)) >= 0), 'PASS', 'FAIL', ...
    'LiDAR host timeline is monotonic');
checks(end+1) = check_line(all(diff((double(radarFrames.first_packet_t_rel_ms) + ...
    double(radarFrames.last_packet_t_rel_ms))/2) >= 0), 'PASS', 'FAIL', ...
    'Radar host timeline is monotonic');
checks(end+1) = check_line(mean(validPair) >= opts.minimumPairCoverage, 'PASS', 'WARN', ...
    sprintf('LiDAR/radar pairs within %.1f ms: %.1f%%', opts.maxDeltaMs, 100*mean(validPair)));
checks(end+1) = check_line(radarLossRateExcludingTail <= opts.maximumRadarLossRate, 'PASS', 'WARN', ...
    sprintf('Radar packet loss excluding stop-tail %.4f%% (total %.4f%%), complete frames %d/%d', ...
    100*radarLossRateExcludingTail, 100*radarLossRate, completeRadarFrames, height(radarFrames)));

[~, takeName] = fileparts(takeDir);
fprintf('\n===== LiDAR + radar acquisition check: %s =====\n', takeName);
fprintf('MID-70 data: native Cartesian XYZ + reflectivity (%s)\n', coordinateFrame);
fprintf('ADT7880 data: raw ADC; point cloud requires FFT/CFAR/angle processing.\n');
for index = 1:numel(checks), fprintf('%s\n', checks(index)); end
fprintf('Sampled XYZ bounds [m]: X %.3f..%.3f, Y %.3f..%.3f, Z %.3f..%.3f\n', ...
    min_or_nan(points(:,1)), max_or_nan(points(:,1)), min_or_nan(points(:,2)), ...
    max_or_nan(points(:,2)), min_or_nan(points(:,3)), max_or_nan(points(:,3)));
fprintf('Sampled range [m]: median %.3f, p99 %.3f, max %.3f\n', ...
    median_or_nan(range), percentile(range, 99), max_or_nan(range));
fprintf('Time delta |LiDAR-radar| [ms]: median %.3f, p95 %.3f, max %.3f\n', ...
    median_or_nan(pairAbs), percentile(pairAbs, 95), max_or_nan(pairAbs));
if sampledOnly
    fprintf('XYZ statistics use %d/%d sampled batches; timeline checks use all rows.\n', ...
        numel(sampleIndices), numel(lidarFiles));
end
fprintf('Important: before LiDAR-to-radar extrinsics are calibrated, compare the clouds side by side only.\n');

figureHandle = [];
if opts.makeFigure
    figureHandle = make_figure(takeName, points, reflectivity, pairs, opts);
end

resFile = '';
radarFrame = NaN;
radarPoints = zeros(0, 3);
if opts.processRadar
    here = fileparts(fileparts(mfilename('fullpath')));
    addpath(here);
    computeOptions = opts.computeOptions;
    if ~isfield(computeOptions, 'maxFrames') || isempty(computeOptions.maxFrames)
        computeOptions.maxFrames = opts.maxRadarFrames;
    end
    temporaryResult = [tempname '.mat'];
    resultCleanup = onCleanup(@() delete_if_exists(temporaryResult)); %#ok<NASGU>
    computeOptions.outputFile = temporaryResult;
    [resFile, ~] = compute_take(takeDir, computeOptions);
    processed = load(resFile, 'frames', 'radarT', 'pts');
    [~, pairIndex] = min(pairAbs);
    targetFrame = double(pairs.radar_frame(pairIndex));
    frameIndex = find(double(processed.frames) == targetFrame, 1);
    if isempty(frameIndex)
        [~, frameIndex] = min(abs(double(processed.radarT) - double(pairs.radar_t_ms(pairIndex))));
    end
    radarFrame = double(processed.frames(frameIndex));
    radarPoints = double(processed.pts{frameIndex});
    radarPoints = radarPoints(:, 1:min(3, size(radarPoints, 2)));
    make_radar_comparison(takeName, points, reflectivity, radarPoints, radarFrame, ...
        pairs.dt_ms(pairIndex), opts);
end

function delete_if_exists(path)
if isfile(path), delete(path); end
end

report = struct('takeDir', takeDir, 'takeName', takeName, 'lidarSource', source, ...
    'coordinateFrame', coordinateFrame, 'lidarFiles', numel(lidarFiles), ...
    'expectedLidarPoints', expectedLidarPoints, 'sampledBatchIndices', sampleIndices, ...
    'sampledPointCounts', sampledCounts, 'sampledPoints', size(points, 1), ...
    'xyzMinM', column_stat(points, @min), 'xyzMaxM', column_stat(points, @max), ...
    'rangeMedianM', median_or_nan(range), 'rangeP99M', percentile(range, 99), ...
    'reflectivityMin', min_or_nan(reflectivity), 'reflectivityMax', max_or_nan(reflectivity), ...
    'radarFrames', height(radarFrames), 'completeRadarFrames', completeRadarFrames, ...
    'radarLossRate', radarLossRate, 'radarLossRateExcludingTail', radarLossRateExcludingTail, ...
    'pairs', pairs, 'pairCoverage', mean(validPair), ...
    'pairMedianAbsMs', median_or_nan(pairAbs), 'pairP95AbsMs', percentile(pairAbs, 95), ...
    'checks', checks, 'figure', figureHandle, 'processedRadarFile', resFile, ...
    'processedRadarFrame', radarFrame, 'processedRadarPoints', radarPoints);
end

function rows = read_lidar_rows(takeDir, manifest, files)
path = fullfile(takeDir, 'lidar_frames.csv');
if isfile(path)
    rows = readtable(path, 'TextType', 'string');
    return;
end
rows = manifest(lower(strtrim(string(manifest.stream))) == "lidar", :);
rows = table(double(rows.frame), double(rows.t_rel_ms), double(rows.t_rel_ms), ...
    NaN(height(rows), 1), string(rows.file), ...
    'VariableNames', {'batch','first_t_rel_ms','last_t_rel_ms','points','file'});
if height(rows) ~= numel(files)
    warning('LiDAR manifest rows (%d) differ from files (%d).', height(rows), numel(files));
end
end

function figureHandle = make_figure(takeName, points, reflectivity, pairs, opts)
figureHandle = figure('Color', 'w', 'Name', ['LiDAR + radar check - ' takeName], ...
    'NumberTitle', 'off', 'Visible', opts.visible);
layout = tiledlayout(figureHandle, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
axisHandle = nexttile(layout, 1, [2 1]);
scatter3(axisHandle, points(:,1), points(:,2), points(:,3), opts.lidarPointSize, reflectivity, 'filled');
axis(axisHandle, 'equal'); grid(axisHandle, 'on'); box(axisHandle, 'on');
xlabel(axisHandle, 'X forward / m'); ylabel(axisHandle, 'Y left / m'); zlabel(axisHandle, 'Z up / m');
title(axisHandle, sprintf('MID-70 native XYZ | %d sampled points', size(points,1)));
colorbar(axisHandle); view(axisHandle, opts.lidarView);

timeAxis = nexttile(layout, 2);
plot(timeAxis, pairs.lidar_t_ms/1000, pairs.dt_ms, '.', 'MarkerSize', 8); hold(timeAxis, 'on');
yline(timeAxis, opts.maxDeltaMs, '--r'); yline(timeAxis, -opts.maxDeltaMs, '--r');
grid(timeAxis, 'on'); xlabel(timeAxis, 'take time / s'); ylabel(timeAxis, 'LiDAR - radar / ms');
title(timeAxis, 'Nearest-frame time difference');

histogramAxis = nexttile(layout, 4);
histogram(histogramAxis, abs(pairs.dt_ms), max(10, min(50, round(height(pairs)/5))));
xline(histogramAxis, opts.maxDeltaMs, '--r'); grid(histogramAxis, 'on');
xlabel(histogramAxis, '|time difference| / ms'); ylabel(histogramAxis, 'pairs');
title(histogramAxis, sprintf('Within limit: %.1f%%', 100*mean(pairs.valid)));
title(layout, [takeName ' | LiDAR native XYZ; radar ADC timeline'], 'Interpreter', 'none');
end

function make_radar_comparison(takeName, lidarPoints, reflectivity, radarPoints, radarFrame, dtMs, opts)
figureHandle = figure('Color', 'w', 'Name', ['LiDAR vs processed radar - ' takeName], ...
    'NumberTitle', 'off', 'Visible', opts.visible);
layout = tiledlayout(figureHandle, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
lidarAxis = nexttile(layout);
scatter3(lidarAxis, lidarPoints(:,1), lidarPoints(:,2), lidarPoints(:,3), ...
    opts.lidarPointSize, reflectivity, 'filled');
axis(lidarAxis, 'equal'); grid(lidarAxis, 'on'); box(lidarAxis, 'on'); view(lidarAxis, opts.lidarView);
xlabel(lidarAxis, 'X forward'); ylabel(lidarAxis, 'Y left'); zlabel(lidarAxis, 'Z up');
title(lidarAxis, 'MID-70 native sensor frame'); colorbar(lidarAxis);
radarAxis = nexttile(layout);
if isempty(radarPoints)
    text(radarAxis, 0.5, 0.5, 'No radar detections', 'HorizontalAlignment', 'center');
else
    scatter3(radarAxis, radarPoints(:,1), radarPoints(:,2), radarPoints(:,3), ...
        opts.radarPointSize, 'r', 'filled');
end
axis(radarAxis, 'equal'); grid(radarAxis, 'on'); box(radarAxis, 'on'); view(radarAxis, opts.radarView);
xlabel(radarAxis, 'X right'); ylabel(radarAxis, 'Y forward'); zlabel(radarAxis, 'Z up');
title(radarAxis, sprintf('ADT7880 processed frame %d | %d points', radarFrame, size(radarPoints,1)));
title(layout, sprintf('%s | nearest time delta %+.2f ms | no spatial overlay before calibration', ...
    takeName, dtMs), 'Interpreter', 'none');
end

function line = check_line(condition, passWord, failWord, message)
if condition, status = passWord; else, status = failWord; end
line = "[" + status + "] " + string(message);
end

function value = get_nested_text(value, fields, fallback)
try
    for index = 1:numel(fields), value = value.(fields{index}); end
    value = char(string(value));
catch
    value = fallback;
end
end

function result = get_nested_number(value, fields, fallback)
try
    for index = 1:numel(fields), value = value.(fields{index}); end
    result = double(value);
    if ~isscalar(result), result = fallback; end
catch
    result = fallback;
end
end

function result = column_stat(points, functionHandle)
if isempty(points), result = [NaN NaN NaN]; else, result = functionHandle(points, [], 1); end
end

function value = min_or_nan(values)
if isempty(values), value = NaN; else, value = min(values); end
end

function value = max_or_nan(values)
if isempty(values), value = NaN; else, value = max(values); end
end

function value = median_or_nan(values)
values = values(isfinite(values));
if isempty(values), value = NaN; else, value = median(values); end
end

function value = percentile(values, percentage)
values = sort(values(isfinite(values)));
if isempty(values), value = NaN; return; end
index = 1 + (numel(values)-1)*percentage/100;
lower = floor(index); upper = ceil(index);
if lower == upper, value = values(lower); return; end
value = values(lower) + (index-lower)*(values(upper)-values(lower));
end

function opts = defaults(opts)
values = struct('maxDeltaMs',30,'minimumPairCoverage',0.95,'maximumRadarLossRate',0.01, ...
    'maxBatchesToRead',60,'maxPointsPerBatch',5000,'makeFigure',true, ...
    'processRadar',false,'maxRadarFrames',12,'computeOptions',struct(), ...
    'lidarPointSize',3,'radarPointSize',24,'lidarView',[-35 22], ...
    'radarView',[-35 22],'visible','on');
names = fieldnames(values);
for index = 1:numel(names)
    if ~isfield(opts, names{index}) || isempty(opts.(names{index}))
        opts.(names{index}) = values.(names{index});
    end
end
end

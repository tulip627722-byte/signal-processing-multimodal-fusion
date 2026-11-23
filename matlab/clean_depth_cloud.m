function [cleanPoints, keepMask, stats, sourceIndices] = clean_depth_cloud(points, uv, imageSize, opts)
%CLEAN_DEPTH_CLOUD Remove depth edge flyers and isolated 3-D samples.

if nargin < 4, opts = struct(); end
o = defaults(opts);
points = double(points);
if isempty(points)
    cleanPoints = zeros(0, 3); keepMask = false(0, 1); sourceIndices = zeros(0,1);
    stats = empty_stats(0); return;
end
if size(points, 2) ~= 3, error('points 必须为 N×3。'); end
uv = double(uv);
validateattributes(uv, {'numeric'}, {'size', [size(points, 1), 2]});
validateattributes(imageSize, {'numeric'}, {'numel', 2, 'integer', 'positive'});

n = size(points, 1);
finiteMask = all(isfinite(points), 2);
rangeMask = points(:, 2) >= o.rangeM(1) & points(:, 2) <= o.rangeM(2);
roiMask = points(:, 1) >= o.roiX(1) & points(:, 1) <= o.roiX(2) & ...
    points(:, 2) >= o.roiY(1) & points(:, 2) <= o.roiY(2) & ...
    points(:, 3) >= o.roiZ(1) & points(:, 3) <= o.roiZ(2);
baseMask = finiteMask & rangeMask & roiMask;

edgeMask = false(n, 1);
componentMask = false(n, 1);
validUv = uv(:, 1) >= 0 & uv(:, 1) < imageSize(2) & ...
    uv(:, 2) >= 0 & uv(:, 2) < imageSize(1);
active = find(baseMask & validUv);
if ~isempty(active)
    depthImage = nan(imageSize);
    linear = sub2ind(imageSize, uv(active, 2) + 1, uv(active, 1) + 1);
    depthImage(linear) = points(active, 2);
    [edgeKeepImage, componentKeepImage] = image_filters(depthImage, o);
    edgeMask(active) = edgeKeepImage(linear);
    componentMask(active) = componentKeepImage(linear);
end

preVoxelMask = baseMask & validUv & edgeMask & componentMask;
preVoxelPoints = points(preVoxelMask, :);
preVoxelIndices = find(preVoxelMask);
if isempty(preVoxelPoints)
    cleanPoints = zeros(0, 3); keepMask = false(n, 1); sourceIndices = zeros(0,1);
    stats = make_stats(n, finiteMask, rangeMask, roiMask, edgeMask, ...
        componentMask, preVoxelMask, 0, 0); return;
end

[voxelPoints, voxelSource] = voxel_average(preVoxelPoints, preVoxelIndices, o.voxelM);
if o.minVoxelNeighbors > 0 && ~isempty(voxelPoints)
    neighbourCount = voxel_neighbour_count(voxelPoints, o.voxelM);
    voxelKeep = neighbourCount >= o.minVoxelNeighbors;
else
    voxelKeep = true(size(voxelPoints, 1), 1);
end
cleanPoints = voxelPoints(voxelKeep, :);
keepMask = false(n, 1);
keptSource = voxelSource(voxelKeep);
sourceIndices = keptSource;
keepMask(keptSource) = true;
stats = make_stats(n, finiteMask, rangeMask, roiMask, edgeMask, ...
    componentMask, preVoxelMask, size(voxelPoints, 1), size(cleanPoints, 1));
end

function [edgeKeep, componentKeep] = image_filters(depthImage, o)
valid = isfinite(depthImage);
filled = depthImage; filled(~valid) = inf;
neighbourDepth = zeros([size(depthImage), 9]);
neighbourValid = false([size(depthImage), 9]);
slot = 0;
for rowOffset = -1:1
    for columnOffset = -1:1
        slot = slot + 1;
        neighbourDepth(:, :, slot) = shift_no_wrap(filled, rowOffset, columnOffset, inf);
        neighbourValid(:, :, slot) = shift_no_wrap(valid, rowOffset, columnOffset, false);
    end
end
sortedDepth = sort(neighbourDepth, 3);
validCount = sum(neighbourValid, 3);
medianIndex = max(1, ceil(validCount / 2));
pixelIndex = repmat(reshape(1:numel(depthImage), size(depthImage)), 1, 1, 1) + ...
    (medianIndex - 1) * numel(depthImage);
localMedian = sortedDepth(pixelIndex);
threshold = max(o.edgeThresholdM, o.edgeRelativeThreshold .* max(localMedian, 0));
edgeKeep = valid & validCount >= o.minimumLocalSamples & ...
    abs(depthImage - localMedian) <= threshold;

% A 3x3 support test removes isolated islands without an Image Processing
% Toolbox dependency or a full-frame flood fill on every recorded frame.
localSupport = conv2(double(edgeKeep), ones(3), 'same');
componentKeep = edgeKeep & localSupport >= min(o.minimumComponentPixels, 9);
end

function shifted = shift_no_wrap(array, rowOffset, columnOffset, fillValue)
shifted = repmat(fillValue, size(array));
sourceRows = max(1, 1-rowOffset):min(size(array,1), size(array,1)-rowOffset);
sourceColumns = max(1, 1-columnOffset):min(size(array,2), size(array,2)-columnOffset);
targetRows = sourceRows + rowOffset; targetColumns = sourceColumns + columnOffset;
shifted(targetRows, targetColumns) = array(sourceRows, sourceColumns);
end

function [averaged, sourceIndices] = voxel_average(points, originalIndices, voxelM)
if voxelM <= 0
    averaged = points; sourceIndices = originalIndices; return;
end
voxel = floor(points ./ voxelM);
[~, ~, group] = unique(voxel, 'rows');
count = accumarray(group, 1);
averaged = [accumarray(group, points(:,1)), accumarray(group, points(:,2)), ...
    accumarray(group, points(:,3))] ./ count;
sourceIndices = accumarray(group, originalIndices, [], @min);
end

function count = voxel_neighbour_count(points, voxelM)
voxel = floor(points ./ voxelM);
voxel = voxel - min(voxel, [], 1) + 1;
dimensions = max(voxel, [], 1) + 2;
key = voxel(:,1) + dimensions(1) .* ((voxel(:,2)-1) + dimensions(2) .* (voxel(:,3)-1));
count = zeros(size(points, 1), 1);
for xOffset = -1:1
    for yOffset = -1:1
        for zOffset = -1:1
            if xOffset == 0 && yOffset == 0 && zOffset == 0, continue; end
            shifted = voxel + [xOffset yOffset zOffset];
            inside = all(shifted >= 1, 2) & shifted(:,1) <= dimensions(1)-2 & ...
                shifted(:,2) <= dimensions(2)-2 & shifted(:,3) <= dimensions(3)-2;
            shiftedKey = shifted(:,1) + dimensions(1) .* ...
                ((shifted(:,2)-1) + dimensions(2) .* (shifted(:,3)-1));
            count(inside) = count(inside) + ismember(shiftedKey(inside), key);
        end
    end
end
end

function stats = make_stats(total, finiteMask, rangeMask, roiMask, edgeMask, componentMask, preVoxelMask, voxelCount, outputCount)
stats = struct('inputCount', total, 'finiteCount', nnz(finiteMask), ...
    'rangeCount', nnz(finiteMask & rangeMask), ...
    'roiCount', nnz(finiteMask & rangeMask & roiMask), ...
    'edgeCount', nnz(finiteMask & rangeMask & roiMask & edgeMask), ...
    'componentCount', nnz(preVoxelMask & componentMask), ...
    'preVoxelCount', nnz(preVoxelMask), 'voxelCount', voxelCount, ...
    'outputCount', outputCount, 'retainedRatio', outputCount / max(total, 1));
end

function stats = empty_stats(total)
stats = make_stats(total, false(total,1), false(total,1), false(total,1), ...
    false(total,1), false(total,1), false(total,1), 0, 0);
end

function o = defaults(o)
d = struct('rangeM', [0.3 12], 'edgeThresholdM', 0.025, ...
    'edgeRelativeThreshold', 0.015, 'minimumLocalSamples', 4, ...
    'minimumComponentPixels', 6, 'roiX', [-3 3], 'roiY', [0 9], ...
    'roiZ', [-1.4 1.6], 'voxelM', 0.015, 'minVoxelNeighbors', 1);
fields = fieldnames(d);
for index = 1:numel(fields)
    if ~isfield(o, fields{index}), o.(fields{index}) = d.(fields{index}); end
end
end

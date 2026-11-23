function [pointsOut, attributesOut] = voxel_reduce_points(points, attributes, voxelSize)
%VOXEL_REDUCE_POINTS Average points and aligned attributes per voxel.
if nargin < 2 || isempty(attributes), attributes = zeros(size(points, 1), 0); end
if nargin < 3 || voxelSize <= 0
    pointsOut = double(points); attributesOut = attributes; return;
end
points = double(points(:, 1:3)); valid = all(isfinite(points), 2);
points = points(valid, :); attributes = attributes(valid, :);
if isempty(points), pointsOut = zeros(0, 3); attributesOut = zeros(0, size(attributes, 2)); return; end
voxel = floor(points ./ voxelSize); [~, ~, group] = unique(voxel, 'rows');
count = accumarray(group, 1);
pointsOut = [accumarray(group, points(:,1)), accumarray(group, points(:,2)), ...
    accumarray(group, points(:,3))] ./ count;
attributesOut = zeros(max(group), size(attributes, 2));
for column = 1:size(attributes, 2)
    values = attributes(:, column); finiteValues = isfinite(values);
    attributesOut(:, column) = accumarray(group(finiteValues), values(finiteValues), ...
        [max(group), 1], @mean, NaN);
end
end

function report = diagnose_depth_lidar_alignment(data, opts)
%DIAGNOSE_DEPTH_LIDAR_ALIGNMENT Quantify static Depth/LiDAR mismatch.
if nargin < 2, opts = data.config; end
depth = sample_points(double(data.depth.radar), opts.maxDiagnosticPoints);
lidar = sample_points(double(data.lidar.radar), opts.maxDiagnosticPoints);
if isempty(depth) || isempty(lidar), error('Depth 或 MID-70 点云为空。'); end
[distance, nearest] = nearest_distances(depth, lidar);
report = struct('before', metrics(distance, opts.maxNearestDistanceM), ...
    'distanceBins', distance_by_range(depth, lidar, opts), ...
    'nearestIndices', nearest, 'plane', plane_diagnostics(depth, lidar));
report.status = ternary(report.before.medianM <= 0.15, 'PASS', 'WARN');
end

function [distance, nearest] = nearest_distances(source, target)
distance = zeros(size(source, 1), 1); nearest = zeros(size(source, 1), 1);
for index = 1:size(source, 1)
    delta = target - source(index, :);
    [distance(index), nearest(index)] = min(sqrt(sum(delta.^2, 2)));
end
end

function result = metrics(distance, cutoff)
inlier = distance <= cutoff; values = distance(inlier);
if isempty(values), values = distance; end
result = struct('count', numel(distance), 'inlierCount', nnz(inlier), ...
    'meanM', mean(values), 'medianM', median(values), ...
    'rmseM', sqrt(mean(values.^2)), 'p95M', percentile(values, 95));
end

function output = distance_by_range(depth, lidar, opts)
edges = [0.8 1.8 2.8 4.0 6.0 12.0];
output = struct('rangeM', {}, 'metrics', {});
for index = 1:numel(edges)-1
    mask = depth(:,2) >= edges(index) & depth(:,2) < edges(index+1);
    if ~any(mask), continue; end
    [distance, ~] = nearest_distances(depth(mask,:), lidar);
    output(end+1) = struct('rangeM', edges(index:index+1), ...
        'metrics', metrics(distance, opts.maxNearestDistanceM)); %#ok<AGROW>
end
end

function result = plane_diagnostics(depth, lidar)
result = struct('available', false, 'normalAngleDeg', NaN, ...
    'depthSignedM', [], 'lidarSignedM', []);
if size(depth,1) < 30 || size(lidar,1) < 30, return; end
[normalL, offsetL] = fit_plane(lidar(lidar(:,3) < percentile(lidar(:,3),35),:));
[normalD, ~] = fit_plane(depth(depth(:,3) < percentile(depth(:,3),35),:));
if isempty(normalL) || isempty(normalD), return; end
result.available = true;
result.normalAngleDeg = acosd(min(1, abs(dot(normalL, normalD))));
result.depthSignedM = depth * normalL(:) + offsetL;
result.lidarSignedM = lidar * normalL(:) + offsetL;
end

function [normal, offset] = fit_plane(points)
if size(points,1) < 3, normal = []; offset = []; return; end
center = mean(points,1); [~,~,V] = svd(points-center,0);
normal = V(:,3).'; offset = -dot(normal, center);
end

function points = sample_points(points, maximum)
if size(points,1) <= maximum, return; end
points = points(unique(round(linspace(1,size(points,1),maximum))),:);
end

function value = percentile(values, p)
values = sort(values(:)); if isempty(values), value = NaN; return; end
position = 1 + (numel(values)-1)*p/100; low = floor(position); high = ceil(position);
if low == high, value = values(low); else, value = values(low) + (position-low)*(values(high)-values(low)); end
end

function value = ternary(condition, yesValue, noValue)
if condition, value = yesValue; else, value = noValue; end
end

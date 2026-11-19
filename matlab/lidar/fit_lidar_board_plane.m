function [center, info] = fit_lidar_board_plane(points, opts)
%FIT_LIDAR_BOARD_PLANE Recover a tilted board center from a guided LiDAR ROI.
%
% points are expressed in the initial ADT7880 radar frame. The fitted plane
% normal is constrained to face radar-forward; tilt changes the recovered
% board center only and is never interpreted as a sensor mounting rotation.

if nargin < 2, opts = struct(); end
opts = defaults(opts);
points = double(points);
points = points(all(isfinite(points), 2), 1:3);
if size(points, 1) < opts.minimumPoints
    error('Need at least %d finite board candidates; got %d.', ...
        opts.minimumPoints, size(points, 1));
end

rng(opts.randomSeed, 'twister');
cloud = pointCloud(points);
[~, ransacIndices] = pcfitplane(cloud, opts.ransacDistanceM, ...
    opts.expectedNormal, opts.maxAngularDistanceDeg);
if numel(ransacIndices) < opts.minimumPoints
    error('Constrained plane has only %d RANSAC inliers.', numel(ransacIndices));
end

normal = opts.expectedNormal; %#ok<NASGU>
inlierMask = false(size(points, 1), 1);
inlierMask(ransacIndices) = true;
for iteration = 1:opts.refineIterations
    fitPoints = points(inlierMask, :);
    anchor = median(fitPoints, 1);
    [~, ~, vectors] = svd(fitPoints-anchor, 0);
    normal = orient_normal(vectors(:,3).', opts.expectedNormal);
    signedDistance = (points-anchor) * normal.';
    residual = signedDistance(inlierMask);
    robustScale = 1.4826 * median(abs(residual-median(residual)));
    threshold = max(opts.minimumRefineDistanceM, opts.madScale*robustScale);
    threshold = min(threshold, opts.maximumRefineDistanceM);
    inlierMask = abs(signedDistance-median(residual)) <= threshold;
end

fitPoints = points(inlierMask, :);
if size(fitPoints, 1) < opts.minimumPoints
    error('Refined plane has only %d inliers.', size(fitPoints, 1));
end
anchor = median(fitPoints, 1);
[~, ~, vectors] = svd(fitPoints-anchor, 0);
normal = orient_normal(vectors(:,3).', opts.expectedNormal);
signedDistance = (fitPoints-anchor) * normal.';
anchor = anchor + median(signedDistance)*normal;

axisU = opts.horizontalAxis - dot(opts.horizontalAxis, normal)*normal;
axisU = axisU / norm(axisU);
axisV = cross(normal, axisU);
if dot(axisV, opts.verticalAxis) < 0, axisV = -axisV; end
axisV = axisV / norm(axisV);
coordinatesU = (fitPoints-anchor) * axisU.';
coordinatesV = (fitPoints-anchor) * axisV.';
boundsU = local_percentile(coordinatesU, opts.extentPercentiles);
boundsV = local_percentile(coordinatesV, opts.extentPercentiles);
center = anchor + mean(boundsU)*axisU + mean(boundsV)*axisV;

distance = (fitPoints-center) * normal.';
tiltDeg = acosd(max(-1, min(1, dot(normal, opts.expectedNormal) / ...
    norm(opts.expectedNormal))));
info = struct('normal', normal, 'tiltDeg', tiltDeg, ...
    'thicknessMadM', 1.4826*median(abs(distance-median(distance))), ...
    'thicknessP95M', local_percentile(abs(distance), 95), ...
    'inlierCount', size(fitPoints, 1), 'candidateCount', size(points, 1), ...
    'inlierMask', inlierMask, 'axisU', axisU, 'axisV', axisV, ...
    'boundsU', boundsU, 'boundsV', boundsV, ...
    'extentU', diff(boundsU), 'extentV', diff(boundsV), ...
    'planePoints', fitPoints);
end

function normal = orient_normal(normal, expected)
normal = normal / norm(normal);
if dot(normal, expected) < 0, normal = -normal; end
end

function opts = defaults(opts)
values = struct('expectedNormal',[0 1 0],'horizontalAxis',[1 0 0], ...
    'verticalAxis',[0 0 1],'maxAngularDistanceDeg',20, ...
    'ransacDistanceM',.012,'minimumRefineDistanceM',.006, ...
    'maximumRefineDistanceM',.018,'madScale',2.8,'refineIterations',5, ...
    'extentPercentiles',[3 97],'minimumPoints',25,'randomSeed',70);
names = fieldnames(values);
for index = 1:numel(names)
    if ~isfield(opts,names{index}) || isempty(opts.(names{index}))
        opts.(names{index}) = values.(names{index});
    end
end
opts.expectedNormal = reshape(double(opts.expectedNormal),1,3);
opts.expectedNormal = opts.expectedNormal/norm(opts.expectedNormal);
opts.horizontalAxis = reshape(double(opts.horizontalAxis),1,3);
opts.verticalAxis = reshape(double(opts.verticalAxis),1,3);
end

function value = local_percentile(values, percentages)
values = sort(values(isfinite(values)));
if isempty(values), value = nan(size(percentages)); return; end
value = arrayfun(@(percentage) interpolate(values, percentage), percentages);
end

function value = interpolate(values, percentage)
position = 1 + (numel(values)-1)*percentage/100;
lower = floor(position); upper = ceil(position);
if lower == upper
    value = values(lower);
else
    value = values(lower) + (position-lower)*(values(upper)-values(lower));
end
end

function solution = solve_extrinsics_candidates(cameraCenters, radarCandidates, opts)
%SOLVE_EXTRINSICS_CANDIDATES 共同中心候选的受约束 R/t/测距偏差联合求解。
%
% cameraCenters: N x 3，相机坐标 X右/Y下/Z前（米）
% radarCandidates{i}: struct 数组，至少含 center(1x3)，可含 confidence

if nargin < 3, opts = struct(); end
o = defaults(opts);
validateattributes(cameraCenters, {'numeric'}, {'ncols', 3, 'finite'});
if numel(radarCandidates) ~= size(cameraCenters, 1)
    error('cameraCenters 与 radarCandidates 组数不一致。');
end

valid = all(isfinite(cameraCenters), 2) & cellfun(@(x) ~isempty(x), radarCandidates(:));
if nnz(valid) < 3, error('至少需要 3 组有效共同中心。'); end
cameraCenters = cameraCenters(valid, :);
radarCandidates = radarCandidates(valid);

lower = [-o.angleLimitDeg*[1 1 1], o.tLower, o.biasBounds(1)];
upper = [ o.angleLimitDeg*[1 1 1], o.tUpper, o.biasBounds(2)];
start = [0 0 0, o.tPrior, o.biasPriorM];
rng(o.randomSeed, 'twister');

best = struct('objective', inf);
starts = zeros(o.multiStarts, 7);
starts(1, :) = start;
for k = 2:o.multiStarts
    starts(k, 1:3) = (2*rand(1,3)-1) * min(o.angleLimitDeg, 7);
    starts(k, 4:6) = o.tPrior + (2*rand(1,3)-1).*[0.035 0.035 0.02];
    starts(k, 7) = o.biasPriorM + (2*rand-1)*0.06;
end
starts = min(max(starts, lower + 1e-5), upper - 1e-5);
options = optimset('Display', 'off', 'MaxIter', o.maxIter, ...
    'MaxFunEvals', o.maxFunEvals, 'TolX', 2e-5, 'TolFun', 1e-7);
for k = 1:size(starts, 1)
    q0 = inverse_logistic((starts(k,:) - lower)./(upper-lower));
    objective = @(q) loss_value(decode(q, lower, upper), cameraCenters, radarCandidates, o);
    [q, value] = fminsearch(objective, q0, options);
    if value < best.objective
        best.objective = value;
        best.params = decode(q, lower, upper);
    end
end

[~, selected, residuals, predicted, corrected] = loss_value(best.params, cameraCenters, radarCandidates, o);
solution = pack(best.params, best.objective, selected, residuals, predicted, corrected, lower, upper, o);
solution.validMask = valid;
end

function [loss, selected, residuals, predicted, corrected] = loss_value(p, camera, candidates, o)
R = rotation_xyz_deg(p(1:3)) * o.R0;
t = p(4:6);
bias = p(7);
predicted = (R * camera.').'+t;
n = size(camera, 1);
selected = zeros(n, 1); residuals = zeros(n, 3); corrected = zeros(n, 3);
dataLoss = 0;
for i = 1:n
    cand = candidates{i};
    centers = vertcat(cand.center);
    q = radial_correct(centers, bias);
    dist = vecnorm(q - predicted(i,:), 2, 2);
    conf = ones(size(dist));
    if isfield(cand, 'confidence'), conf = reshape([cand.confidence], [], 1); end
    conf = min(max(conf, 0), 1);
    cost = huber(dist, o.huberM) + o.candidatePenaltyM*(1-conf);
    [bestCost, j] = min(cost);
    selected(i) = j;
    residuals(i,:) = q(j,:) - predicted(i,:);
    corrected(i,:) = q(j,:);
    dataLoss = dataLoss + bestCost;
end
dataLoss = dataLoss / n;
anglePrior = sum((p(1:3)/o.anglePriorSigmaDeg).^2);
tPrior = sum(((t-o.tPrior)./o.tPriorSigma).^2);
biasPrior = ((bias-o.biasPriorM)/o.biasPriorSigmaM)^2;
loss = dataLoss + o.priorWeight*(anglePrior+tPrior+biasPrior);
end

function solution = pack(p, objective, selected, residuals, predicted, corrected, lower, upper, o)
Rdelta = rotation_xyz_deg(p(1:3));
solution = struct('anglesDegRadarXYZ', p(1:3), 'Rdelta', Rdelta, ...
    'R', Rdelta*o.R0, 't', p(4:6), 'rangeBiasM', p(7), ...
    'selectedCandidate', selected, 'residuals', residuals, ...
    'errorNormM', vecnorm(residuals,2,2), 'predictedRadar', predicted, ...
    'correctedRadar', corrected, 'objective', objective, ...
    'lowerBounds', lower, 'upperBounds', upper);
span = upper-lower;
solution.boundMarginFraction = min((p-lower)./span, (upper-p)./span);
end

function points = radial_correct(points, bias)
r = vecnorm(points, 2, 2);
points = points .* (max(r-bias, 0.1)./max(r, eps));
end

function value = huber(x, delta)
value = 0.5*x.^2;
far = x > delta;
value(far) = delta*(x(far)-0.5*delta);
end

function R = rotation_xyz_deg(a)
rx = deg2rad(a(1)); ry = deg2rad(a(2)); rz = deg2rad(a(3));
Rx = [1 0 0; 0 cos(rx) -sin(rx); 0 sin(rx) cos(rx)];
Ry = [cos(ry) 0 sin(ry); 0 1 0; -sin(ry) 0 cos(ry)];
Rz = [cos(rz) -sin(rz) 0; sin(rz) cos(rz) 0; 0 0 1];
R = Rz*Ry*Rx;
end

function p = decode(q, lower, upper)
s = 1./(1+exp(-q));
p = lower+(upper-lower).*s;
end

function q = inverse_logistic(s)
s = min(max(s,1e-7),1-1e-7);
q = log(s./(1-s));
end

function o = defaults(o)
d = struct('R0', [1 0 0;0 0 1;0 -1 0], 'angleLimitDeg', 15, ...
    'tLower', [-0.10 -0.10 0.05], 'tUpper', [0.10 0.10 0.15], ...
    'tPrior', [0 0 0.10], 'tPriorSigma', [0.06 0.06 0.025], ...
    'biasBounds', [0 0.30], 'biasPriorM', 0.16, 'biasPriorSigmaM', 0.10, ...
    'anglePriorSigmaDeg', 10, 'priorWeight', 0.0001, ...
    'candidatePenaltyM', 0.015, 'huberM', 0.12, 'multiStarts', 12, ...
    'maxIter', 220, 'maxFunEvals', 500, 'randomSeed', 1104);
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(o,f{i}), o.(f{i})=d.(f{i}); end
end
end

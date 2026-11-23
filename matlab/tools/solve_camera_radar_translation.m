function solution = solve_camera_radar_translation(cameraCenters, radarCenters, rotation, initialTranslation, opts)
%SOLVE_CAMERA_RADAR_TRANSLATION Fit bounded translation with group holdouts.
if nargin < 5, opts = struct(); end
o = defaults(opts);
cameraCenters = double(cameraCenters); radarCenters = double(radarCenters);
initialTranslation = reshape(double(initialTranslation), 1, 3);
if size(cameraCenters,2) ~= 3 || size(radarCenters,2) ~= 3 || ...
        ~isequal(size(cameraCenters),size(radarCenters)) || ...
        any(~isfinite(cameraCenters(:))) || any(~isfinite(radarCenters(:)))
    error('cameraCenters 和 radarCenters 必须是同尺寸有限 N×3 数组。');
end
validateattributes(rotation, {'numeric'}, {'size', [3 3], 'finite'});
if size(cameraCenters,1) < 3, error('至少需要三组中心对应。'); end

rotated = (rotation * cameraCenters.').';
lower = initialTranslation - o.translationHalfRangeM;
upper = initialTranslation + o.translationHalfRangeM;
translation = bounded_median(radarCenters - rotated, lower, upper);
predicted = rotated + translation;
baseline = vecnorm(rotated + initialTranslation - radarCenters, 2, 2);
fitError = vecnorm(predicted - radarCenters, 2, 2);

n = size(cameraCenters,1); looError = nan(n,1); foldTranslation = nan(n,3);
for holdout = 1:n
    fit = true(n,1); fit(holdout) = false;
    foldTranslation(holdout,:) = bounded_median( ...
        radarCenters(fit,:) - rotated(fit,:), lower, upper);
    looError(holdout) = norm(rotated(holdout,:) + foldTranslation(holdout,:) - radarCenters(holdout,:));
end

[diagnosticTranslation, rangeBias] = fit_with_range_bias(rotated, radarCenters, ...
    translation, lower, upper, o.maxRangeBiasM);
diagnosticShift = norm(diagnosticTranslation - translation);
lowerMargin = (translation-lower) ./ (upper-lower);
upperMargin = (upper-translation) ./ (upper-lower);
margin = min([lowerMargin upperMargin]);
improvement = 1 - median(fitError) / max(median(baseline), eps);
passed = median(looError) <= o.maxLooMedianM && percentile(looError,90) <= o.maxLooP90M && ...
    improvement >= o.minimumImprovement && margin >= o.minimumBoundMarginFraction && ...
    diagnosticShift <= o.maxDiagnosticTranslationShiftM;

bootstrapTranslations = bootstrap_fit(radarCenters-rotated, lower, upper, o);
solution = struct('translation', translation, 'baselineErrorM', baseline, ...
    'fitErrorM', fitError, 'leaveOneOutErrorM', looError, ...
    'foldTranslationM', foldTranslation, 'improvementRatio', improvement, ...
    'boundMarginFraction', margin, 'diagnosticTranslationM', diagnosticTranslation, ...
    'diagnosticRangeBiasM', rangeBias, 'diagnosticTranslationShiftM', diagnosticShift, ...
    'bootstrapCI95M', percentile(bootstrapTranslations,[2.5 97.5]), ...
    'status', pass_text(passed), 'applicable', passed, 'options', o);
end

function value = bounded_median(samples, lower, upper)
value = min(max(median(samples,1), lower), upper);
end

function [translation, bias] = fit_with_range_bias(rotated, radar, initial, lower, upper, maximumBias)
encode = @(value,lo,hi) log(max(min((value-lo)./(hi-lo),1-1e-6),1e-6) ./ ...
    (1-max(min((value-lo)./(hi-lo),1-1e-6),1e-6)));
decode = @(value,lo,hi) lo + (hi-lo)./(1+exp(-value));
q0 = [encode(initial,lower,upper), encode(maximumBias/4,0,maximumBias)];
objective = @(q) robust_loss(decode(q(1:3),lower,upper), ...
    decode(q(4),0,maximumBias), rotated, radar);
q = fminsearch(objective, q0, optimset('Display','off','MaxIter',250,'MaxFunEvals',600));
translation = decode(q(1:3),lower,upper); bias = decode(q(4),0,maximumBias);
end

function loss = robust_loss(translation, bias, rotated, radar)
range = vecnorm(radar,2,2);
corrected = radar .* max(range-bias,0.1) ./ max(range,eps);
residual = vecnorm(rotated+translation-corrected,2,2);
scale = 0.15; loss = mean(scale^2*(sqrt(1+(residual/scale).^2)-1));
end

function samples = bootstrap_fit(delta, lower, upper, o)
rng(o.randomSeed,'twister'); n = size(delta,1);
samples = zeros(o.bootstrapCount,3);
for index = 1:o.bootstrapCount
    pick = randi(n,n,1); samples(index,:) = bounded_median(delta(pick,:),lower,upper);
end
end

function value = percentile(values, p)
values = sort(values,1); n = size(values,1); p = p(:)/100;
position = 1 + (n-1)*p; low = floor(position); high = ceil(position);
weight = position-low;
value = values(low,:).*(1-weight) + values(high,:).*weight;
if isscalar(value), value = double(value); end
end

function text = pass_text(value)
if value, text='PASS'; else, text='FAIL'; end
end

function o = defaults(o)
d = struct('translationHalfRangeM',[0.05 0.05 0.02], 'maxRangeBiasM',0.30, ...
    'maxLooMedianM',0.15, 'maxLooP90M',0.30, 'minimumImprovement',0.20, ...
    'minimumBoundMarginFraction',0.02, 'maxDiagnosticTranslationShiftM',0.02, ...
    'bootstrapCount',500, 'randomSeed',1122);
fields=fieldnames(d);
for index=1:numel(fields)
    if ~isfield(o,fields{index})
        o.(fields{index})=d.(fields{index});
    end
end
end

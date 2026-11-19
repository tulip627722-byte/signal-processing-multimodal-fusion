function report = solve_lidar_radar_extrinsics(correspondences, opts)
%SOLVE_LIDAR_RADAR_EXTRINSICS Robust rigid fit from MID-70 to ADT7880 radar.
% Columns: lidar_x/y/z and radar_x/y/z in metres; optional weight and split.

if nargin < 2, opts = struct(); end
opts = defaults(opts);
if ischar(correspondences) || isstring(correspondences)
    inputPath = char(correspondences);
    correspondences = readtable(inputPath, 'TextType', 'string');
else
    inputPath = '';
end
required = {'lidar_x','lidar_y','lidar_z','radar_x','radar_y','radar_z'};
if ~all(ismember(required, correspondences.Properties.VariableNames))
    error('Correspondence table must contain: %s', strjoin(required, ', '));
end
lidarPoints = double(correspondences{:,required(1:3)}); radarPoints = double(correspondences{:,required(4:6)});
if ismember('weight', correspondences.Properties.VariableNames)
    baseWeights = double(correspondences.weight);
else
    baseWeights = ones(height(correspondences),1);
end
if ismember('split', correspondences.Properties.VariableNames)
    split = lower(strtrim(string(correspondences.split)));
    fitMask = split=="fit" | split=="train" | split=="training";
    holdoutMask = split=="holdout" | split=="validation" | split=="test";
    if ~any(fitMask), fitMask = ~holdoutMask; end
else
    fitMask = true(height(correspondences),1);
    holdoutMask = false(height(correspondences),1);
end
finiteMask = all(isfinite(lidarPoints),2) & all(isfinite(radarPoints),2) & isfinite(baseWeights) & baseWeights>0;
fitMask = fitMask & finiteMask; holdoutMask = holdoutMask & finiteMask;
if nnz(fitMask)<opts.minSamples, error('Need at least %d fit correspondences; got %d.',opts.minSamples,nnz(fitMask)); end
centered = lidarPoints(fitMask,:)-mean(lidarPoints(fitMask,:),1); singularValues=svd(centered,'econ');
baseline=max(vecnorm(centered,2,2))*2;
if baseline<opts.minBaselineM || numel(singularValues)<2 || singularValues(2)<opts.minSecondarySpreadM
    error(['Calibration poses do not constrain a 3-D transform. Move the reflector across ' ...
        'left/right, up/down and near/far; baseline %.3f m, secondary spread %.3f m.'], ...
        baseline,singularValues(min(2,numel(singularValues))));
end
fitLidar=lidarPoints(fitMask,:); fitRadar=radarPoints(fitMask,:); weights=baseWeights(fitMask);
for iteration=1:opts.robustIterations
    [rotation,translation]=weighted_rigid_fit(fitLidar,fitRadar,weights);
    residual=vecnorm((rotation*fitLidar.').'+translation-fitRadar,2,2);
    robustScale=max(1.4826*median(abs(residual-median(residual))),opts.minimumScaleM);
    weights=baseWeights(fitMask).*min(1,opts.huberScale*robustScale./max(residual,eps));
end
predicted=(rotation*lidarPoints.').'+translation; residualAll=vecnorm(predicted-radarPoints,2,2);
fitMetrics=metrics(residualAll(fitMask)); holdoutMetrics=metrics(residualAll(holdoutMask)); quality=fitMetrics;
if any(holdoutMask), quality=holdoutMetrics; end
passed=quality.medianM<=opts.acceptanceMedianM && quality.p95M<=opts.acceptanceP95M;
extrinsics=struct('version',1,'sourceFrame','livox_sensor','targetFrame','adt7880_radar','units','m', ...
    'lidarAxes','X forward, Y left, Z up','radarAxes','X right, Y forward, Z up', ...
    'lidarToRadarR',rotation,'lidarOriginInRadarM',translation,'status',pass_text(passed), ...
    'applicable',passed,'fitSamples',nnz(fitMask),'holdoutSamples',nnz(holdoutMask), ...
    'fitMedianErrorM',fitMetrics.medianM,'fitP95ErrorM',fitMetrics.p95M, ...
    'holdoutMedianErrorM',holdoutMetrics.medianM,'holdoutP95ErrorM',holdoutMetrics.p95M, ...
    'source',inputPath,'note','p_radar = lidarToRadarR*p_lidar + lidarOriginInRadarM; validate on independent poses.');
report=struct('extrinsics',extrinsics,'fitMetrics',fitMetrics,'holdoutMetrics',holdoutMetrics, ...
    'residualM',residualAll,'predictedRadarPoints',predicted,'fitMask',fitMask,'holdoutMask',holdoutMask,'pass',passed);
if ~isempty(opts.outputFile)
    outputDirectory=fileparts(opts.outputFile); if ~isempty(outputDirectory)&&~isfolder(outputDirectory),mkdir(outputDirectory);end
    fileId=fopen(opts.outputFile,'w','n','UTF-8'); if fileId<0,error('Cannot write %s',opts.outputFile);end
    cleanup=onCleanup(@()fclose(fileId)); fwrite(fileId,jsonencode(extrinsics,'PrettyPrint',true),'char');
end
fprintf('LiDAR -> radar: fit median %.1f mm, P95 %.1f mm',fitMetrics.medianM*1000,fitMetrics.p95M*1000);
if any(holdoutMask),fprintf('; holdout median %.1f mm, P95 %.1f mm',holdoutMetrics.medianM*1000,holdoutMetrics.p95M*1000);end
fprintf('; %s\n',pass_text(passed));
end

function [R,t]=weighted_rigid_fit(source,target,weights)
weights=weights(:)/sum(weights); sourceCenter=sum(source.*weights,1); targetCenter=sum(target.*weights,1);
sourceCentered=source-sourceCenter; targetCentered=target-targetCenter;
[U,~,V]=svd((sourceCentered.*weights).'*targetCentered); correction=eye(3); correction(3,3)=sign(det(V*U.'));
R=V*correction*U.'; t=targetCenter-(R*sourceCenter.').';
end
function result=metrics(values)
values=values(isfinite(values)); if isempty(values),result=struct('count',0,'medianM',NaN,'rmseM',NaN,'p95M',NaN,'maxM',NaN);return;end
sorted=sort(values);p95Index=max(1,min(numel(sorted),ceil(.95*numel(sorted))));
result=struct('count',numel(values),'medianM',median(values),'rmseM',sqrt(mean(values.^2)),'p95M',sorted(p95Index),'maxM',max(values));
end
function opts=defaults(opts)
values=struct('minSamples',6,'minBaselineM',.30,'minSecondarySpreadM',.05,'robustIterations',8, ...
    'minimumScaleM',.005,'huberScale',1.5,'acceptanceMedianM',.08,'acceptanceP95M',.15,'outputFile','');
names=fieldnames(values);for index=1:numel(names),if ~isfield(opts,names{index})||isempty(opts.(names{index})),opts.(names{index})=values.(names{index});end,end
end
function value=pass_text(passed)
if passed,value='PASS';else,value='FAIL';end
end

function analysis = analyze_board_plane_alignment(data, associations, opts)
%ANALYZE_BOARD_PLANE_ALIGNMENT Estimate board plane and radar normal errors.
geometry = [double(data.depth.radar); double(data.lidar.radar)];
if isempty(geometry) || ~isfield(associations, 'cluster') || isempty(associations.cluster)
    analysis = empty_analysis();
    return;
end
clusterPoints = zeros(0, 3);
for k = 1:numel(associations.cluster)
    item = associations.cluster(k);
    delta = geometry - item.radar_centroid;
    range = sqrt(sum(geometry.^2, 2));
    azimuth = atan2d(geometry(:,1), geometry(:,2));
    targetRange = norm(item.radar_centroid);
    targetAzimuth = atan2d(item.radar_centroid(1), item.radar_centroid(2));
    keep = abs(range-targetRange) <= opts.radarAssociationRangeToleranceM & ...
        abs(wrap_angle(azimuth-targetAzimuth)) <= opts.radarAssociationAzimuthDeg & ...
        sqrt(delta(:,1).^2 + delta(:,2).^2) <= 0.55;
    clusterPoints = [clusterPoints; geometry(keep,:)]; %#ok<AGROW>
end
known = load_known_board(data, opts);
if known.valid
    center = known.center;
    normal = known.normal;
    planeInlier = abs((geometry-center)*normal(:)) <= 0.08;
    fitPointCount = known.candidateCount;
else
    if size(clusterPoints,1) < 20
        analysis = empty_analysis();
        return;
    end
    fitPointCount = size(clusterPoints,1);
    [normal, center, ~] = fit_plane_ransac(clusterPoints, opts.boardPlaneRansacThresholdM, 300, opts.randomSeed);
    if normal(3) < 0, normal = -normal; end
    distances = abs((geometry-center)*normal(:));
    planeInlier = distances <= 0.08;
    if nnz(planeInlier) >= 20
        [normal, center] = fit_plane_svd(geometry(planeInlier,:));
        if normal(3) < 0, normal = -normal; end
    end
end
errors = struct('cluster_index',{},'normal_error_m',{},'tangent_error_m',{}, ...
    'signed_normal_error_m',{},'projected_point',{},'radar_centroid',{},'target_error_m',{},'is_board_candidate',{});
targetCenter = center;
for k = 1:numel(associations.cluster)
    item = associations.cluster(k); point = item.radar_centroid;
    signed = dot(point-center, normal); projected = point-signed*normal;
    tangent = norm((point-center)-signed*normal);
    targetError = norm(point-targetCenter);
    boardCandidate = abs(signed) <= opts.boardClusterNormalToleranceM && ...
        tangent <= hypot(opts.boardWidthM/2,opts.boardHeightM/2)+0.10;
    errors(end+1) = struct('cluster_index',item.cluster_index,'normal_error_m',abs(signed), ...
        'tangent_error_m',tangent,'signed_normal_error_m',signed, ...
        'projected_point',projected,'radar_centroid',point,'target_error_m',targetError,'is_board_candidate',boardCandidate); %#ok<AGROW>
end
normalErrors = [errors.normal_error_m];
analysis = struct('valid',known.valid || nnz(planeInlier)/size(geometry,1) >= 0.25,'source',known.source, ...
    'plane_center',center,'radar_target_center',targetCenter,'plane_normal',normal,'fit_point_count',fitPointCount, ...
    'errors',errors,'boardCandidateClusterIndices',[errors([errors.is_board_candidate]).cluster_index], ...
    'normal_median_m',median(normalErrors), ...
    'normal_p95_m',percentile_value(normalErrors,95),'inlier_ratio',nnz(planeInlier)/size(geometry,1));
end

function [normal, center, inlier] = fit_plane_ransac(points, threshold, iterations, seed)
rng(seed,'twister'); bestCount=0; bestInlier=false(size(points,1),1);
for i=1:iterations
    sample=randperm(size(points,1),3); a=points(sample(2),:)-points(sample(1),:); b=points(sample(3),:)-points(sample(1),:); n=cross(a,b); nn=norm(n); if nn<eps,continue;end
    n=n/nn; d=abs((points-points(sample(1),:))*n(:)); current=d<=threshold;
    if nnz(current)>bestCount,bestCount=nnz(current);bestInlier=current;end
end
if bestCount<3, inlier=true(size(points,1),1); else, inlier=bestInlier; end
[normal,center]=fit_plane_svd(points(inlier,:));
end

function [normal,center] = fit_plane_svd(points)
center=median(points,1);[~,~,v]=svd(points-center,0);normal=v(:,3).';normal=normal/norm(normal);
end

function value=percentile_value(values,p)
values=sort(values(:)); if isempty(values),value=NaN;return;end
index=1+(numel(values)-1)*p/100;lo=floor(index);hi=ceil(index);if lo==hi,value=values(lo);else,value=values(lo)+(index-lo)*(values(hi)-values(lo));end
end

function angle=wrap_angle(angle),angle=mod(angle+180,360)-180;end
function known=load_known_board(data, opts)
known=struct('valid',false,'center',[NaN NaN NaN],'radarTarget',[NaN NaN NaN],'normal',[NaN NaN NaN],'candidateCount',0,'source','ransac');
if ~isfield(data,'takeName') || ~isfield(opts,'knownBoardCorrespondenceFile') || ~isfile(opts.knownBoardCorrespondenceFile),return;end
try
    tableData=readtable(opts.knownBoardCorrespondenceFile,'TextType','string');
    match=strcmp(string(tableData.sample_id),string(data.takeName));
    if ~any(match),return;end
    row=find(match,1); known.center=[tableData.radar_x(row),tableData.radar_y(row),tableData.radar_z(row)];
    if all(ismember({'radar_validation_x','radar_validation_y','radar_validation_z'},tableData.Properties.VariableNames))
        known.radarTarget=[tableData.radar_validation_x(row),tableData.radar_validation_y(row),tableData.radar_validation_z(row)];
    else
        known.radarTarget=known.center;
    end
    known.normal=[tableData.plane_normal_x(row),tableData.plane_normal_y(row),tableData.plane_normal_z(row)];
    known.normal=known.normal/norm(known.normal); known.candidateCount=tableData.plane_candidates(row);
    known.source='known_checkerboard_plane'; known.valid=all(isfinite([known.center known.radarTarget known.normal]));
catch
end
end
function result=empty_analysis(),result=struct('valid',false,'source','none','plane_center',[NaN NaN NaN],'radar_target_center',[NaN NaN NaN],'plane_normal',[NaN NaN NaN],'fit_point_count',0,'geometry_inlier_count',0,'errors',struct([]),'boardCandidateClusterIndices',[],'normal_median_m',NaN,'normal_p95_m',NaN,'inlier_ratio',NaN);end

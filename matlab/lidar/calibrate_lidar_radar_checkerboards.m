function report = calibrate_lidar_radar_checkerboards(dataRoot, resultsRoot, outputDir, opts)
%CALIBRATE_LIDAR_RADAR_CHECKERBOARDS Calibrate MID-70 to ADT7880 from board centers.
%
% The known camera-to-radar extrinsics put depth checkerboard centers in the
% ADT7880 frame. A camera-guided ROI then extracts the same physical plane
% from a 1000 ms MID-70 accumulation. The saved transform maps native Livox
% coordinates (X forward/Y left/Z up) to radar coordinates
% (X right/Y forward/Z up).

projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
if nargin < 1 || isempty(dataRoot), dataRoot = 'E:\leida\11-18'; end
if nargin < 2 || isempty(resultsRoot)
    resultsRoot = fullfile(projectRoot, 'results', 'takes');
end
if nargin < 3 || isempty(outputDir)
    outputDir = fullfile(fileparts(resultsRoot), 'lidar_radar_calibration_20261118');
end
if nargin < 4, opts = struct(); end
opts = defaults(opts);
validate_options(opts);
opts.initialTranslation=reshape(double(opts.initialTranslation),1,3);
if ~isempty(opts.cameraOriginInRadarM)
    opts.cameraOriginInRadarM=reshape(double(opts.cameraOriginInRadarM),1,3);
end
if ~isfolder(outputDir), mkdir(outputDir); end

takeDirs = select_take_dirs(dataRoot, opts.takeNames);
if numel(takeDirs) < 3
    error('At least 3 checkerboard takes are required; found %d in %s.', numel(takeDirs), dataRoot);
end

cameraExtrinsicsPath = opts.cameraExtrinsicsFile;
if isempty(cameraExtrinsicsPath)
    cameraExtrinsicsPath = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
        'radar_camera_extrinsics.json');
end
cameraExtrinsics = load_radar_camera_extrinsics(cameraExtrinsicsPath);
if ~isempty(opts.cameraToRadarR)
    cameraExtrinsics.cameraToRadarR = opts.cameraToRadarR;
end
if ~isempty(opts.cameraOriginInRadarM)
    cameraExtrinsics.cameraOriginInRadarM = opts.cameraOriginInRadarM;
end
samples = repmat(empty_sample(), numel(takeDirs), 1);
for takeIndex = 1:numel(takeDirs)
    takeDir = fullfile(takeDirs(takeIndex).folder, takeDirs(takeIndex).name);
    resultPath = fullfile(resultsRoot, [takeDirs(takeIndex).name '_res.mat']);
    if ~isfile(resultPath)
        if ~opts.autoComputeMissingResults
            error('Missing processed result: %s', resultPath);
        end
        fprintf('Missing processed result; computing %s ...\n', takeDirs(takeIndex).name);
        computeOptions = struct('cameraToRadarR',cameraExtrinsics.cameraToRadarR, ...
            'cameraOriginInRadarM',cameraExtrinsics.cameraOriginInRadarM, ...
            'extrinsicsFile',cameraExtrinsicsPath);
        [resultPath, ~] = compute_take(takeDir, computeOptions);
    end
    samples(takeIndex) = extract_sample(takeDir, resultPath, opts, cameraExtrinsics);
    fprintf('%s: depth [%+.3f %.3f %+.3f], LiDAR initial [%+.3f %.3f %+.3f], %d plane points\n', ...
        samples(takeIndex).name, samples(takeIndex).depthCenterRadar, ...
        samples(takeIndex).lidarCenterInitialRadar, samples(takeIndex).lidarPlanePointCount);
end

lidarCenters = vertcat(samples.lidarCenterNative);
legacyLidarCenters = vertcat(samples.lidarCenterThinSliceNative);
radarReference = vertcat(samples.depthCenterRadar);
if strcmpi(opts.solveMode, 'translation_only')
    rotation = opts.initialRotation;
    translation = median(radarReference-(rotation*lidarCenters.').',1);
else
    [rotation, translation] = rigid_fit(lidarCenters, radarReference);
end
before = (opts.initialRotation * lidarCenters.').'+opts.initialTranslation;
after = (rotation * lidarCenters.').'+translation;
beforeError = vecnorm(before-radarReference, 2, 2);
afterError = vecnorm(after-radarReference, 2, 2);
[legacyTranslation, legacyAfterError, legacyLooError] = translation_metrics( ...
    legacyLidarCenters, radarReference, opts.initialRotation);

looError = nan(numel(samples), 1);
for holdout = 1:numel(samples)
    fitMask = true(numel(samples), 1); fitMask(holdout) = false;
    if strcmpi(opts.solveMode, 'translation_only')
        foldRotation = opts.initialRotation;
        foldTranslation = median(radarReference(fitMask,:)- ...
            (foldRotation*lidarCenters(fitMask,:).').',1);
    else
        [foldRotation, foldTranslation] = rigid_fit(lidarCenters(fitMask,:), radarReference(fitMask,:));
    end
    predicted = (foldRotation * lidarCenters(holdout,:).').'+foldTranslation;
    looError(holdout) = norm(predicted-radarReference(holdout,:));
end

for sampleIndex = 1:numel(samples)
    samples(sampleIndex).lidarCenterBeforeRadar = before(sampleIndex,:);
    samples(sampleIndex).lidarCenterAfterRadar = after(sampleIndex,:);
    samples(sampleIndex).beforeErrorM = beforeError(sampleIndex);
    samples(sampleIndex).afterErrorM = afterError(sampleIndex);
    samples(sampleIndex).leaveOneOutErrorM = looError(sampleIndex);
end

rotationDelta = rotation * opts.initialRotation.';
rotationCorrectionDeg = rotation_angle_deg(rotationDelta);
qualityImproved = median(afterError) < median(legacyAfterError) && ...
    median(looError) < median(legacyLooError);
passed = median(afterError) <= opts.maxFitMedianM && max(afterError) <= opts.maxFitMaxM && ...
    median(looError) <= opts.maxLooMedianM && qualityImproved;
extrinsics = struct('version', 1, 'sourceFrame', 'livox_sensor', ...
    'targetFrame', 'adt7880_radar', 'units', 'm', ...
    'lidarAxes', 'X forward, Y left, Z up', ...
    'radarAxes', 'X right, Y forward, Z up', ...
    'lidarToRadarR', rotation, 'lidarOriginInRadarM', translation, ...
    'initialLidarToRadarR', opts.initialRotation, ...
    'initialLidarOriginInRadarM', opts.initialTranslation, ...
    'solveMode', opts.solveMode, ...
    'rotationCorrectionFromMeasuredDeg', rotationCorrectionDeg, ...
    'status', pass_text(passed), 'applicable', passed, ...
    'fitSamples', numel(samples), 'fitMedianErrorM', median(afterError), ...
    'fitP95ErrorM', percentile(afterError, 95), 'fitMaxErrorM', max(afterError), ...
    'leaveOneOutMedianErrorM', median(looError), ...
    'leaveOneOutP95ErrorM', percentile(looError, 95), ...
    'thinSliceFitMedianErrorM', median(legacyAfterError), ...
    'thinSliceLeaveOneOutMedianErrorM', median(legacyLooError), ...
    'tiltCompensationImproved', qualityImproved, ...
    'cameraExtrinsicsFile', cameraExtrinsicsPath, ...
    'cameraToRadarR', cameraExtrinsics.cameraToRadarR, ...
    'cameraOriginInRadarM', cameraExtrinsics.cameraOriginInRadarM, ...
    'source', dataRoot, ...
    'note', ['Depth checkerboard centers were transformed by the configured camera-to-radar extrinsics. ' ...
    'MID-70 centers use camera-guided 1000 ms plane extraction. Rotation is fixed from the known parallel mounting axes; ' ...
    'checkerboard centers robustly fit translation. Validate visually before promotion.']);

correspondences = sample_table(samples);
csvPath = fullfile(outputDir, 'lidar_radar_correspondences.csv');
writetable(correspondences, csvPath);
jsonPath = fullfile(outputDir, 'lidar_radar_extrinsics_candidate.json');
write_json(jsonPath, extrinsics);
comparisonPath = fullfile(outputDir, 'checkerboard_centers_before_after.png');
plot_real_image_comparison(samples, cameraExtrinsics, comparisonPath);
summaryPath = fullfile(outputDir, 'calibration_summary.png');
plot_summary(samples, rotation, translation, opts, summaryPath);
tiltPath = fullfile(outputDir, 'tilt_compensation_before_after.png');
plot_tilt_compensation(samples, legacyTranslation, translation, opts, tiltPath);

report = struct('samples', samples, 'correspondences', correspondences, ...
    'extrinsics', extrinsics, 'beforeErrorM', beforeError, 'afterErrorM', afterError, ...
    'leaveOneOutErrorM', looError, 'jsonPath', jsonPath, 'csvPath', csvPath, ...
    'legacyAfterErrorM', legacyAfterError, 'legacyLeaveOneOutErrorM', legacyLooError, ...
    'comparisonPath', comparisonPath, 'summaryPath', summaryPath, 'tiltPath', tiltPath);
save(fullfile(outputDir, 'calibration_report.mat'), 'report', '-v7.3');

fprintf('\nLiDAR -> radar calibration: %s\n', extrinsics.status);
fprintf('  solve mode          = %s\n', opts.solveMode);
fprintf('  measured initial t = [%+.4f %+.4f %+.4f] m\n', opts.initialTranslation);
fprintf('  fitted t           = [%+.4f %+.4f %+.4f] m\n', translation);
fprintf('  correction angle   = %.3f deg\n', rotationCorrectionDeg);
fprintf('  center error before median %.1f mm, after %.1f mm, max %.1f mm\n', ...
    median(beforeError)*1000, median(afterError)*1000, max(afterError)*1000);
fprintf('  leave-one-out median %.1f mm, P95 %.1f mm\n', ...
    median(looError)*1000, percentile(looError,95)*1000);
fprintf('  thin-slice baseline fit %.1f mm, leave-one-out %.1f mm; improved=%d\n', ...
    median(legacyAfterError)*1000, median(legacyLooError)*1000, qualityImproved);
fprintf('  real-image comparison: %s\n', comparisonPath);
fprintf('  candidate JSON: %s\n', jsonPath);
end

function sample = extract_sample(takeDir, resultPath, opts, cameraExtrinsics)
data = load(resultPath, 'radarT','pts','labels','depthPts','depthUV','colorFrame','metaOut');
frameIndex = max(1, round(numel(data.radarT)/2));
colorFrame = double(data.colorFrame(frameIndex));
imagePath = fullfile(takeDir, 'color', sprintf('%06d.png', colorFrame));
image = imread(imagePath);
warningState = warning('off', 'all');
[corners, boardSize] = detectCheckerboardPoints(image);
warning(warningState);
if isempty(corners), error('Checkerboard not found in %s', imagePath); end

uv = double(data.depthUV{frameIndex}) + 1;
depthPoints = double(data.depthPts{frameIndex});
depthPoints = remap_depth_points(depthPoints, data.metaOut, cameraExtrinsics);
hull = convhull(corners(:,1), corners(:,2));
inside = inpolygon(uv(:,1), uv(:,2), corners(hull,1), corners(hull,2));
boardDepth = depthPoints(inside,:);
if size(boardDepth,1) < opts.minimumDepthPoints
    centerUv = mean(corners,1); span = max(corners,[],1)-min(corners,[],1);
    inside = abs(uv(:,1)-centerUv(1)) <= .6*span(1) & ...
        abs(uv(:,2)-centerUv(2)) <= .6*span(2);
    boardDepth = depthPoints(inside,:);
end
rangeCenter = median(boardDepth(:,2));
boardDepth = boardDepth(abs(boardDepth(:,2)-rangeCenter) <= opts.depthPlaneToleranceM,:);
depthCenter = median(boardDepth,1);
depthBoundsX = percentile(boardDepth(:,1),[3 97]);
depthBoundsZ = percentile(boardDepth(:,3),[3 97]);

lidarRows = readtable(fullfile(takeDir,'lidar_frames.csv'),'TextType','string');
lidarTime = (double(lidarRows.first_t_rel_ms)+double(lidarRows.last_t_rel_ms))/2;
selectedRows = find(abs(lidarTime-double(data.radarT(frameIndex))) <= opts.lidarWindowMs/2);
lidarNative = zeros(0,3); lidarReflectivity = zeros(0,1);
for rowIndex = selectedRows(:).'
    filePath = fullfile(takeDir,strrep(char(lidarRows.file(rowIndex)),'/',filesep));
    batch = read_livox_bin(filePath);
    lidarNative = [lidarNative;double(batch.points)]; %#ok<AGROW>
    lidarReflectivity = [lidarReflectivity;double(batch.reflectivity)]; %#ok<AGROW>
end
lidarInitial = (opts.initialRotation*lidarNative.').'+opts.initialTranslation;
roi = lidarInitial(:,1) >= depthBoundsX(1)-opts.boardPaddingXM & ...
    lidarInitial(:,1) <= depthBoundsX(2)+opts.boardPaddingXM & ...
    lidarInitial(:,3) >= depthBoundsZ(1)-opts.boardPaddingZM & ...
    lidarInitial(:,3) <= depthBoundsZ(2)+opts.boardPaddingZM & ...
    abs(lidarInitial(:,2)-depthCenter(2)) <= opts.rangeSearchM;
candidateInitial = lidarInitial(roi,:);
candidateNative = lidarNative(roi,:);
candidateReflectivity = lidarReflectivity(roi);
if size(candidateInitial,1) < opts.minimumLidarPoints
    error('Too few LiDAR board candidates in %s: %d', takeDir, size(candidateInitial,1));
end
planeRange = dominant_range(candidateInitial(:,2), depthCenter(2), opts);
thinSlice = abs(candidateInitial(:,2)-planeRange) <= opts.lidarPlaneToleranceM;
thinSliceInitial = candidateInitial(thinSlice,:);
thinSliceNative = candidateNative(thinSlice,:);
if size(thinSliceNative,1) < opts.minimumLidarPoints
    error('Too few LiDAR thin-slice points in %s: %d', takeDir, size(thinSliceNative,1));
end
thinSliceCenterInitial = median(thinSliceInitial,1);
thinSliceCenterNative = (opts.initialRotation.' * ...
    (thinSliceCenterInitial-opts.initialTranslation).').';
planeOptions = struct('maxAngularDistanceDeg',opts.maxBoardTiltDeg, ...
    'ransacDistanceM',opts.lidarPlaneToleranceM,'minimumPoints',opts.minimumLidarPoints, ...
    'extentPercentiles',opts.planeExtentPercentiles,'randomSeed',opts.randomSeed);
[lidarCenterInitial, planeInfo] = fit_lidar_board_plane(candidateInitial, planeOptions);
lidarCenterNative = (opts.initialRotation.' * ...
    (lidarCenterInitial-opts.initialTranslation).').';
planeReflectivity = candidateReflectivity(planeInfo.inlierMask);

radarCenter = extract_radar_center(data.pts, data.labels, depthCenter, opts);
[~, name] = fileparts(takeDir);
sample = empty_sample();
sample.name = name; sample.takeDir = takeDir; sample.resultPath = resultPath;
sample.frameIndex = frameIndex; sample.radarTimeMs = double(data.radarT(frameIndex));
sample.colorFrame = colorFrame; sample.imagePath = imagePath; sample.image = image;
sample.checkerCorners = corners; sample.boardSize = boardSize;
sample.depthCenterRadar = depthCenter; sample.depthBoardPoints = boardDepth;
sample.lidarCenterNative = lidarCenterNative;
sample.lidarCenterInitialRadar = lidarCenterInitial;
sample.lidarCenterThinSliceNative = thinSliceCenterNative;
sample.lidarCenterThinSliceInitialRadar = thinSliceCenterInitial;
sample.lidarTiltCompensationShiftM = lidarCenterInitial-thinSliceCenterInitial;
sample.lidarPlaneNormalInitialRadar = planeInfo.normal;
sample.lidarPlaneTiltDeg = planeInfo.tiltDeg;
sample.lidarPlaneThicknessMadM = planeInfo.thicknessMadM;
sample.lidarPlaneThicknessP95M = planeInfo.thicknessP95M;
sample.lidarPlanePointCount = planeInfo.inlierCount;
sample.lidarPlaneCandidateCount = planeInfo.candidateCount;
sample.lidarPlaneExtentM = [planeInfo.extentU planeInfo.extentV];
sample.lidarPlaneRangeInitialM = planeRange;
sample.lidarReflectivityMedian = median(planeReflectivity);
sample.radarCenter = radarCenter;
sample.metaOut = data.metaOut;
end

function center = extract_radar_center(pointsByFrame, labelsByFrame, reference, opts)
centers = zeros(0,3);
for frameIndex = 1:numel(pointsByFrame)
    points = double(pointsByFrame{frameIndex}); labels = double(labelsByFrame{frameIndex});
    if isempty(points), continue; end
    candidates = zeros(0,3);
    clusterIds = unique(labels(labels>0));
    for clusterId = clusterIds(:).'
        candidates(end+1,:) = mean(points(labels==clusterId,1:3),1); %#ok<AGROW>
    end
    if isempty(candidates), candidates = points(:,1:3); end
    distance = vecnorm(candidates-reference,2,2);
    [minimumDistance,index] = min(distance);
    if minimumDistance <= opts.radarValidationRadiusM
        centers(end+1,:) = candidates(index,:); %#ok<AGROW>
    end
end
if isempty(centers), center = [NaN NaN NaN]; else, center = median(centers,1); end
end

function range = dominant_range(values, expected, opts)
edges = (min(values)-opts.rangeBinM):opts.rangeBinM:(max(values)+2*opts.rangeBinM);
[counts,edges] = histcounts(values,edges);
centers = (edges(1:end-1)+edges(2:end))/2;
score = counts .* exp(-.5*((centers-expected)/opts.rangePriorSigmaM).^2);
[~,index] = max(score); range = centers(index);
end

function [rotation,translation] = rigid_fit(source,target)
sourceCenter=mean(source,1); targetCenter=mean(target,1);
[U,~,V]=svd((source-sourceCenter).'*(target-targetCenter));
correction=eye(3); correction(3,3)=sign(det(V*U.'));
rotation=V*correction*U.';
translation=targetCenter-(rotation*sourceCenter.').';
end

function [translation, afterError, looError] = translation_metrics(source, target, rotation)
translation = median(target-(rotation*source.').',1);
predicted = (rotation*source.').'+translation;
afterError = vecnorm(predicted-target,2,2);
looError = nan(size(afterError));
for holdout=1:size(source,1)
    fitMask=true(size(source,1),1); fitMask(holdout)=false;
    foldTranslation=median(target(fitMask,:)-(rotation*source(fitMask,:).').',1);
    foldPredicted=(rotation*source(holdout,:).').'+foldTranslation;
    looError(holdout)=norm(foldPredicted-target(holdout,:));
end
end

function plot_real_image_comparison(samples, cameraExtrinsics, outputPath)
figureHandle=figure('Color','w','Position',[40 40 1800 1050],'Visible','off');
layout=tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
for index=1:numel(samples)
    axisHandle=nexttile(layout,index); imshow(samples(index).image,'Parent',axisHandle); hold(axisHandle,'on');
    corners=samples(index).checkerCorners;
    plot(axisHandle,corners(:,1),corners(:,2),'.','Color',[.2 1 .2],'MarkerSize',8);
    referenceUv=radar_to_image(samples(index).depthCenterRadar,samples(index).metaOut,cameraExtrinsics);
    beforeUv=radar_to_image(samples(index).lidarCenterBeforeRadar,samples(index).metaOut,cameraExtrinsics);
    afterUv=radar_to_image(samples(index).lidarCenterAfterRadar,samples(index).metaOut,cameraExtrinsics);
    plot(axisHandle,referenceUv(1),referenceUv(2),'gp','MarkerSize',18,'LineWidth',2.5);
    plot(axisHandle,beforeUv(1),beforeUv(2),'x','Color',[1 .45 0],'MarkerSize',16,'LineWidth',3);
    plot(axisHandle,afterUv(1),afterUv(2),'c+','MarkerSize',17,'LineWidth',3);
    plot(axisHandle,[referenceUv(1) beforeUv(1)],[referenceUv(2) beforeUv(2)],'-','Color',[1 .45 0],'LineWidth',1.3);
    plot(axisHandle,[referenceUv(1) afterUv(1)],[referenceUv(2) afterUv(2)],'c-','LineWidth',1.3);
    title(axisHandle,sprintf('%s | 初值 %.0f mm → 标定后 %.0f mm | 留一 %.0f mm', ...
        samples(index).name,samples(index).beforeErrorM*1000,samples(index).afterErrorM*1000, ...
        samples(index).leaveOneOutErrorM*1000),'Interpreter','none','FontSize',9);
end
legendAxis=nexttile(layout,6); axis(legendAxis,'off'); hold(legendAxis,'on');
plot(legendAxis,nan,nan,'gp','MarkerSize',18,'LineWidth',2.5);
plot(legendAxis,nan,nan,'x','Color',[1 .45 0],'MarkerSize',16,'LineWidth',3);
plot(legendAxis,nan,nan,'c+','MarkerSize',17,'LineWidth',3);
legend(legendAxis,{'深度棋盘中心（已在毫米波坐标）','LiDAR安装测量初值','LiDAR标定后'}, ...
    'Location','northwest','FontSize',13);
text(legendAxis,0,.55,{'坐标基准：ADT7880毫米波雷达','绿色小点：实图检测的36个棋盘角点', ...
    '橙色→青色：LiDAR板心标定前后','毫米波点云验证结果另存于对应点CSV'},'FontSize',13,'VerticalAlignment','top');
exportgraphics(figureHandle,outputPath,'Resolution',170); close(figureHandle);
end

function plot_summary(samples, rotation, translation, opts, outputPath)
reference=vertcat(samples.depthCenterRadar); before=vertcat(samples.lidarCenterBeforeRadar); after=vertcat(samples.lidarCenterAfterRadar);
beforeError=[samples.beforeErrorM].'; afterError=[samples.afterErrorM].'; loo=[samples.leaveOneOutErrorM].';
figureHandle=figure('Color','w','Position',[50 50 1450 700],'Visible','off');
layout=tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
axisHandle=nexttile(layout); hold(axisHandle,'on');
plot3(axisHandle,reference(:,1),reference(:,2),reference(:,3),'gp','MarkerSize',13,'LineWidth',2);
plot3(axisHandle,before(:,1),before(:,2),before(:,3),'x','Color',[1 .45 0],'MarkerSize',12,'LineWidth',2);
plot3(axisHandle,after(:,1),after(:,2),after(:,3),'c+','MarkerSize',13,'LineWidth',2);
for index=1:size(reference,1)
    plot3(axisHandle,[reference(index,1) before(index,1)],[reference(index,2) before(index,2)], ...
        [reference(index,3) before(index,3)],'-','Color',[1 .6 .2]);
    plot3(axisHandle,[reference(index,1) after(index,1)],[reference(index,2) after(index,2)], ...
        [reference(index,3) after(index,3)],'c-');
end
grid(axisHandle,'on'); axis(axisHandle,'equal'); view(axisHandle,[-35 22]);
xlabel(axisHandle,'X右 / m'); ylabel(axisHandle,'Y前 / m'); zlabel(axisHandle,'Z上 / m');
legend(axisHandle,{'深度参考','LiDAR标定前','LiDAR标定后'},'Location','best');
title(axisHandle,sprintf('R correction %.2f° | t [%+.3f %+.3f %+.3f] m', ...
    rotation_angle_deg(rotation*opts.initialRotation.'),translation));
axisHandle=nexttile(layout); bar(axisHandle,[beforeError afterError loo]*1000);
grid(axisHandle,'on'); ylabel(axisHandle,'板心三维误差 / mm');
set(axisHandle,'XTick',1:numel(samples),'XTickLabel',erase({samples.name},'20261118_'),'XTickLabelRotation',20);
legend(axisHandle,{'安装初值','五组拟合后','留一验证'},'Location','northwest');
title(axisHandle,sprintf('中位：初值 %.0f mm，拟合 %.0f mm，留一 %.0f mm', ...
    median(beforeError)*1000,median(afterError)*1000,median(loo)*1000));
title(layout,'LiDAR → 毫米波雷达外参标定汇总','FontSize',16);
exportgraphics(figureHandle,outputPath,'Resolution',170); close(figureHandle);
end

function plot_tilt_compensation(samples, legacyTranslation, translation, opts, outputPath)
reference=vertcat(samples.depthCenterRadar);
thinNative=vertcat(samples.lidarCenterThinSliceNative);
planeNative=vertcat(samples.lidarCenterNative);
thinAfter=(opts.initialRotation*thinNative.').'+legacyTranslation;
planeAfter=(opts.initialRotation*planeNative.').'+translation;
thinError=vecnorm(thinAfter-reference,2,2)*1000;
planeError=vecnorm(planeAfter-reference,2,2)*1000;
shift=vecnorm(vertcat(samples.lidarTiltCompensationShiftM),2,2)*1000;
tilt=[samples.lidarPlaneTiltDeg].';
figureHandle=figure('Color','w','Position',[50 50 1450 700],'Visible','off');
layout=tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
axisHandle=nexttile(layout); bar(axisHandle,[thinError planeError]); grid(axisHandle,'on');
ylabel(axisHandle,'板心三维误差 / mm'); legend(axisHandle,{'旧固定距离薄片','倾斜平面补偿'},'Location','best');
set(axisHandle,'XTick',1:numel(samples),'XTickLabel',erase({samples.name},'20261118_'),'XTickLabelRotation',20);
title(axisHandle,sprintf('拟合误差中位：%.0f → %.0f mm',median(thinError),median(planeError)));
axisHandle=nexttile(layout); yyaxis(axisHandle,'left'); bar(axisHandle,shift); ylabel(axisHandle,'板心补偿位移 / mm');
yyaxis(axisHandle,'right'); plot(axisHandle,tilt,'o-','LineWidth',1.8); ylabel(axisHandle,'板面倾角 / deg');
grid(axisHandle,'on'); set(axisHandle,'XTick',1:numel(samples),'XTickLabel',erase({samples.name},'20261118_'),'XTickLabelRotation',20);
title(axisHandle,'倾角只修正板心，不修改传感器安装旋转');
title(layout,'MID-70 倾斜标定板补偿前后对比','FontSize',16);
exportgraphics(figureHandle,outputPath,'Resolution',170); close(figureHandle);
end

function uv=radar_to_image(radarPoint,meta,cameraExtrinsics)
if any(~isfinite(radarPoint)), uv=[NaN NaN]; return; end
camera=(cameraExtrinsics.cameraToRadarR.'*(radarPoint-cameraExtrinsics.cameraOriginInRadarM).').';
fx=double(meta.fx); fy=double(meta.fy); cx=double(meta.cx); cy=double(meta.cy);
uv=[fx*camera(1)/camera(3)+cx+1, fy*camera(2)/camera(3)+cy+1];
end

function tableOut=sample_table(samples)
count=numel(samples); sample_id=strings(count,1); lidar=zeros(count,3); radar=zeros(count,3);
before=zeros(count,1); after=zeros(count,1); loo=zeros(count,1); radarValidation=zeros(count,3);
thin=zeros(count,3); shift=zeros(count,3); normal=zeros(count,3); tilt=zeros(count,1);
thicknessMad=zeros(count,1); thicknessP95=zeros(count,1); inliers=zeros(count,1); candidates=zeros(count,1);
for index=1:count
    sample_id(index)=string(samples(index).name); lidar(index,:)=samples(index).lidarCenterNative;
    radar(index,:)=samples(index).depthCenterRadar; before(index)=samples(index).beforeErrorM;
    after(index)=samples(index).afterErrorM; loo(index)=samples(index).leaveOneOutErrorM;
    radarValidation(index,:)=samples(index).radarCenter;
    thin(index,:)=samples(index).lidarCenterThinSliceNative;
    shift(index,:)=samples(index).lidarTiltCompensationShiftM;
    normal(index,:)=samples(index).lidarPlaneNormalInitialRadar;
    tilt(index)=samples(index).lidarPlaneTiltDeg;
    thicknessMad(index)=samples(index).lidarPlaneThicknessMadM;
    thicknessP95(index)=samples(index).lidarPlaneThicknessP95M;
    inliers(index)=samples(index).lidarPlanePointCount;
    candidates(index)=samples(index).lidarPlaneCandidateCount;
end
tableOut=table(sample_id,lidar(:,1),lidar(:,2),lidar(:,3),radar(:,1),radar(:,2),radar(:,3), ...
    thin(:,1),thin(:,2),thin(:,3),shift(:,1),shift(:,2),shift(:,3), ...
    normal(:,1),normal(:,2),normal(:,3),tilt,thicknessMad,thicknessP95,inliers,candidates, ...
    before,after,loo,radarValidation(:,1),radarValidation(:,2),radarValidation(:,3), ...
    'VariableNames',{'sample_id','lidar_x','lidar_y','lidar_z','radar_x','radar_y','radar_z', ...
    'thin_slice_lidar_x','thin_slice_lidar_y','thin_slice_lidar_z', ...
    'tilt_center_shift_x','tilt_center_shift_y','tilt_center_shift_z', ...
    'plane_normal_x','plane_normal_y','plane_normal_z','plane_tilt_deg', ...
    'plane_thickness_mad_m','plane_thickness_p95_m','plane_inliers','plane_candidates', ...
    'before_error_m','after_error_m','leave_one_out_error_m','radar_validation_x','radar_validation_y','radar_validation_z'});
end

function sample=empty_sample()
sample=struct('name','','takeDir','','resultPath','','frameIndex',0,'radarTimeMs',NaN, ...
    'colorFrame',0,'imagePath','','image',[],'checkerCorners',[],'boardSize',[], ...
    'depthCenterRadar',[NaN NaN NaN],'depthBoardPoints',[],'lidarCenterNative',[NaN NaN NaN], ...
    'lidarCenterThinSliceNative',[NaN NaN NaN],'lidarCenterThinSliceInitialRadar',[NaN NaN NaN], ...
    'lidarTiltCompensationShiftM',[NaN NaN NaN],'lidarPlaneNormalInitialRadar',[NaN NaN NaN], ...
    'lidarPlaneTiltDeg',NaN,'lidarPlaneThicknessMadM',NaN,'lidarPlaneThicknessP95M',NaN, ...
    'lidarCenterInitialRadar',[NaN NaN NaN],'lidarCenterBeforeRadar',[NaN NaN NaN], ...
    'lidarCenterAfterRadar',[NaN NaN NaN],'lidarPlanePointCount',0, ...
    'lidarPlaneCandidateCount',0,'lidarPlaneExtentM',[NaN NaN], ...
    'lidarPlaneRangeInitialM',NaN,'lidarReflectivityMedian',NaN,'radarCenter',[NaN NaN NaN], ...
    'beforeErrorM',NaN,'afterErrorM',NaN,'leaveOneOutErrorM',NaN,'metaOut',struct());
end

function opts=defaults(opts)
values=struct('lidarWindowMs',1000,'initialRotation',[0 -1 0;1 0 0;0 0 1], ...
    'initialTranslation',[-.10 0 0],'minimumDepthPoints',20,'depthPlaneToleranceM',.08, ...
    'boardPaddingXM',.08,'boardPaddingZM',.08,'rangeSearchM',.35,'rangeBinM',.005, ...
    'rangePriorSigmaM',.18,'lidarPlaneToleranceM',.012,'minimumLidarPoints',25, ...
    'maxBoardTiltDeg',20,'planeExtentPercentiles',[3 97],'randomSeed',70, ...
    'radarValidationRadiusM',.90,'solveMode','translation_only', ...
    'maxFitMedianM',.12,'maxFitMaxM',.20,'maxLooMedianM',.20, ...
    'takeNames',strings(0,1),'cameraExtrinsicsFile','', ...
    'cameraToRadarR',[],'cameraOriginInRadarM',[], ...
    'autoComputeMissingResults',true);
names=fieldnames(values);
for index=1:numel(names)
    if ~isfield(opts,names{index})||isempty(opts.(names{index})),opts.(names{index})=values.(names{index});end
end
end

function validate_options(opts)
validateattributes(opts.initialRotation,{'numeric'},{'size',[3 3],'finite'});
validateattributes(opts.initialTranslation,{'numeric'},{'numel',3,'finite'});
if norm(opts.initialRotation*opts.initialRotation.'-eye(3),'fro')>1e-6 || ...
        abs(det(opts.initialRotation)-1)>1e-6
    error('initialRotation must be a valid 3-by-3 rotation matrix.');
end
if ~isempty(opts.cameraToRadarR)
    validateattributes(opts.cameraToRadarR,{'numeric'},{'size',[3 3],'finite'});
    if norm(opts.cameraToRadarR*opts.cameraToRadarR.'-eye(3),'fro')>1e-6 || ...
            abs(det(opts.cameraToRadarR)-1)>1e-6
        error('cameraToRadarR must be a valid 3-by-3 rotation matrix.');
    end
end
if ~isempty(opts.cameraOriginInRadarM)
    validateattributes(opts.cameraOriginInRadarM,{'numeric'},{'numel',3,'finite'});
end
if ~any(strcmpi(opts.solveMode,{'translation_only','rigid'}))
    error('solveMode must be translation_only or rigid.');
end
end

function takeDirs=select_take_dirs(dataRoot,takeNames)
if ~isfolder(dataRoot),error('Data root does not exist: %s',dataRoot);end
allDirs=dir(dataRoot);
allDirs=allDirs([allDirs.isdir] & ~ismember({allDirs.name},{'.','..'}));
if isempty(takeNames)
    takeDirs=allDirs;
    return;
end
takeNames=cellstr(string(takeNames(:)));
available={allDirs.name};
[found,index]=ismember(takeNames,available);
if any(~found)
    error('Take directories not found under %s: %s',dataRoot,strjoin(takeNames(~found),', '));
end
takeDirs=allDirs(index);
end

function points=remap_depth_points(points,meta,cameraExtrinsics)
if isempty(points),return;end
if ~isfield(meta,'cameraToRadarR') || ~isfield(meta,'cameraOriginInRadarM')
    error('Processed result lacks the camera extrinsics needed to remap depth points.');
end
oldRotation=double(meta.cameraToRadarR);
oldTranslation=reshape(double(meta.cameraOriginInRadarM),1,3);
cameraPoints=(oldRotation.'*(points-oldTranslation).').';
points=(cameraExtrinsics.cameraToRadarR*cameraPoints.').'+ ...
    reshape(cameraExtrinsics.cameraOriginInRadarM,1,3);
end

function value=rotation_angle_deg(rotation)
value=rad2deg(acos(max(-1,min(1,(trace(rotation)-1)/2))));
end

function value=percentile(values,percentage)
values=sort(values(isfinite(values))); if isempty(values),value=NaN;return;end
if numel(percentage)>1,value=arrayfun(@(p)percentile(values,p),percentage);return;end
position=1+(numel(values)-1)*percentage/100; low=floor(position); high=ceil(position);
if low==high,value=values(low);else,value=values(low)+(position-low)*(values(high)-values(low));end
end

function write_json(path,value)
fileId=fopen(path,'w','n','UTF-8'); if fileId<0,error('Cannot write %s',path);end
cleanup=onCleanup(@()fclose(fileId)); fwrite(fileId,jsonencode(value,'PrettyPrint',true),'char');
end

function text=pass_text(passed)
if passed,text='PASS';else,text='FAIL';end
end

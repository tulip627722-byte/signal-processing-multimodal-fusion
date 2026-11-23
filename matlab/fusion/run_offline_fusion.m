function report = run_offline_fusion(takeDir, opts)
%RUN_OFFLINE_FUSION Generate and validate a reusable static fusion artifact.
if nargin < 1 || isempty(takeDir), error('请提供 take 目录。'); end
if nargin < 2, opts = struct(); end
here = fileparts(mfilename('fullpath'));
addpath(here, '-begin');
if ~isfield(opts, 'savePly'), opts.savePly = true; end
demo = run_static_fusion_demo(takeDir, opts);
o=fusion_defaults(opts,fileparts(here));
demo.planeAnalysis.guidedRadarBoard=guided_board_return(takeDir,demo.planeAnalysis,o);
[~, name] = fileparts(takeDir); %#ok<ASGLU>
offlineMat = [demo.outputBase '_offline_fusion.mat'];
offlinePly = [offlineMat(1:end-4) '.ply'];
offlineReport = [offlineMat(1:end-4) '_report.json'];
installation = struct();
if isfield(demo, 'data') && isfield(demo.data, 'meta') && isfield(demo.data.meta, 'calibrationInstallation')
    installation = demo.data.meta.calibrationInstallation;
end
quality = struct('status','FAIL','depthLidarStatus','FAIL','radarBoardStatus','FAIL', ...
    'depthLidarMedianM',demo.diagnosis.before.medianM,'depthLidarP95M',demo.diagnosis.before.p95M, ...
    'boardCandidateClusterIndices',demo.planeAnalysis.boardCandidateClusterIndices);
quality.guidedRadarBoard=demo.planeAnalysis.guidedRadarBoard;
if quality.guidedRadarBoard.valid && quality.guidedRadarBoard.errorM<=.15
    quality.radarBoardStatus='PASS';
end
if quality.depthLidarMedianM<=.20 && quality.depthLidarP95M<=.35
    quality.depthLidarStatus='PASS';
elseif quality.depthLidarMedianM<=.25 && quality.depthLidarP95M<=.50
    quality.depthLidarStatus='WARN';
end
if strcmp(quality.radarBoardStatus,'PASS') && strcmp(quality.depthLidarStatus,'PASS')
    quality.status='PASS';
elseif strcmp(quality.radarBoardStatus,'PASS') && strcmp(quality.depthLidarStatus,'WARN')
    quality.status='WARN';
end
save(offlineMat, 'demo', 'installation', 'quality', '-v7.3');
if opts.savePly && isfile([demo.outputBase '_fused.ply'])
    copyfile([demo.outputBase '_fused.ply'], offlinePly, 'f');
end
json = struct('takeDir',takeDir,'offlineMat',offlineMat,'offlinePly',offlinePly, ...
    'installation',installation,'quality',quality,'sourceFusedMat',[demo.outputBase '_fused.mat']);
fid=fopen(offlineReport,'w','n','UTF-8'); if fid<0,error('无法写入 %s',offlineReport);end
cleanup=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(json,'PrettyPrint',true),'char');
report=demo; report.offlineMat=offlineMat; report.offlinePly=offlinePly; report.offlineReport=offlineReport; report.quality=quality;
fprintf('离线融合结果：%s\n质量状态：%s（雷达板面 %s，Depth/MID-70 %s）\n', ...
    offlineMat,quality.status,quality.radarBoardStatus,quality.depthLidarStatus);
end

function result=guided_board_return(takeDir,plane,o)
result=struct('valid',false,'point',[NaN NaN NaN],'rawRangeM',NaN,'correctedRangeM',NaN,'errorM',NaN,'peakProminenceDb',NaN);
if ~plane.valid||~isfile(o.guidedRadarRangeCandidateFile),return;end
candidate=jsondecode(fileread(o.guidedRadarRangeCandidateFile));if ~isfield(candidate,'applicable')||~candidate.applicable,return;end
matlabDir=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(matlabDir,'tools'),'-begin');
diagnostic=diagnose_guided_radar_board(takeDir,plane.plane_center);if ~diagnostic.reliable,return;end
corrected=max(diagnostic.peakRangeM-double(candidate.rangeBiasM),.1);direction=plane.plane_center/norm(plane.plane_center);point=corrected*direction;
result=struct('valid',true,'point',point,'rawRangeM',diagnostic.peakRangeM,'correctedRangeM',corrected, ...
    'errorM',norm(point-plane.plane_center),'peakProminenceDb',diagnostic.peakProminenceDb,'candidateFile',o.guidedRadarRangeCandidateFile);
end

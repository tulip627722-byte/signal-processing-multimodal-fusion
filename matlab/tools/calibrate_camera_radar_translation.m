function report = calibrate_camera_radar_translation(dataRoot, resultsRoot, outputDir, opts)
%CALIBRATE_CAMERA_RADAR_TRANSLATION Build a gated camera->radar candidate.
% Rotation remains fixed. Five checkerboard takes provide depth/radar center
% pairs; each complete take is held out once for validation.
here = fileparts(mfilename('fullpath'));
matlabDir = fileparts(here); projectRoot = fileparts(matlabDir);
if nargin < 1 || isempty(dataRoot), dataRoot = fullfile(projectRoot,'..','11-18'); end
if nargin < 2 || isempty(resultsRoot), resultsRoot = fullfile(projectRoot,'results','takes'); end
if nargin < 3 || isempty(outputDir)
    outputDir = fullfile(projectRoot,'results','camera_radar_translation_candidate');
end
if nargin < 4, opts = struct(); end
o = defaults(opts, matlabDir);
if ~isfolder(outputDir), mkdir(outputDir); end
addpath(matlabDir,'-begin'); addpath(here,'-begin');
E = load_radar_camera_extrinsics(o.extrinsicsFile);

takeDirs = select_takes(dataRoot, o.takeNames);
if numel(takeDirs) < 5, error('需要至少五组棋盘采集，当前只有 %d 组。',numel(takeDirs)); end
samples = repmat(empty_sample(),numel(takeDirs),1);
for index=1:numel(takeDirs)
    takeDir=fullfile(takeDirs(index).folder,takeDirs(index).name);
    resultFile=fullfile(resultsRoot,[takeDirs(index).name '_res.mat']);
    if ~isfile(resultFile), error('缺少处理结果：%s。先运行 compute_all 强制重算。',resultFile); end
    samples(index)=extract_sample(takeDir,resultFile,E,o);
    fprintf('%s：相机中心 [%+.3f %.3f %+.3f]，毫米波中心 [%+.3f %.3f %+.3f]\n', ...
        samples(index).name,samples(index).cameraCenter,samples(index).radarCenter);
end

cameraCenters=vertcat(samples.cameraCenter); radarCenters=vertcat(samples.radarCenter);
solution=solve_camera_radar_translation(cameraCenters,radarCenters,E.cameraToRadarR, ...
    E.cameraOriginInRadarM,o.solver);
for index=1:numel(samples)
    samples(index).baselineErrorM=solution.baselineErrorM(index);
    samples(index).fitErrorM=solution.fitErrorM(index);
    samples(index).leaveOneOutErrorM=solution.leaveOneOutErrorM(index);
end

candidate=struct('version',2,'sourceFrame','depth_camera_openni','targetFrame','radar', ...
    'units','m','cameraAxes','X right, Y down, Z forward', ...
    'radarAxes','X right, Y forward, Z up','cameraToRadarR',E.cameraToRadarR, ...
    'cameraOriginInRadarM',solution.translation,'status',solution.status, ...
    'applicable',solution.applicable,'source','calibrate_camera_radar_translation', ...
    'fitSamples',numel(samples),'fitMedianErrorM',median(solution.fitErrorM), ...
    'leaveOneOutMedianErrorM',median(solution.leaveOneOutErrorM), ...
    'leaveOneOutP90ErrorM',percentile(solution.leaveOneOutErrorM,90), ...
    'improvementRatio',solution.improvementRatio,'boundMarginFraction',solution.boundMarginFraction, ...
    'diagnosticTranslationM',solution.diagnosticTranslationM, ...
    'diagnosticRadarRangeBiasM',solution.diagnosticRangeBiasM, ...
    'diagnosticTranslationShiftM',solution.diagnosticTranslationShiftM, ...
    'bootstrapCI95M',solution.bootstrapCI95M, ...
    'note',candidate_note(solution));

tableOut=sample_table(samples);
csvPath=fullfile(outputDir,'camera_radar_translation_folds.csv'); writetable(tableOut,csvPath);
jsonPath=fullfile(outputDir,'radar_camera_extrinsics_candidate.json'); write_json(jsonPath,candidate);
comparisonPath=fullfile(outputDir,'translation_before_after.png');
plot_comparison(samples,E,solution,comparisonPath);
lidarCheck=lidar_crosscheck(samples,E,solution,projectRoot);
report=struct('samples',samples,'solution',solution,'candidate',candidate, ...
    'lidarCrosscheck',lidarCheck,'jsonPath',jsonPath,'csvPath',csvPath, ...
    'comparisonPath',comparisonPath);
save(fullfile(outputDir,'calibration_report.mat'),'report','-v7.3');

fprintf('\n相机 -> 毫米波平移候选：%s\n',solution.status);
fprintf('  当前 t [%+.4f %+.4f %+.4f] m\n',E.cameraOriginInRadarM);
fprintf('  候选 t [%+.4f %+.4f %+.4f] m\n',solution.translation);
fprintf('  留一中位 %.1f mm，P90 %.1f mm，改善 %.1f%%\n', ...
    median(solution.leaveOneOutErrorM)*1000,percentile(solution.leaveOneOutErrorM,90)*1000, ...
    100*solution.improvementRatio);
fprintf('  平移/测距混淆检查：平移变化 %.1f mm\n',solution.diagnosticTranslationShiftM*1000);
fprintf('  候选：%s\n',jsonPath);
end

function sample=extract_sample(takeDir,resultFile,E,o)
data=load(resultFile,'pts','labels','depthPts','depthUV','colorFrame','depthDt','metaOut');
validTime=find(abs(double(data.depthDt))<=o.maxDepthTimeResidualMs);
if isempty(validTime), error('%s 没有时间残差合格帧。',takeDir); end
frameIndex=validTime(max(1,round(numel(validTime)/2)));
imagePath=fullfile(takeDir,'color',sprintf('%06d.png',double(data.colorFrame(frameIndex))));
image=imread(imagePath); warningState=warning('off','all');
[corners,~]=detectCheckerboardPoints(image); warning(warningState);
if isempty(corners), error('棋盘检测失败：%s',imagePath); end
hull=convhull(corners(:,1),corners(:,2));
uv=double(data.depthUV{frameIndex})+1; depthRadar=double(data.depthPts{frameIndex});
inside=inpolygon(uv(:,1),uv(:,2),corners(hull,1),corners(hull,2));
board=depthRadar(inside,:);
if size(board,1)<o.minimumDepthPoints, error('%s 棋盘深度点不足：%d',takeDir,size(board,1)); end
rangeCenter=median(board(:,2)); board=board(abs(board(:,2)-rangeCenter)<=o.depthPlaneToleranceM,:);
depthCenterRadar=median(board,1);
cameraCenter=(E.cameraToRadarR.'*(depthCenterRadar-E.cameraOriginInRadarM).').';
radarCenter=extract_radar_center(data.pts,data.labels,depthCenterRadar,o);
if any(~isfinite(radarCenter)), error('%s 没有稳定毫米波中心。',takeDir); end
[~,name]=fileparts(takeDir); sample=empty_sample();
sample.name=name;sample.frameIndex=frameIndex;sample.cameraCenter=cameraCenter;
sample.depthCenterRadar=depthCenterRadar;sample.radarCenter=radarCenter;
sample.depthPointCount=size(board,1);sample.imagePath=imagePath;
end

function center=extract_radar_center(pointsByFrame,labelsByFrame,reference,o)
centers=zeros(0,3);
for frameIndex=1:numel(pointsByFrame)
    points=double(pointsByFrame{frameIndex}); labels=double(labelsByFrame{frameIndex});
    if isempty(points),continue;end
    ids=unique(labels(labels>0)); candidates=zeros(0,3);
    for id=ids(:).',candidates(end+1,:)=mean(points(labels==id,1:3),1);end %#ok<AGROW>
    if isempty(candidates),candidates=points(:,1:3);end
    [distance,index]=min(vecnorm(candidates-reference,2,2));
    if distance<=o.radarValidationRadiusM,centers(end+1,:)=candidates(index,:);end %#ok<AGROW>
end
if size(centers,1)<o.minimumRadarFrames,center=[NaN NaN NaN];else,center=median(centers,1);end
end

function check=lidar_crosscheck(samples,E,solution,projectRoot)
check=struct('available',false,'note','未找到可用 LiDAR 标定报告。');
folders=dir(fullfile(projectRoot,'results','lidar_radar_calibration_*'));folders=folders([folders.isdir]);
[~,order]=sort([folders.datenum],'descend');folders=folders(order);
for folder=folders(:).'
    path=fullfile(folder.folder,folder.name,'calibration_report.mat');if ~isfile(path),continue;end
    loaded=load(path,'report');if ~isfield(loaded,'report')||~isfield(loaded.report,'samples'),continue;end
    lidarSamples=loaded.report.samples; before=[]; after=[];
    for index=1:numel(samples)
        match=find(strcmp({lidarSamples.name},samples(index).name),1);if isempty(match),continue;end
        lidarPoint=lidarSamples(match).lidarCenterAfterRadar;
        oldDepth=(E.cameraToRadarR*samples(index).cameraCenter.').'+E.cameraOriginInRadarM;
        newDepth=(E.cameraToRadarR*samples(index).cameraCenter.').'+solution.translation;
        before(end+1,1)=norm(oldDepth-lidarPoint);after(end+1,1)=norm(newDepth-lidarPoint); %#ok<AGROW>
    end
    if ~isempty(before)
        check=struct('available',true,'sampleCount',numel(before), ...
            'beforeMedianM',median(before),'afterMedianM',median(after), ...
            'beforeP90M',percentile(before,90),'afterP90M',percentile(after,90), ...
            'note','MID-70 外参曾由深度板心引导，仅作非独立辅助检查，不参与 PASS 判定。');
        return;
    end
end
end

function plot_comparison(samples,E,solution,path)
camera=vertcat(samples.cameraCenter);radar=vertcat(samples.radarCenter);
before=(E.cameraToRadarR*camera.').'+E.cameraOriginInRadarM;
after=(E.cameraToRadarR*camera.').'+solution.translation;
figureHandle=figure('Visible','off','Color','w','Position',[50 50 1400 650]);
layout=tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
axisHandle=nexttile(layout);hold(axisHandle,'on');grid(axisHandle,'on');axis(axisHandle,'equal');
plot3(axisHandle,radar(:,1),radar(:,2),radar(:,3),'rp','MarkerSize',13,'LineWidth',2);
plot3(axisHandle,before(:,1),before(:,2),before(:,3),'kx','MarkerSize',11,'LineWidth',2);
plot3(axisHandle,after(:,1),after(:,2),after(:,3),'c+','MarkerSize',12,'LineWidth',2);
for i=1:size(radar,1),plot3(axisHandle,[radar(i,1) after(i,1)],[radar(i,2) after(i,2)],[radar(i,3) after(i,3)],'c-');end
xlabel(axisHandle,'X右 / m');ylabel(axisHandle,'Y前 / m');zlabel(axisHandle,'Z上 / m');view(axisHandle,-35,22);
legend(axisHandle,{'毫米波中心','当前外参','候选外参'},'Location','best');title(axisHandle,'中心对应');
axisHandle=nexttile(layout);bar(axisHandle,[solution.baselineErrorM solution.fitErrorM solution.leaveOneOutErrorM]*1000);
grid(axisHandle,'on');ylabel(axisHandle,'三维误差 / mm');set(axisHandle,'XTick',1:numel(samples), ...
    'XTickLabel',erase({samples.name},'20261118_'),'XTickLabelRotation',20);
legend(axisHandle,{'当前外参','全组拟合','整组留一'},'Location','northwest');
title(layout,sprintf('相机→毫米波平移候选 %s | t=[%+.3f %+.3f %+.3f] m', ...
    solution.status,solution.translation));exportgraphics(figureHandle,path,'Resolution',170);close(figureHandle);
end

function tableOut=sample_table(samples)
n=numel(samples);name=strings(n,1);camera=zeros(n,3);radar=zeros(n,3);before=zeros(n,1);fit=zeros(n,1);loo=zeros(n,1);
for i=1:n,name(i)=string(samples(i).name);camera(i,:)=samples(i).cameraCenter;radar(i,:)=samples(i).radarCenter; ...
        before(i)=samples(i).baselineErrorM;fit(i)=samples(i).fitErrorM;loo(i)=samples(i).leaveOneOutErrorM;end
tableOut=table(name,camera(:,1),camera(:,2),camera(:,3),radar(:,1),radar(:,2),radar(:,3),before,fit,loo, ...
    'VariableNames',{'take','cameraX','cameraY','cameraZ','radarX','radarY','radarZ','baselineErrorM','fitErrorM','leaveOneOutErrorM'});
end

function dirs=select_takes(root,names)
dirs=dir(root);dirs=dirs([dirs.isdir]);dirs=dirs(~ismember({dirs.name},{'.','..'}));
dirs=dirs(arrayfun(@(d)isfile(fullfile(d.folder,d.name,'meta.json')),dirs));
if ~isempty(names),dirs=dirs(ismember(string({dirs.name}),string(names)));end
[~,order]=sort({dirs.name});dirs=dirs(order);
end

function sample=empty_sample()
sample=struct('name','','frameIndex',0,'cameraCenter',[NaN NaN NaN], ...
    'depthCenterRadar',[NaN NaN NaN],'radarCenter',[NaN NaN NaN], ...
    'depthPointCount',0,'imagePath','','baselineErrorM',NaN,'fitErrorM',NaN,'leaveOneOutErrorM',NaN);
end

function note=candidate_note(solution)
if solution.applicable
    note='PASS: fixed rotation; translation passed complete-take holdout, bound, improvement, and range-bias-confounding gates.';
else
    note='FAIL: diagnostic candidate only; one or more complete-take holdout, bound, improvement, or range-bias-confounding gates failed.';
end
end

function write_json(path,value)
file=fopen(path,'w');if file<0,error('无法写入 %s',path);end
cleanup=onCleanup(@()fclose(file));fwrite(file,jsonencode(value,'PrettyPrint',true),'char');
end

function value=percentile(values,p)
values=sort(values(:));position=1+(numel(values)-1)*p/100;low=floor(position);high=ceil(position);
value=values(low)*(high-position)+values(high)*(position-low);if low==high,value=values(low);end
end

function o=defaults(o,matlabDir)
d=struct('takeNames',string.empty(0,1),'extrinsicsFile',fullfile(matlabDir,'radar_camera_extrinsics.json'), ...
    'maxDepthTimeResidualMs',33,'minimumDepthPoints',80,'depthPlaneToleranceM',0.06, ...
    'radarValidationRadiusM',0.75,'minimumRadarFrames',3,'solver',struct());
fields=fieldnames(d);for i=1:numel(fields),if ~isfield(o,fields{i}),o.(fields{i})=d.(fields{i});end,end
end

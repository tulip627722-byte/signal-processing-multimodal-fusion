function report = calibrate_extrinsics_from_res(opts)
%CALIBRATE_EXTRINSICS_FROM_RES 仅用 3.2 res.mat 标定雷达/深度相机外参。
%
%   report = calibrate_extrinsics_from_res()
%   report = calibrate_extrinsics_from_res(struct('randomSeed', 1104))
%
% 产物位于 results/extrinsics_calibration；正式外参文件不会被覆盖。

if nargin < 1, opts = struct(); end
here = fileparts(mfilename('fullpath'));
matlabDir = fileparts(here); root32 = fileparts(matlabDir);
o = defaults(opts, root32);
addpath(matlabDir, '-begin');
if ~isfolder(o.outputDir), mkdir(o.outputDir); end

groups = calibration_groups();
n = size(groups,1);
samples = repmat(empty_sample(), n, 1);
fprintf('\n===== 基于 3.2 点云的雷达/相机外参标定 =====\n');
for i=1:n
    fprintf('  [%d/%d] %s\n',i,n,groups{i,1});
    withData = load_result(o.resultsRoot, groups{i,2});
    noData = load_result(o.resultsRoot, groups{i,3});
    [depthCenter, depthInfo] = depth_board_center(withData, noData, groups{i,4}, o);
    candidates = radar_candidates(withData, noData, depthCenter, o);
    samples(i).name=groups{i,1}; samples(i).withTake=groups{i,2};
    samples(i).noBoardTake=groups{i,3}; samples(i).nominalRangeM=groups{i,4};
    samples(i).cameraCenter=depthCenter; samples(i).depthInfo=depthInfo;
    samples(i).radarCandidates=candidates;
    samples(i).valid=all(isfinite(depthCenter)) && ~isempty(candidates);
    plot_detection(samples(i),o);
    fprintf('       相机中心 [%+.3f %+.3f %+.3f]，雷达候选 %d 个\n', ...
        depthCenter,numel(candidates));
end

valid=find([samples.valid]);
if numel(valid)<5
    report=failed_report(samples,sprintf('只有 %d 组有效共同中心，少于要求的 5 组。',numel(valid)),o);
    write_outputs(report,o); return;
end

camera=vertcat(samples(valid).cameraCenter);
candidates={samples(valid).radarCandidates}.';
solverOpts=struct('randomSeed',o.randomSeed,'multiStarts',o.multiStarts);
solution=solve_extrinsics_candidates(camera,candidates,solverOpts);

[folds,foldMedian,baselineMedian]=cross_validate(camera,candidates,solverOpts,o);
bootstrap=bootstrap_intervals(camera,candidates,solverOpts,o);
for k=1:numel(valid)
    i=valid(k); j=solution.selectedCandidate(k);
    samples(i).selectedCandidate=j;
    samples(i).selectedRadarCenter=samples(i).radarCandidates(j).center;
    samples(i).correctedRadarCenter=solution.correctedRadar(k,:);
    samples(i).predictedRadarCenter=solution.predictedRadar(k,:);
    samples(i).residual=solution.residuals(k,:);
    samples(i).errorNormM=solution.errorNormM(k);
end

candidateMad=nan(numel(valid),1);
for k=1:numel(valid)
    candidateMad(k)=samples(valid(k)).radarCandidates(solution.selectedCandidate(k)).madM;
end
angleMargin=min(solution.boundMarginFraction(1:3));
tMargin=min(solution.boundMarginFraction(4:6));
improvement=1-foldMedian/max(baselineMedian,eps);
checks=struct('validGroups',numel(valid)>=5, ...
    'candidateStability',median(candidateMad,'omitnan')<=o.maxCandidateMadM, ...
    'notAtAngleBounds',angleMargin>=o.minBoundMarginFraction, ...
    'notAtTranslationBounds',tMargin>=o.minBoundMarginFraction, ...
    'crossValidation',foldMedian<=o.maxValidationMedianM, ...
    'improvement',improvement>=o.minImprovementRatio);
pass=all(structfun(@(x)logical(x),checks));
reason=acceptance_reason(checks,foldMedian,improvement,candidateMad,o);

report=struct('version',1,'createdAt',char(datetime('now','Format','yyyy-MM-dd HH:mm:ss')), ...
    'source','3.2 results/takes res.mat only','status',ternary(pass,'PASS','FAIL'), ...
    'applicable',pass,'reason',reason,'samples',samples,'validIndices',valid, ...
    'solution',solution,'crossValidation',folds,'crossValidationMedianM',foldMedian, ...
    'baselineMedianM',baselineMedian,'improvementRatio',improvement, ...
    'bootstrap',bootstrap,'checks',checks,'options',o);
plot_alignment(report,o); report.holdoutPlotIndex=plot_holdout(report,o);
write_outputs(report,o);

fprintf('\n候选 R/t 求解完成：%s\n',report.status);
fprintf('  角度补偿 [roll pitch yaw] = [%+.3f %+.3f %+.3f] deg\n',solution.anglesDegRadarXYZ);
fprintf('  t = [%+.4f %+.4f %+.4f] m，雷达径向偏差 %.4f m\n',solution.t,solution.rangeBiasM);
fprintf('  六折中位 %.3f m；基线 %.3f m；改善 %.1f%%\n',foldMedian,baselineMedian,100*improvement);
fprintf('  %s\n  输出：%s\n',reason,o.outputDir);
end

function groups=calibration_groups()
groups={ ...
 '2.5m-1','20261104_132932_室内2米5_1','20261104_133249_室内2米5_3_（无板子）',2.5; ...
 '2.5m-2','20261104_133030_室内2米5_2','20261104_133249_室内2米5_3_（无板子）',2.5; ...
 '2.0m-1','20261104_133930_室内2米_1','20261104_134333_室内2米_3（无板子）',2.0; ...
 '2.0m-2','20261104_134227_室内2米_2','20261104_134333_室内2米_3（无板子）',2.0; ...
 '3.0m-1','20261104_134904_室内3米_1','20261104_135142_室内3米_3（无板子）',3.0; ...
 '3.0m-2','20261104_134957_室内3米_2','20261104_135142_室内3米_3（无板子）',3.0};
end

function s=empty_sample()
s=struct('name','','withTake','','noBoardTake','','nominalRangeM',NaN, ...
 'cameraCenter',nan(1,3),'depthInfo',struct(),'radarCandidates',struct([]), ...
 'valid',false,'selectedCandidate',NaN,'selectedRadarCenter',nan(1,3), ...
 'correctedRadarCenter',nan(1,3),'predictedRadarCenter',nan(1,3), ...
 'residual',nan(1,3),'errorNormM',NaN);
end

function D=load_result(root,name)
path=fullfile(root,[name '_res.mat']);
if ~isfile(path),error('找不到结果：%s',path);end
D=load(path,'pts','depthPts','labels','metaOut','frames');
D.name=name; D.path=path;
end

function [center,info]=depth_board_center(withData,noData,nominal,o)
% 旧 res 使用固定轴+地面归一化；新 res 使用其记录的逆外参。
[withPoints,oldR,oldT]=aggregate_depth(withData,o);
[noPoints,~,~]=aggregate_depth(noData,o);
radarGuess=[0 nominal -0.40];
lo=radarGuess- [1.15 0.65 0.85]; hi=radarGuess+[1.15 0.65 0.95];
edges=make_edges(lo,hi,o.depthVoxelM);
withDensity=voxel_presence(withData.depthPts,edges,o.maxDepthPointsPerFrame);
noDensity=voxel_presence(noData.depthPts,edges,o.maxDepthPointsPerFrame);
diff=max(withDensity-noDensity,0);
threshold=max(o.depthDiffMin,prctile(diff(diff>0),o.depthDiffPercentile));
mask=diff>=threshold;
components=grid_components(mask);
bestScore=-inf; bestIdx=[];
for k=1:numel(components)
    idx=components{k};
    if numel(idx)<o.depthMinVoxels,continue;end
    xyz=voxel_centers(idx,edges,size(mask));
    extent=max(xyz,[],1)-min(xyz,[],1);
    if extent(1)<0.15 || extent(3)<0.15 || extent(2)>0.35,continue;end
    score=sum(diff(idx))*sqrt(numel(idx))/(1+4*extent(2));
    if score>bestScore,bestScore=score;bestIdx=idx;end
end
if isempty(bestIdx)
    center=nan(1,3);info=struct('status','no_depth_board_component');return;
end
vox=voxel_centers(bestIdx,edges,size(mask));
expand=o.depthVoxelM*1.8;
keep=withPoints(:,1)>=min(vox(:,1))-expand & withPoints(:,1)<=max(vox(:,1))+expand & ...
     withPoints(:,2)>=min(vox(:,2))-expand & withPoints(:,2)<=max(vox(:,2))+expand & ...
     withPoints(:,3)>=min(vox(:,3))-expand & withPoints(:,3)<=max(vox(:,3))+expand;
P=withPoints(keep,:);
if size(P,1)<80,center=nan(1,3);info=struct('status','too_few_depth_points');return;end
[planeCenter,normal,inlier,extent]=robust_plane(P,o);
centerRadar=planeCenter;
center=(oldR.'*(centerRadar-oldT).').';
info=struct('status','ok','centerRadarOld',centerRadar,'normalRadarOld',normal, ...
 'extentM',extent,'pointCount',size(P,1),'inlierCount',nnz(inlier), ...
 'voxelCount',numel(bestIdx),'score',bestScore,'oldR',oldR,'oldT',oldT, ...
 'bboxRadarOld',[min(P(inlier,:),[],1);max(P(inlier,:),[],1)]);
end

function [allPoints,R,t]=aggregate_depth(D,o)
R=o.R0;t=[0 0 0];
if isfield(D.metaOut,'cameraToRadarR'),R=double(D.metaOut.cameraToRadarR);end
if isfield(D.metaOut,'cameraOriginInRadarM'),t=reshape(double(D.metaOut.cameraOriginInRadarM),1,3);end
allPoints=[];
pick=unique(round(linspace(1,numel(D.depthPts),min(o.depthFrames,numel(D.depthPts)))));
for i=pick
    P=double(D.depthPts{i});
    if size(P,1)>o.maxDepthPointsPerFrame
        P=P(round(linspace(1,size(P,1),o.maxDepthPointsPerFrame)),:);
    end
    allPoints=[allPoints;P]; %#ok<AGROW>
end
end

function candidates=radar_candidates(withData,noData,cameraCenter,o)
expected=(o.R0*cameraCenter.').'+[0 0 0.10];
lo=expected-o.radarSearchHalfM;hi=expected+o.radarSearchHalfM;
edges=make_edges(lo,hi,o.radarVoxelM);
[occW,intW,framesW,pointsW]=radar_voxels(withData,edges);
[occN,intN]=radar_voxels(noData,edges);
diffOcc=max(occW-occN,0);
diffInt=max(intW-intN,0);
score=diffOcc.*(1+0.15*diffInt).*sqrt(max(framesW,0));
positive=score(score>0);
if isempty(positive),candidates=struct([]);return;end
threshold=max(o.radarScoreMin,prctile(positive,o.radarScorePercentile));
active=score>=threshold & framesW>=o.radarMinFrameRatio;
components=grid_components(active);
raw=repmat(struct('center',[],'score',0,'confidence',0,'madM',NaN, ...
 'persistence',0,'meanIntensityDb',NaN,'frameCount',0,'pointCount',0, ...
 'voxelCount',0,'compactness',0),0,1);
for k=1:numel(components)
    idx=components{k};
    if numel(idx)<1,continue;end
    xyz=voxel_centers(idx,edges,size(active));w=score(idx);
    center=sum(xyz.*w,1)/sum(w);
    near=vecnorm(pointsW.xyz-center,2,2)<=o.radarCandidateRadiusM;
    if nnz(near)<o.radarMinPoints,continue;end
    P=pointsW.xyz(near,:);frame=pointsW.frame(near);intensity=pointsW.intensity(near);
    frameCenters=[];
    for f=unique(frame).'
        Q=P(frame==f,:);if size(Q,1)>=2,frameCenters(end+1,:)=median(Q,1);end %#ok<AGROW>
    end
    if isempty(frameCenters),continue;end
    center=median(frameCenters,1);
    madM=median(vecnorm(frameCenters-center,2,2));
    persistence=size(frameCenters,1)/numel(withData.pts);
    compactness=1/(1+madM/o.radarVoxelM);
    raw(end+1)=struct('center',center,'score',sum(w),'confidence',0, ... %#ok<AGROW>
      'madM',madM,'persistence',persistence,'meanIntensityDb',mean(intensity), ...
      'frameCount',size(frameCenters,1),'pointCount',nnz(near),'voxelCount',numel(idx), ...
      'compactness',compactness);
end
if isempty(raw),candidates=struct([]);return;end
rankScore=[raw.score].*[raw.persistence].*[raw.compactness];
[~,order]=sort(rankScore,'descend');raw=raw(order(1:min(o.maxRadarCandidates,numel(order))));
top=max(rankScore); for k=1:numel(raw)
    raw(k).confidence=min(1,max(0,([raw(k).score]*raw(k).persistence*raw(k).compactness)/max(top,eps)));
end
candidates=raw;
end

function density=voxel_presence(frames,edges,maxPoints)
shape=cellfun(@(x)numel(x)-1,edges);density=zeros(shape);
for i=1:numel(frames)
 P=double(frames{i});if isempty(P),continue;end
 if size(P,1)>maxPoints,P=P(round(linspace(1,size(P,1),maxPoints)),:);end
 idx=voxel_index(P,edges,shape);idx=unique(idx(idx>0));density(idx)=density(idx)+1;
end
density=density/max(numel(frames),1);
end

function [occ,meanInt,frameRatio,points]=radar_voxels(D,edges)
shape=cellfun(@(x)numel(x)-1,edges);occ=zeros(shape);intSum=zeros(shape);intCount=zeros(shape);
xyz=[];frame=[];intensity=[];
for i=1:numel(D.pts)
 P=double(D.pts{i});if isempty(P),continue;end
 idx=voxel_index(P(:,1:3),edges,shape);valid=idx>0;
 if ~any(valid),continue;end
 iv=idx(valid);pv=P(valid,:);u=unique(iv);occ(u)=occ(u)+1;
 for j=1:numel(iv),intSum(iv(j))=intSum(iv(j))+pv(j,4);intCount(iv(j))=intCount(iv(j))+1;end
 xyz=[xyz;pv(:,1:3)];frame=[frame;repmat(i,size(pv,1),1)];intensity=[intensity;pv(:,4)]; %#ok<AGROW>
end
frameRatio=occ/max(numel(D.pts),1);occ=frameRatio;
meanInt=zeros(shape);good=intCount>0;meanInt(good)=intSum(good)./intCount(good);
points=struct('xyz',xyz,'frame',frame,'intensity',intensity);
end

function idx=voxel_index(P,edges,shape)
ix=floor((P(:,1)-edges{1}(1))/(edges{1}(2)-edges{1}(1)))+1;
iy=floor((P(:,2)-edges{2}(1))/(edges{2}(2)-edges{2}(1)))+1;
iz=floor((P(:,3)-edges{3}(1))/(edges{3}(2)-edges{3}(1)))+1;
valid=ix>=1&ix<=shape(1)&iy>=1&iy<=shape(2)&iz>=1&iz<=shape(3);
idx=zeros(size(ix));idx(valid)=sub2ind(shape,ix(valid),iy(valid),iz(valid));
end

function edges=make_edges(lo,hi,step)
edges=cell(1,3);for d=1:3,edges{d}=lo(d):step:(hi(d)+step);end
end

function xyz=voxel_centers(linear,edges,shape)
[ix,iy,iz]=ind2sub(shape,linear);
c=cell(1,3);for d=1:3,c{d}=(edges{d}(1:end-1)+edges{d}(2:end))/2;end
xyz=[c{1}(ix).',c{2}(iy).',c{3}(iz).'];
end

function components=grid_components(mask)
shape=size(mask);active=find(mask);components={};if isempty(active),return;end
lookup=false(shape);lookup(active)=true;seen=false(shape);
for seed=active(:).'
 if seen(seed),continue;end
 queue=seed;seen(seed)=true;component=zeros(0,1);
 while ~isempty(queue)
  current=queue(end);queue(end)=[];component(end+1,1)=current; %#ok<AGROW>
  [x,y,z]=ind2sub(shape,current);
  neighbors=[x-1 y z;x+1 y z;x y-1 z;x y+1 z;x y z-1;x y z+1];
  good=all(neighbors>=1,2)&neighbors(:,1)<=shape(1)&neighbors(:,2)<=shape(2)&neighbors(:,3)<=shape(3);
  neighbors=neighbors(good,:);
  ni=sub2ind(shape,neighbors(:,1),neighbors(:,2),neighbors(:,3));
  ni=ni(lookup(ni)&~seen(ni));seen(ni)=true;queue=[queue;ni]; %#ok<AGROW>
 end
 components{end+1}=component; %#ok<AGROW>
end
end

function [center,normal,inlier,extent]=robust_plane(P,o)
center=median(P,1);inlier=true(size(P,1),1);
for iter=1:5
 Q=P(inlier,:);center=median(Q,1);[~,~,V]=svd(Q-center,0);normal=V(:,3).';
 distance=abs((P-center)*normal.');scale=1.4826*median(abs(distance-median(distance)))+1e-5;
 inlier=distance<=max(o.depthPlaneToleranceM,2.5*scale);
end
Q=P(inlier,:);center=median(Q,1);[~,~,V]=svd(Q-center,0);normal=V(:,3).';
extent=max(Q,[],1)-min(Q,[],1);
end

function [folds,med,baselineMed]=cross_validate(camera,candidates,solverOpts,o)
n=size(camera,1);folds=repmat(struct('holdout',0,'errorM',NaN,'baselineErrorM',NaN, ...
 'anglesDeg',nan(1,3),'t',nan(1,3),'rangeBiasM',NaN),n,1);
errors=nan(n,1);base=nan(n,1);
for h=1:n
 train=setdiff(1:n,h);so=solverOpts;so.randomSeed=o.randomSeed+h;so.multiStarts=max(6,ceil(o.multiStarts/2));
 sol=solve_extrinsics_candidates(camera(train,:),candidates(train),so);
 [errors(h),~]=predict_holdout(camera(h,:),candidates{h},sol);
 base(h)=baseline_error(camera(h,:),candidates{h},o);
 folds(h)=struct('holdout',h,'errorM',errors(h),'baselineErrorM',base(h), ...
   'anglesDeg',sol.anglesDegRadarXYZ,'t',sol.t,'rangeBiasM',sol.rangeBiasM);
end
med=median(errors,'omitnan');baselineMed=median(base,'omitnan');
end

function [error,j]=predict_holdout(camera,candidates,sol)
pred=(sol.R*camera.').'+sol.t;centers=vertcat(candidates.center);r=vecnorm(centers,2,2);
corrected=centers.*(max(r-sol.rangeBiasM,0.1)./max(r,eps));[error,j]=min(vecnorm(corrected-pred,2,2));
end

function e=baseline_error(camera,candidates,o)
pred=(o.R0*camera.').'+[0 0 0.10];centers=vertcat(candidates.center);[e,~]=min(vecnorm(centers-pred,2,2));
end

function bootstrap=bootstrap_intervals(camera,candidates,solverOpts,o)
rng(o.randomSeed+99,'twister');n=size(camera,1);values=nan(o.bootstrapCount,7);
for b=1:o.bootstrapCount
 idx=randi(n,n,1);if numel(unique(idx))<3,continue;end
 so=solverOpts;so.multiStarts=4;so.randomSeed=o.randomSeed+100+b;
 try,s=solve_extrinsics_candidates(camera(idx,:),candidates(idx),so);values(b,:)=[s.anglesDegRadarXYZ s.t s.rangeBiasM];catch,end
end
good=all(isfinite(values),2);values=values(good,:);
if isempty(values),ci=nan(2,7);else,ci=[prctile(values,2.5,1);prctile(values,97.5,1)];end
bootstrap=struct('samples',values,'ci95',ci,'columns',{{'rollDeg','pitchDeg','yawDeg','tx','ty','tz','rangeBiasM'}});
end

function plot_detection(sample,o)
fig=figure('Visible','off','Color','w','Position',[100 100 1100 480]);
subplot(1,2,1);c=sample.depthInfo.centerRadarOld;b=sample.depthInfo.bboxRadarOld;
scatter3(c(1),c(2),c(3),120,'b','filled');hold on;draw_box(b);grid on;axis equal;xlabel X;ylabel Y;zlabel Z;
title(sprintf('%s 深度差分板心，平面内点 %d',sample.name,sample.depthInfo.inlierCount));
subplot(1,2,2);hold on;colors=lines(max(1,numel(sample.radarCandidates)));
for k=1:numel(sample.radarCandidates),q=sample.radarCandidates(k).center;scatter3(q(1),q(2),q(3),100,colors(k,:),'filled');text(q(1),q(2),q(3),sprintf('  #%d C%.2f MAD%.2f',k,sample.radarCandidates(k).confidence,sample.radarCandidates(k).madM));end
grid on;axis equal;xlabel X;ylabel Y;zlabel Z;title('雷达差分候选');
exportgraphics(fig,fullfile(o.outputDir,sprintf('detect_%s.png',sample.name)),'Resolution',140);close(fig);
end

function draw_box(b)
x=[b(1,1) b(2,1)];y=[b(1,2) b(2,2)];z=[b(1,3) b(2,3)];
v=[x(1) y(1) z(1);x(2) y(1) z(1);x(2) y(2) z(1);x(1) y(2) z(1); ...
   x(1) y(1) z(2);x(2) y(1) z(2);x(2) y(2) z(2);x(1) y(2) z(2)];
e=[1 2;2 3;3 4;4 1;5 6;6 7;7 8;8 5;1 5;2 6;3 7;4 8];
for i=1:size(e,1),plot3(v(e(i,:),1),v(e(i,:),2),v(e(i,:),3),'b-','HandleVisibility','off');end
end

function plot_alignment(report,o)
valid=report.validIndices;s=report.samples;fig=figure('Visible','off','Color','w','Position',[100 100 1200 600]);
ax=axes(fig);hold(ax,'on');
for i=valid(:).'
 p=s(i).predictedRadarCenter;q=s(i).correctedRadarCenter;
 scatter3(ax,p(1),p(2),p(3),100,'b','filled');scatter3(ax,q(1),q(2),q(3),100,'ro','LineWidth',1.5);
 plot3(ax,[p(1) q(1)],[p(2) q(2)],[p(3) q(3)],'k-','HandleVisibility','off');text(q(1),q(2),q(3),[' ' s(i).name]);
end
grid(ax,'on');axis(ax,'equal');xlabel(ax,'X / m');ylabel(ax,'Y / m');zlabel(ax,'Z / m');view(ax,-35,24);
title(ax,sprintf('候选外参 %s：蓝=相机预测，红=雷达候选；六折中位 %.3f m',report.status,report.crossValidationMedianM));
legend(ax,{'相机经 R,t','雷达径向补偿后'},'Location','best');
exportgraphics(fig,fullfile(o.outputDir,'alignment_before_after.png'),'Resolution',160);close(fig);
end

function holdoutIndex=plot_holdout(report,o)
rng(o.randomSeed,'twister');valid=report.validIndices;holdoutIndex=valid(randi(numel(valid)));
s=report.samples(holdoutIndex);fold=report.crossValidation(find(valid==holdoutIndex,1));
D=load_result(o.resultsRoot,s.withTake);k=max(1,round(numel(D.pts)/2));
radar=double(D.pts{k}(:,1:3));rr=vecnorm(radar,2,2);
radar=radar.*(max(rr-report.solution.rangeBiasM,0.1)./max(rr,eps));
depth=double(D.depthPts{k});oldR=o.R0;oldT=[0 0 0];
if isfield(D.metaOut,'cameraToRadarR'),oldR=double(D.metaOut.cameraToRadarR);end
if isfield(D.metaOut,'cameraOriginInRadarM'),oldT=reshape(double(D.metaOut.cameraOriginInRadarM),1,3);end
camera=(oldR.'*(depth-oldT).').';depthNew=(report.solution.R*camera.').'+report.solution.t;
if size(depthNew,1)>5000,depthNew=depthNew(round(linspace(1,size(depthNew,1),5000)),:);end
fig=figure('Visible','off','Color','w','Position',[100 100 1300 560]);
subplot(1,2,1);scatter3(depthNew(:,1),depthNew(:,2),depthNew(:,3),3,[0.2 0.55 0.9],'filled');hold on;
scatter3(radar(:,1),radar(:,2),radar(:,3),24,'ro','LineWidth',1);grid on;axis equal;view(-35,24);xlabel('X / m');ylabel('Y / m');zlabel('Z / m');
title(sprintf('随机留出 %s：候选外参点云叠加',s.name));legend({'深度点云→候选雷达系','雷达点云→径向补偿'});
subplot(1,2,2);bar([fold.baselineErrorM fold.errorM]);set(gca,'XTickLabel',{'固定轴+10cm','候选外参'});ylabel('板心三维误差 / m');grid on;
title(sprintf('补偿前 %.3f m，补偿后 %.3f m',fold.baselineErrorM,fold.errorM));
exportgraphics(fig,fullfile(o.outputDir,'holdout_validation.png'),'Resolution',160);close(fig);
end

function write_outputs(report,o)
save(fullfile(o.outputDir,'extrinsics_calibration_report.mat'),'report');
rows=[];for i=1:numel(report.samples),s=report.samples(i);rows=[rows;{s.name,s.valid,s.cameraCenter(1),s.cameraCenter(2),s.cameraCenter(3),s.selectedCandidate,s.errorNormM}];end %#ok<AGROW>
T=cell2table(rows,'VariableNames',{'group','valid','cameraX','cameraY','cameraZ','selectedCandidate','errorM'});
writetable(T,fullfile(o.outputDir,'extrinsics_calibration_report.csv'));
if isfield(report,'solution')
 sol=report.solution;candidate=struct('version',1,'status',report.status,'applicable',report.applicable, ...
  'source','calibrate_extrinsics_from_res','cameraToRadarR',sol.R, ...
  'cameraOriginInRadarM',sol.t,'rotationCorrectionDegRadarXYZ',sol.anglesDegRadarXYZ, ...
  'radarRangeBiasM',sol.rangeBiasM,'crossValidationMedianM',report.crossValidationMedianM, ...
  'baselineMedianM',report.baselineMedianM,'improvementRatio',report.improvementRatio, ...
  'bootstrapCI95',report.bootstrap.ci95,'note',report.reason);
 write_json(fullfile(o.outputDir,'radar_camera_extrinsics_candidate.json'),candidate);
end
end

function write_json(path,value)
fid=fopen(path,'w');if fid<0,error('无法写入 %s',path);end
cleanup=onCleanup(@()fclose(fid));fwrite(fid,jsonencode(value,'PrettyPrint',true),'char');
end

function report=failed_report(samples,reason,o)
report=struct('version',1,'createdAt',char(datetime('now')),'source','3.2 res.mat only', ...
 'status','FAIL','applicable',false,'reason',reason,'samples',samples,'options',o);
end

function text=acceptance_reason(c,foldMedian,improvement,mad,o)
if all(structfun(@logical,c))
 text=sprintf('通过：六折中位 %.3f m，较基线改善 %.1f%%。',foldMedian,100*improvement);return;
end
failed={};f=fieldnames(c);for i=1:numel(f),if ~c.(f{i}),failed{end+1}=f{i};end,end %#ok<AGROW>
text=sprintf('未通过：%s；六折中位 %.3f m（门限 %.3f），改善 %.1f%%，候选 MAD 中位 %.3f m。', ...
 strjoin(failed,', '),foldMedian,o.maxValidationMedianM,100*improvement,median(mad,'omitnan'));
end

function value=ternary(condition,a,b)
if condition,value=a;else,value=b;end
end

function o=defaults(o,root32)
d=struct('resultsRoot',fullfile(root32,'results','takes'), ...
 'outputDir',fullfile(root32,'results','extrinsics_calibration'), ...
 'R0',[1 0 0;0 0 1;0 -1 0],'randomSeed',1104,'multiStarts',14, ...
 'depthVoxelM',0.05,'radarVoxelM',0.10,'depthFrames',12, ...
 'maxDepthPointsPerFrame',5000,'depthDiffMin',0.08,'depthDiffPercentile',75, ...
 'depthMinVoxels',8,'depthPlaneToleranceM',0.045, ...
 'radarSearchHalfM',[0.80 0.80 0.80],'radarScoreMin',0.01, ...
 'radarScorePercentile',72,'radarMinFrameRatio',0.05,'radarCandidateRadiusM',0.30, ...
 'radarMinPoints',12,'maxRadarCandidates',5,'maxCandidateMadM',0.15, ...
 'minBoundMarginFraction',0.015,'maxValidationMedianM',0.15, ...
 'minImprovementRatio',0.20,'bootstrapCount',36);
f=fieldnames(d);for i=1:numel(f),if ~isfield(o,f{i}),o.(f{i})=d.(f{i});end,end
end

function candidate = calibrate_radar_measurement_correction(correspondenceFile, outputFile)
%CALIBRATE_RADAR_MEASUREMENT_CORRECTION Fit guarded radar-only correction.
if nargin < 1 || isempty(correspondenceFile), error('需要板心对应 CSV。'); end
if nargin < 2, outputFile = ''; end
tableData=readtable(correspondenceFile,'TextType','string');
truth=[tableData.radar_x tableData.radar_y tableData.radar_z];
measured=[tableData.radar_validation_x tableData.radar_validation_y tableData.radar_validation_z];
valid=all(isfinite(truth),2)&all(isfinite(measured),2);truth=truth(valid,:);measured=measured(valid,:);
if size(truth,1)<5,error('至少需要五组有效板心对应。');end
biasGrid=(-0.30:0.001:0.30).';
[rotation,bias,fitError]=fit_model(measured,truth,biasGrid);
loo=zeros(size(truth,1),1);
for holdout=1:size(truth,1)
    fit=true(size(truth,1),1);fit(holdout)=false;
    [foldRotation,foldBias]=fit_model(measured(fit,:),truth(fit,:),biasGrid);
    loo(holdout)=norm(apply_model(measured(holdout,:),foldRotation,foldBias)-truth(holdout,:));
end
fitMedian=median(fitError);looMedian=median(loo);looP95=percentile_value(loo,95);
applicable=fitMedian<=0.15 && looMedian<=0.15 && looP95<=0.30;
candidate=struct('version',1,'source','radar_board_center_guarded_fit', ...
    'model','rotation_plus_radial_bias','rotation',rotation,'rangeBiasM',bias, ...
    'fitErrorM',fitError,'fitMedianM',fitMedian,'leaveOneOutErrorM',loo, ...
    'leaveOneOutMedianM',looMedian,'leaveOneOutP95M',looP95, ...
    'status',pass_text(applicable),'applicable',applicable, ...
    'note','Never apply unless applicable=true; no XYZ snapping or free translation.');
if ~isempty(outputFile)
    fid=fopen(outputFile,'w','n','UTF-8');if fid<0,error('无法写入 %s',outputFile);end
    cleanup=onCleanup(@()fclose(fid));fwrite(fid,jsonencode(candidate,'PrettyPrint',true),'char');
end
fprintf('雷达测角候选：%s；拟合中位 %.0f mm，留一中位 %.0f mm，P95 %.0f mm\n', ...
    candidate.status,fitMedian*1000,looMedian*1000,looP95*1000);
end

function [rotation,bias,error] = fit_model(measured,truth,biasGrid)
best=inf;rotation=eye(3);bias=0;error=inf(size(truth,1),1);
for currentBias=biasGrid(:).'
    corrected=radial_correct(measured,currentBias);[u,~,v]=svd(corrected.'*truth);
    fix=eye(3);fix(3,3)=sign(det(v*u.'));currentRotation=v*fix*u.';
    currentError=vecnorm((currentRotation*corrected.').'-truth,2,2);
    score=median(currentError);
    if score<best,best=score;rotation=currentRotation;bias=currentBias;error=currentError;end
end
end
function points=apply_model(points,rotation,bias),points=(rotation*radial_correct(points,bias).').';end
function points=radial_correct(points,bias),range=vecnorm(points,2,2);points=points.*(max(range-bias,.1)./max(range,eps));end
function value=percentile_value(values,p),values=sort(values(:));position=1+(numel(values)-1)*p/100;lo=floor(position);hi=ceil(position);if lo==hi,value=values(lo);else,value=values(lo)+(position-lo)*(values(hi)-values(lo));end;end
function value=pass_text(pass),if pass,value='PASS';else,value='FAIL';end;end

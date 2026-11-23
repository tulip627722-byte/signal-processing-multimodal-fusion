function candidate = calibrate_guided_radar_board_range(dataRoot, correspondenceFile, outputFile)
%CALIBRATE_GUIDED_RADAR_BOARD_RANGE Validate radar range for geometry-guided fusion.
if nargin < 1 || isempty(dataRoot), error('需要静态 take 根目录。'); end
if nargin < 2 || isempty(correspondenceFile), error('需要板心对应 CSV。'); end
if nargin < 3, outputFile=''; end
here=fileparts(mfilename('fullpath'));addpath(fullfile(fileparts(here),'tools'),'-begin');
tableData=readtable(correspondenceFile,'TextType','string');count=height(tableData);
measured=zeros(count,1);truth=zeros(count,1);prominence=zeros(count,1);
for index=1:count
    center=[tableData.radar_x(index) tableData.radar_y(index) tableData.radar_z(index)];
    result=diagnose_guided_radar_board(fullfile(dataRoot,char(tableData.sample_id(index))),center);
    measured(index)=result.peakRangeM;truth(index)=norm(center);prominence(index)=result.peakProminenceDb;
end
bias=median(measured-truth);loo=zeros(count,1);
for holdout=1:count
    fit=true(count,1);fit(holdout)=false;foldBias=median(measured(fit)-truth(fit));
    loo(holdout)=abs(measured(holdout)-foldBias-truth(holdout));
end
looMedian=median(loo);looP95=percentile_value(loo,95);
applicable=all(prominence>=3)&&looMedian<=.08&&looP95<=.15;
candidate=struct('version',1,'model','geometry_bearing_plus_corrected_radar_range', ...
    'rangeBiasM',bias,'measuredRangeM',measured,'truthRangeM',truth, ...
    'peakProminenceDb',prominence,'leaveOneOutErrorM',loo, ...
    'leaveOneOutMedianM',looMedian,'leaveOneOutP95M',looP95, ...
    'status',pass_text(applicable),'applicable',applicable, ...
    'note','Radar contributes range/energy; aligned Depth-LiDAR geometry contributes bearing. Raw radar XYZ remains unchanged.');
if ~isempty(outputFile)
    fid=fopen(outputFile,'w','n','UTF-8');if fid<0,error('无法写入 %s',outputFile);end
    cleanup=onCleanup(@()fclose(fid));fwrite(fid,jsonencode(candidate,'PrettyPrint',true),'char');
end
fprintf('引导雷达距离候选：%s，偏差 %.0f mm，留一中位 %.0f mm，P95 %.0f mm\n',candidate.status,bias*1000,looMedian*1000,looP95*1000);
end
function value=percentile_value(values,p),values=sort(values(:));position=1+(numel(values)-1)*p/100;lo=floor(position);hi=ceil(position);if lo==hi,value=values(lo);else,value=values(lo)+(position-lo)*(values(hi)-values(lo));end;end
function value=pass_text(pass),if pass,value='PASS';else,value='FAIL';end;end

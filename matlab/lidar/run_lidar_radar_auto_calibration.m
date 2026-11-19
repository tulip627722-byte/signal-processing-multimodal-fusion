function report = run_lidar_radar_auto_calibration(config)
%RUN_LIDAR_RADAR_AUTO_CALIBRATION Configure and run checkerboard calibration.
%
% All positions use the ADT7880 frame and metres:
%   [right, forward, up]
% Example: a LiDAR 15 cm directly below radar is [0 0 -0.15].

if nargin<1||isempty(config),config=lidar_radar_calibration_config();end
required={'dataRoot','lidarOriginInRadarM'};
for index=1:numel(required)
    if ~isfield(config,required{index})||isempty(config.(required{index}))
        error('Calibration config requires %s.',required{index});
    end
end
here=fileparts(mfilename('fullpath'));
projectRoot=fileparts(fileparts(here));
config=set_default(config,'takeNames',strings(0,1));
config=set_default(config,'resultsRoot',fullfile(projectRoot,'results','takes'));
config=set_default(config,'cameraExtrinsicsFile',fullfile(projectRoot,'matlab','radar_camera_extrinsics.json'));
config=set_default(config,'cameraOriginInRadarM',[]);
config=set_default(config,'cameraToRadarR',[]);
config=set_default(config,'lidarToRadarR',[0 -1 0;1 0 0;0 0 1]);
config=set_default(config,'solveMode','translation_only');
config=set_default(config,'autoComputeMissingResults',true);
config=set_default(config,'installationConfigFile',fullfile(projectRoot,'capture','app','calibration_installation.json'));
timeTag=char(datetime('now','Format','yyyyMMdd_HHmmss'));
config=set_default(config,'outputDir',fullfile(projectRoot,'results', ...
    ['lidar_radar_calibration_' timeTag]));

validateattributes(config.lidarOriginInRadarM,{'numeric'},{'numel',3,'finite'});
opts=struct('takeNames',config.takeNames, ...
    'initialTranslation',reshape(double(config.lidarOriginInRadarM),1,3), ...
    'initialRotation',double(config.lidarToRadarR), ...
    'cameraExtrinsicsFile',config.cameraExtrinsicsFile, ...
    'cameraOriginInRadarM',config.cameraOriginInRadarM, ...
    'cameraToRadarR',config.cameraToRadarR, ...
    'solveMode',config.solveMode, ...
    'autoComputeMissingResults',logical(config.autoComputeMissingResults));
if isfield(config,'options')
    names=fieldnames(config.options);
    for index=1:numel(names),opts.(names{index})=config.options.(names{index});end
end
fprintf('Coordinate convention: [right, forward, up] metres in ADT7880 radar frame.\n');
fprintf('Camera origin initial = %s\n',vector_text(config.cameraOriginInRadarM,'from extrinsics file'));
fprintf('LiDAR origin initial  = [%+.4f %+.4f %+.4f] m\n',opts.initialTranslation);
fprintf('Output directory      = %s\n',config.outputDir);
report=calibrate_lidar_radar_checkerboards(config.dataRoot,config.resultsRoot, ...
    config.outputDir,opts);
report.config=config;
save(fullfile(config.outputDir,'calibration_report.mat'),'report','-v7.3');
end

function config=set_default(config,name,value)
if ~isfield(config,name)||isempty(config.(name)),config.(name)=value;end
end

function value=vector_text(vector,fallback)
if isempty(vector),value=fallback;else,value=sprintf('[%+.4f %+.4f %+.4f] m',vector);end
end


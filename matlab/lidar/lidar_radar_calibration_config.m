function config = lidar_radar_calibration_config()
%LIDAR_RADAR_CALIBRATION_CONFIG Edit these values before calibration.
% Positions are [right, forward, up] in metres in the ADT7880 radar frame.

projectRoot=fileparts(fileparts(fileparts(mfilename('fullpath'))));
config.installationConfigFile=fullfile(projectRoot,'capture','app','calibration_installation.json');

% Folder containing one subfolder per static checkerboard take.
config.dataRoot=fullfile(projectRoot,'..','11-18');

% Fill selected take folder names. Leave empty to use every subfolder.
config.takeNames=strings(0,1);

% Existing camera rotation is loaded from this formal extrinsics file.
config.cameraExtrinsicsFile=fullfile(projectRoot,'matlab','radar_camera_extrinsics.json');

% Camera optical origin relative to radar: [right, forward, up] metres.
% Leave empty when the camera has not moved and the formal file is valid.
config.cameraOriginInRadarM=[];

% Leave empty to retain camera rotation from the formal file. A measured
% XYZ distance cannot determine rotation; only override with calibrated R.
config.cameraToRadarR=[];

% LiDAR optical origin relative to radar: [right, forward, up] metres.
% Example: 15 cm directly below radar -> [0 0 -0.15].
config.lidarOriginInRadarM=[0 0 -0.15];

% Keep this fixed-axis rotation when LiDAR and radar point parallel.
config.lidarToRadarR=[0 -1 0;1 0 0;0 0 1];
config.solveMode='translation_only';

% Missing processed results are generated automatically.
config.autoComputeMissingResults=true;

% Optional overrides, for example:
% config.options=struct('lidarWindowMs',1000,'maxBoardTiltDeg',20);
config.options=struct();

if isfile(config.installationConfigFile)
    saved=jsondecode(fileread(config.installationConfigFile));
    if isfield(saved,'cameraOriginInRadarM'),config.cameraOriginInRadarM=reshape(double(saved.cameraOriginInRadarM),1,3);end
    if isfield(saved,'lidarOriginInRadarM'),config.lidarOriginInRadarM=reshape(double(saved.lidarOriginInRadarM),1,3);end
    if isfield(saved,'lidarToRadarR'),config.lidarToRadarR=double(saved.lidarToRadarR);end
end
end

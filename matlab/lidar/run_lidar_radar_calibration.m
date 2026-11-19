function report = run_lidar_radar_calibration(correspondenceCsv, outputJson)
%RUN_LIDAR_RADAR_CALIBRATION Solve and save a LiDAR-to-radar candidate.
here=fileparts(mfilename('fullpath'));
if nargin<1||isempty(correspondenceCsv),correspondenceCsv=fullfile(here,'lidar_radar_correspondences.csv');end
if nargin<2||isempty(outputJson),outputJson=fullfile(here,'lidar_radar_extrinsics_candidate.json');end
report=solve_lidar_radar_extrinsics(correspondenceCsv,struct('outputFile',outputJson));
fprintf('Candidate written to %s\n',outputJson);
end

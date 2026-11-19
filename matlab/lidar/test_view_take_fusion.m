function result = test_view_take_fusion()
%TEST_VIEW_TAKE_FUSION Regression checks for viewer fusion prerequisites.

here = fileparts(fileparts(mfilename('fullpath')));
addpath(here);
addpath(fileparts(mfilename('fullpath')));

rotation = [0 -1 0; 1 0 0; 0 0 1];
translation = [-0.1 0.2 0.05];
points = [1 2 3; -0.5 0.4 0.2];
expected = (rotation * points.').';
expected = expected + translation;
extrinsics = struct('lidarToRadarR', rotation, ...
    'lidarOriginInRadarM', translation, 'status', 'PASS', 'applicable', true);
cameraExtrinsics = struct('cameraToRadarR', eye(3), 'cameraOriginInRadarM', [0 0 0]);
actual = transform_multimodal_to_radar(points, zeros(0, 3), extrinsics, cameraExtrinsics);
assert(max(abs(actual.lidarRadar(:) - expected(:))) < 1e-12);

invalidPath = [tempname '.json'];
cleanup = onCleanup(@() delete_if_exists(invalidPath));
fid = fopen(invalidPath, 'w');
assert(fid >= 0);
fprintf(fid, '{"lidarToRadarR":[[1,0,0],[0,1,0],[0,0,1]],"lidarOriginInRadarM":[0,0,0],"status":"FAIL","applicable":false}');
fclose(fid);
invalid = load_lidar_radar_extrinsics(invalidPath);
assert(~logical(invalid.applicable));

takeDir = tempname;
mkdir(takeDir);
takeCleanup = onCleanup(@() remove_take(takeDir));
writetable(table([1; 2], [95; 195], [105; 205], ...
    'VariableNames', {'batch', 'first_t_rel_ms', 'last_t_rel_ms'}), ...
    fullfile(takeDir, 'lidar_frames.csv'));
writetable(table([11; 12], [95; 195], [105; 205], ...
    'VariableNames', {'frame', 'first_packet_t_rel_ms', 'last_packet_t_rel_ms'}), ...
    fullfile(takeDir, 'radar_frames.csv'));
pairs = pair_lidar_radar_frames(takeDir, 20);
assert(isequal(pairs.lidar_batch(:), [1; 2]));
assert(all(abs(pairs.dt_ms) < 1e-12));
assert(all(pairs.valid));

result = struct('pass', true, 'transformedPoints', actual.lidarRadar, ...
    'invalidExtrinsicsRejectedByApplicability', ~logical(invalid.applicable), ...
    'pairedFrames', height(pairs));
fprintf('PASS: view_take fusion transform and applicability guard.\n');
end

function delete_if_exists(path)
if isfile(path), delete(path); end
end

function remove_take(path)
if isfolder(path), rmdir(path, 's'); end
end

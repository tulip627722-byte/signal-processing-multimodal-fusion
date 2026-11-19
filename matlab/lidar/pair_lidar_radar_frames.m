function pairs = pair_lidar_radar_frames(takeDir, maxDeltaMs)
%PAIR_LIDAR_RADAR_FRAMES Match LiDAR batch midpoints to radar frame midpoints.

if nargin < 2 || isempty(maxDeltaMs), maxDeltaMs = 30; end
lidarPath = fullfile(takeDir, 'lidar_frames.csv');
radarPath = fullfile(takeDir, 'radar_frames.csv');
manifestPath = fullfile(takeDir, 'manifest.csv');
if isfile(lidarPath)
    lidar = readtable(lidarPath, 'TextType', 'string');
    lidarTime = (double(lidar.first_t_rel_ms) + double(lidar.last_t_rel_ms)) / 2;
    lidarBatch = double(lidar.batch);
else
    if ~isfile(manifestPath), error('Missing lidar_frames.csv and manifest.csv: %s', takeDir); end
    manifest = readtable(manifestPath, 'TextType', 'string');
    lidar = manifest(lower(strtrim(string(manifest.stream))) == "lidar", :);
    lidarTime = double(lidar.t_rel_ms);
    lidarBatch = double(lidar.frame);
end
if isfile(radarPath)
    radar = readtable(radarPath, 'TextType', 'string');
    radarTime = (double(radar.first_packet_t_rel_ms) + double(radar.last_packet_t_rel_ms)) / 2;
    radarFrame = double(radar.frame);
else
    if ~isfile(manifestPath), error('Missing radar_frames.csv and manifest.csv: %s', takeDir); end
    if ~exist('manifest', 'var'), manifest = readtable(manifestPath, 'TextType', 'string'); end
    radar = manifest(lower(strtrim(string(manifest.stream))) == "radar", :);
    radarTime = double(radar.t_rel_ms);
    radarFrame = double(radar.frame);
end
if isempty(lidarBatch), error('No LiDAR batches in %s', takeDir); end
if isempty(radarFrame), error('No radar frames in %s', takeDir); end
matchedRadarFrame = nan(size(lidarBatch)); matchedRadarTime = nan(size(lidarTime)); deltaMs = nan(size(lidarTime));
for index = 1:numel(lidarTime)
    [~, nearest] = min(abs(radarTime - lidarTime(index)));
    matchedRadarFrame(index) = radarFrame(nearest); matchedRadarTime(index) = radarTime(nearest);
    deltaMs(index) = lidarTime(index) - matchedRadarTime(index);
end
valid = abs(deltaMs) <= maxDeltaMs;
pairs = table(lidarBatch, lidarTime, matchedRadarFrame, matchedRadarTime, deltaMs, valid, ...
    'VariableNames', {'lidar_batch','lidar_t_ms','radar_frame','radar_t_ms','dt_ms','valid'});
end

function extrinsics = load_lidar_radar_extrinsics(path)
%LOAD_LIDAR_RADAR_EXTRINSICS Load and validate LiDAR-to-radar extrinsics.

if ~isfile(path), error('Cannot find LiDAR/radar extrinsics: %s', path); end
extrinsics = jsondecode(fileread(path));
required = {'lidarToRadarR', 'lidarOriginInRadarM'};
for index = 1:numel(required)
    if ~isfield(extrinsics, required{index}), error('Extrinsics file lacks %s', required{index}); end
end
extrinsics.lidarToRadarR = double(extrinsics.lidarToRadarR);
extrinsics.lidarOriginInRadarM = reshape(double(extrinsics.lidarOriginInRadarM), 1, 3);
validateattributes(extrinsics.lidarToRadarR, {'numeric'}, {'size',[3 3],'finite'});
orthogonalityError = norm(extrinsics.lidarToRadarR * extrinsics.lidarToRadarR.' - eye(3), 'fro');
if orthogonalityError > 1e-6 || abs(det(extrinsics.lidarToRadarR)-1) > 1e-6
    error('Invalid LiDAR-to-radar rotation: orthogonality %.3g, det %.6f', ...
        orthogonalityError, det(extrinsics.lidarToRadarR));
end
extrinsics.path = path;
end

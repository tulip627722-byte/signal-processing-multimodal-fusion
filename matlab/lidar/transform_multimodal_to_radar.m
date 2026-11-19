function output = transform_multimodal_to_radar(lidarPoints, cameraPoints, lidarExtrinsics, cameraExtrinsics)
%TRANSFORM_MULTIMODAL_TO_RADAR Put LiDAR and camera points in radar coordinates.

if ischar(lidarExtrinsics) || isstring(lidarExtrinsics)
    lidarExtrinsics = load_lidar_radar_extrinsics(lidarExtrinsics);
end
if ischar(cameraExtrinsics) || isstring(cameraExtrinsics)
    cameraExtrinsics = load_radar_camera_extrinsics(cameraExtrinsics);
end
output = struct();
output.lidarRadar = (double(lidarExtrinsics.lidarToRadarR) * double(lidarPoints).').';
output.lidarRadar = output.lidarRadar + reshape(double(lidarExtrinsics.lidarOriginInRadarM), 1, 3);
output.cameraRadar = (double(cameraExtrinsics.cameraToRadarR) * double(cameraPoints).').';
output.cameraRadar = output.cameraRadar + reshape(double(cameraExtrinsics.cameraOriginInRadarM), 1, 3);
end

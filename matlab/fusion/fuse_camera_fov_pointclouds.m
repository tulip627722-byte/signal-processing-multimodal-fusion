function [fusedPc, associations] = fuse_camera_fov_pointclouds(data, opts)
%FUSE_CAMERA_FOV_POINTCLOUDS Fuse labeled channels inside camera FOV.
if nargin < 2, opts = data.config; end
K = data.depth.intrinsics; fx=K(1); fy=K(2); cx=K(3); cy=K(4);
imageSize = size(data.depth.depthImage);
depthRadar = double(data.depth.radar); depthCam = double(data.depth.camera);
lidarRadar = double(data.lidar.radar); lidarCam = to_camera(lidarRadar, data.cameraExtrinsics);
radarRadar = double(data.radar.xyz); radarCam = to_camera(radarRadar, data.cameraExtrinsics);
[lidarKeep,lidarUv] = in_fov(lidarCam,fx,fy,cx,cy,imageSize,opts.fovMinDepthM);
[radarKeep,radarUv] = in_fov(radarCam,fx,fy,cx,cy,imageSize,opts.fovMinDepthM);
depthIdx = sample_indices((1:size(depthRadar,1)).',opts.maxFusionPointsPerSensor);
lidarIdx = sample_indices(find(lidarKeep),opts.maxFusionPointsPerSensor);
radarIdx = sample_indices(find(radarKeep),opts.maxFusionPointsPerSensor);
xyz = [depthRadar(depthIdx,:); lidarRadar(lidarIdx,:); radarRadar(radarIdx,:)];
xyzCam = [depthCam(depthIdx,:); lidarCam(lidarIdx,:); radarCam(radarIdx,:)];
uv = [data.depth.uv(depthIdx,:)+1; lidarUv(lidarIdx,:); radarUv(radarIdx,:)];
sensorId = uint8([ones(numel(depthIdx),1); 2*ones(numel(lidarIdx),1); 3*ones(numel(radarIdx),1)]);
rgb = [data.depth.rgb(depthIdx,:); repmat(uint8([128 128 128]),numel(lidarIdx)+numel(radarIdx),1)];
intensity = [nan(numel(depthIdx),1); data.lidar.intensity(lidarIdx); data.radar.intensity(radarIdx)];
doppler = [nan(numel(depthIdx)+numel(lidarIdx),1); data.radar.doppler_v(radarIdx)];
fusedPc = struct('xyz',xyz,'xyz_cam',xyzCam,'uv',uv,'sensor_id',sensorId,'rgb',rgb, ...
    'intensity',intensity,'doppler_v',doppler,'has_doppler',isfinite(doppler), ...
    'has_intensity',isfinite(intensity),'meta',struct('frame','ADT7880 radar', ...
    'fov','depth camera','counts',[numel(depthIdx) numel(lidarIdx) numel(radarIdx)],'intrinsics',K));
geometry = [depthRadar; lidarRadar]; geometrySensor = [ones(size(depthRadar,1),1); 2*ones(size(lidarRadar,1),1)];
associations = associate_radar(radarRadar(radarIdx,:),radarCam(radarIdx,:),geometry,geometrySensor, ...
    data.radar.intensity(radarIdx),data.radar.doppler_v(radarIdx),opts);
associations.cluster = associate_clusters(data.radar.clusters,geometry,geometrySensor,opts);
end

function output = associate_clusters(clusters,geometry,geometrySensor,opts)
output=struct('radar_centroid',{},'geometry_centroid',{},'geometry_sensor',{}, ...
    'cluster_index',{},'point_count',{},'geometry_point_count',{},'radial_error_m',{}, ...
    'azimuth_error_deg',{},'vertical_error_m',{},'error_norm_m',{}, ...
    'mean_intensity_db',{},'mean_doppler_mps',{});
for index=1:numel(clusters)
    radar=double(clusters(index).centroid);radarRange=norm(radar);radarAz=atan2d(radar(1),radar(2));
    geometryRange=sqrt(sum(geometry.^2,2));geometryAz=atan2d(geometry(:,1),geometry(:,2));
    candidate=abs(geometryRange-radarRange)<=opts.radarAssociationRangeToleranceM & ...
        abs(wrap_angle(geometryAz-radarAz))<=opts.radarAssociationAzimuthDeg;
    if ~any(candidate),continue;end
    candidatePoints=geometry(candidate,:);candidateSensors=geometrySensor(candidate);
    horizontal=sqrt((candidatePoints(:,1)-radar(1)).^2+(candidatePoints(:,2)-radar(2)).^2);
    local=horizontal<=opts.radarAssociationLocalRadiusM;
    if nnz(local)<3
        [~,order]=sort(horizontal);local=false(size(horizontal));local(order(1:min(30,numel(order))))=true;
    end
    center=median(candidatePoints(local,:),1);delta=radar-center;
    item=struct('radar_centroid',radar,'geometry_centroid',center, ...
        'geometry_sensor',mode(candidateSensors(local)),'cluster_index',index,'point_count',clusters(index).n, ...
        'geometry_point_count',nnz(local),'radial_error_m',norm(radar)-norm(center), ...
        'azimuth_error_deg',wrap_angle(atan2d(radar(1),radar(2))-atan2d(center(1),center(2))), ...
        'vertical_error_m',delta(3),'error_norm_m',norm(delta), ...
        'mean_intensity_db',clusters(index).meanInt,'mean_doppler_mps',clusters(index).meanVel);
    output(end+1)=item; %#ok<AGROW>
end
end

function angle=wrap_angle(angle)
angle=mod(angle+180,360)-180;
end

function camera = to_camera(radar,E)
camera = (double(E.cameraToRadarR).' * (double(radar)-reshape(double(E.cameraOriginInRadarM),1,3)).').';
end

function [keep,uv] = in_fov(camera,fx,fy,cx,cy,imageSize,minDepth)
z=camera(:,3); u=fx*camera(:,1)./max(z,eps)+cx+1; v=fy*camera(:,2)./max(z,eps)+cy+1;
keep=z>minDepth & u>=1 & u<=imageSize(2) & v>=1 & v<=imageSize(1); uv=[u v];
end

function idx = sample_indices(indices,maxCount)
indices=indices(:); if numel(indices)<=maxCount,idx=indices;return;end
idx=indices(unique(round(linspace(1,numel(indices),maxCount))));
end

function associations = associate_radar(radar,radarCam,geometry,geometrySensor,intensity,doppler,opts)
associations=struct('radar_xyz',radar,'radar_xyz_cam',radarCam,'intensity',intensity,'doppler_v',doppler, ...
    'geometry_xyz',nan(size(radar)),'geometry_sensor',zeros(size(radar,1),1,'uint8'), ...
    'geometry_distance_m',nan(size(radar,1),1));
for index=1:size(radar,1)
    delta=geometry-radar(index,:);[distance,nearest]=min(sqrt(sum(delta.^2,2)));
    if distance<=opts.maxNearestDistanceM
        associations.geometry_xyz(index,:)=geometry(nearest,:); associations.geometry_sensor(index)=geometrySensor(nearest); associations.geometry_distance_m(index)=distance;
    end
end
end

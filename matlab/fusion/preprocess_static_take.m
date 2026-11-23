function data = preprocess_static_take(takeDir, opts)
%PREPROCESS_STATIC_TAKE Build temporally enhanced static Depth/LiDAR/Radar clouds.
if nargin < 2, opts = struct(); end
here = fileparts(mfilename('fullpath')); matlabDir = fileparts(here);
addpath(matlabDir, '-begin'); addpath(fullfile(matlabDir, 'lidar'), '-begin');
addpath(fullfile(matlabDir, 'algorithm'), '-begin');
o = fusion_defaults(opts, matlabDir);
meta = jsondecode(fileread(fullfile(takeDir, 'meta.json')));
cameraExtrinsics = load_radar_camera_extrinsics(o.cameraExtrinsicsFile);
lidarExtrinsics = load_lidar_radar_extrinsics(o.lidarExtrinsicsFile);
[fx, fy, cx, cy, depthScale, width, height] = camera_parameters(meta);

files = dir(fullfile(takeDir, 'depth', '*.png'));
if isempty(files), error('take 没有深度 PNG。'); end
[~, order] = sort({files.name}); files = files(order);
indices = unique(round(linspace(1, numel(files), min(numel(files), o.nDepthFrames)))).';
stack = nan(height, width, numel(indices));
for i = 1:numel(indices)
    image = imread(fullfile(files(indices(i)).folder, files(indices(i)).name));
    if size(image,3) > 1, image = image(:,:,1); end
    image = double(fliplr(image)) * depthScale;
    image(image < o.depthRangeM(1) | image > o.depthRangeM(2)) = NaN;
    stack(:,:,i) = image;
end
depthImage = median(stack, 3, 'omitnan');
[U,V] = meshgrid(0:width-1, 0:height-1); valid = isfinite(depthImage);
Z = depthImage; X = (U - ((width-1)-cx)) .* Z / fx; Y = (V-cy) .* Z / fy;
cameraPoints = [X(valid), Y(valid), Z(valid)]; uv = [U(valid), V(valid)];
pixel = sub2ind(size(depthImage), uv(:,2)+1, uv(:,1)+1);
local = fill_missing(depthImage); local = conv2(local, ones(3)/9, 'same');
keep = abs(cameraPoints(:,3)-local(pixel)) <= max(0.025, 0.015*cameraPoints(:,3));
cameraPoints = cameraPoints(keep,:); uv = uv(keep,:);
depthRadar = transform_points(cameraPoints, cameraExtrinsics.cameraToRadarR, cameraExtrinsics.cameraOriginInRadarM);
depthRgb = sample_color(takeDir, meta, uv, width, height);

lidar = accumulate_lidar(takeDir, lidarExtrinsics, o);
radar = accumulate_radar(takeDir, o);
data = struct('takeDir',takeDir,'takeName',take_name(takeDir),'meta',meta, ...
    'cameraExtrinsics',cameraExtrinsics,'lidarExtrinsics',lidarExtrinsics, ...
    'depth',struct('camera',cameraPoints,'radar',depthRadar,'uv',uv,'rgb',depthRgb, ...
    'depthImage',depthImage,'intrinsics',[fx fy cx cy],'depthScale',depthScale,'frameIndices',indices), ...
    'lidar',lidar,'radar',radar,'config',o);
data.summary = struct('depthPoints',size(depthRadar,1),'lidarPoints',size(lidar.radar,1), ...
    'radarPoints',size(radar.xyz,1),'depthFrames',numel(indices),'lidarBatches',lidar.batchCount, ...
    'radarFrames',radar.frameCount);
end

function [fx,fy,cx,cy,scale,width,height] = camera_parameters(meta)
fx=570.3422;fy=fx;cx=319.5;cy=239.5;scale=.001;width=640;height=480;
if isfield(meta,'camera')&&isfield(meta.camera,'depthIntrinsics')
    k=meta.camera.depthIntrinsics;if isfield(k,'fx'),fx=double(k.fx);end;if isfield(k,'fy'),fy=double(k.fy);end
    if isfield(k,'cx'),cx=double(k.cx);end;if isfield(k,'cy'),cy=double(k.cy);end
    if isfield(k,'width'),width=double(k.width);end;if isfield(k,'height'),height=double(k.height);end
end
if isfield(meta,'depth')&&isfield(meta.depth,'depthScaleMetersPerUnit'),scale=double(meta.depth.depthScaleMetersPerUnit);end
end

function image = fill_missing(image)
valid=isfinite(image);image(~valid)=0;weight=conv2(double(valid),ones(3),'same');image=conv2(image,ones(3),'same')./max(weight,1);image(weight==0)=NaN;
end

function rgb = sample_color(takeDir,meta,uv,width,height)
rgb=repmat(uint8([128 128 128]),size(uv,1),1);if isempty(uv)||~isfield(meta,'color')||~meta.color.available,return;end
files=dir(fullfile(takeDir,'color','*.png'));if isempty(files),return;end;[~,order]=sort({files.name});file=files(order(max(1,round(numel(files)/2))));image=imread(fullfile(file.folder,file.name));
if size(image,1)~=height||size(image,2)~=width,image=image(round(linspace(1,size(image,1),height)),round(linspace(1,size(image,2),width)),:);end
index=sub2ind([height width],min(height,uv(:,2)+1),min(width,uv(:,1)+1));rgb=[image(index),image(index+height*width),image(index+2*height*width)];
end

function lidar = accumulate_lidar(takeDir,E,o)
lidar=struct('native',zeros(0,3),'radar',zeros(0,3),'intensity',zeros(0,1),'batchCount',0,'timeRangeMs',[NaN NaN]);path=fullfile(takeDir,'lidar_frames.csv');if ~isfile(path),return;end
rows=readtable(path,'TextType','string');time=(double(rows.first_t_rel_ms)+double(rows.last_t_rel_ms))/2;center=median(time);selected=abs(time-center)<=o.lidarWindowMs/2;native=zeros(0,3);intensity=zeros(0,1);
for i=find(selected(:)).',file=fullfile(takeDir,strrep(char(rows.file(i)),'/',filesep));batch=read_livox_bin(file);native=[native;double(batch.points)];intensity=[intensity;double(batch.reflectivity(:))];end
[native,intensity]=voxel_reduce_points(native,intensity,o.lidarVoxelM);radar=transform_points(native,E.lidarToRadarR,E.lidarOriginInRadarM);lidar.native=native;lidar.radar=radar;lidar.intensity=intensity;lidar.batchCount=nnz(selected);if any(selected),lidar.timeRangeMs=[min(time(selected)) max(time(selected))];end
end

function radar = accumulate_radar(takeDir,o)
radar=struct('xyz',zeros(0,3),'intensity',zeros(0,1),'doppler_v',zeros(0,1), ...
    'range',zeros(0,1),'frameCount',0,'sourceFrames',zeros(0,1), ...
    'persistenceCount',zeros(0,1),'clusterLabels',zeros(0,1),'clusters',struct([]));
[~,name]=fileparts(takeDir);file=fullfile(o.resultsDir,[name '_res.mat']);if ~isfile(file),return;end
loaded=load(file,'pts','frames');allPoints=zeros(0,3);attrs=zeros(0,3);frameIds=zeros(0,1);
for i=1:numel(loaded.pts),P=double(loaded.pts{i});if isempty(P),continue;end;keep=abs(P(:,6))<=o.radarMaxDopplerMps;allPoints=[allPoints;P(keep,1:3)];attrs=[attrs;P(keep,[4 6 5])];frameIds=[frameIds;repmat(double(loaded.frames(i)),nnz(keep),1)];end
radar.frameCount=numel(loaded.pts);if isempty(allPoints),return;end;voxel=floor(allPoints./o.radarVoxelM);[~,~,group]=unique(voxel,'rows');groups=unique(group);minFrames=max(o.radarMinFrames,ceil(o.radarPersistenceRatio*radar.frameCount));selected=groups(arrayfun(@(g)numel(unique(frameIds(group==g)))>=minFrames,groups));
xyz=zeros(numel(selected),3);out=zeros(numel(selected),3);source=zeros(numel(selected),1);persistence=zeros(numel(selected),1);
for i=1:numel(selected),m=group==selected(i);xyz(i,:)=mean(allPoints(m,:),1);out(i,:)=mean(attrs(m,:),1);source(i)=median(frameIds(m));persistence(i)=numel(unique(frameIds(m)));end
radar.xyz=xyz;radar.intensity=out(:,1);radar.doppler_v=out(:,2);radar.range=out(:,3);radar.sourceFrames=source;radar.persistenceCount=persistence;
if ~isempty(xyz)
    clusterInput=[xyz,out(:,1),out(:,3),out(:,2)];
    [radar.clusterLabels,radar.clusters]=cluster_points(clusterInput,o.radarClusterEpsM,o.radarClusterMinPoints);
end
end
function name=take_name(path),[~,name]=fileparts(path);end

function E = load_radar_camera_extrinsics(path)
%LOAD_RADAR_CAMERA_EXTRINSICS 读取“深度相机坐标 -> 雷达坐标”的刚体外参。

if nargin < 1 || isempty(path)
    path = fullfile(fileparts(mfilename('fullpath')), 'radar_camera_extrinsics.json');
end
if ~isfile(path)
    error('找不到雷达/相机外参文件：%s', path);
end

E = jsondecode(fileread(path));
required = {'cameraToRadarR', 'cameraOriginInRadarM'};
for i = 1:numel(required)
    if ~isfield(E, required{i})
        error('外参文件缺少字段 %s：%s', required{i}, path);
    end
end

E.cameraToRadarR = double(E.cameraToRadarR);
E.cameraOriginInRadarM = reshape(double(E.cameraOriginInRadarM), 1, 3);
validateattributes(E.cameraToRadarR, {'numeric'}, {'size', [3 3], 'finite'});
validateattributes(E.cameraOriginInRadarM, {'numeric'}, {'size', [1 3], 'finite'});

orthogonalityError = norm(E.cameraToRadarR * E.cameraToRadarR.' - eye(3), 'fro');
if orthogonalityError > 1e-6 || abs(det(E.cameraToRadarR) - 1) > 1e-6
    error('cameraToRadarR 不是合法旋转矩阵：正交误差 %.3g，det %.6f', ...
        orthogonalityError, det(E.cameraToRadarR));
end

E.path = path;
if ~isfield(E, 'status'), E.status = 'unspecified'; end
if ~isfield(E, 'note'), E.note = ''; end
if ~isfield(E, 'radarRangeBiasM'), E.radarRangeBiasM = 0; end
validateattributes(E.radarRangeBiasM, {'numeric'}, {'scalar', 'finite', 'nonnegative'});
if ~isfield(E, 'applicable'), E.applicable = true; end
end

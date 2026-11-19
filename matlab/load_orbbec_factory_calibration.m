function calibration = load_orbbec_factory_calibration(path)
%LOAD_ORBBEC_FACTORY_CALIBRATION Load validated RGB/depth factory calibration.

% The device JSON stores color/right -> depth/left as
% P_depth = colorToDepthR * P_color + colorToDepthT.

if nargin < 1 || isempty(path)
    here = fileparts(mfilename('fullpath'));
    path = fullfile(here, '..', 'capture', 'app', 'orbbec_factory_calibration.json');
end
path = char(path);
if ~isfile(path)
    error('找不到 Orbbec 工厂标定：%s。请连接相机后运行 dump_orbbec_factory_calibration。', path);
end

raw = jsondecode(fileread(path));
if ~isfield(raw, 'schema') || ~strcmp(char(raw.schema), 'orbbec-openni2-factory-calibration-v1')
    error('不支持的 Orbbec 标定格式：%s', path);
end
required = {'colorRight', 'depthLeft', 'colorToDepth'};
for index = 1:numel(required)
    if ~isfield(raw, required{index}), error('Orbbec 标定缺少 %s：%s', required{index}, path); end
end

calibration = struct();
calibration.path = path;
calibration.device = raw.device;
calibration.colorK = intrinsics(raw.colorRight, 'colorRight');
calibration.depthK = intrinsics(raw.depthLeft, 'depthLeft');
calibration.colorDistortion = distortion(raw.colorRight, 'colorRight');
calibration.depthDistortion = distortion(raw.depthLeft, 'depthLeft');

rotationValues = double(raw.colorToDepth.rotationRowMajor(:));
if numel(rotationValues) ~= 9, error('colorToDepth.rotationRowMajor 必须有 9 个值'); end
calibration.colorToDepthR = reshape(rotationValues, 3, 3).';
validate_rotation(calibration.colorToDepthR, 'colorToDepthR');

if isfield(raw.colorToDepth, 'translationMeters')
    translation = double(raw.colorToDepth.translationMeters(:)).';
elseif isfield(raw.colorToDepth, 'translationUnit') && strcmpi(char(raw.colorToDepth.translationUnit), 'mm')
    translation = double(raw.colorToDepth.translation(:)).' * 0.001;
elseif isfield(raw, 'schema') && strcmp(char(raw.schema), 'orbbec-openni2-factory-calibration-v1')
    translation = double(raw.colorToDepth.translation(:)).' * 0.001;
else
    error('colorToDepth 平移缺少明确单位');
end
validateattributes(translation, {'numeric'}, {'size', [1 3], 'finite'});
if norm(translation) > 0.25
    error('colorToDepth 平移 %.3f m 不合理，疑似单位错误', norm(translation));
end
calibration.colorToDepthT = translation;
calibration.depthToColorR = calibration.colorToDepthR.';
calibration.depthToColorT = -(calibration.colorToDepthR.' * translation.').';
end

function K = intrinsics(value, name)
if ~isfield(value, 'intrinsicsFxFyCxCy'), error('%s 缺少内参', name); end
parameters = double(value.intrinsicsFxFyCxCy(:)).';
validateattributes(parameters, {'numeric'}, {'size', [1 4], 'finite'});
if any(parameters(1:2) <= 10), error('%s 焦距不合理', name); end
K = struct('fx', parameters(1), 'fy', parameters(2), ...
    'cx', parameters(3), 'cy', parameters(4));
end

function values = distortion(value, name)
if ~isfield(value, 'distortionOpenCvK1K2P1P2K3')
    error('%s 缺少 OpenCV 畸变参数', name);
end
values = double(value.distortionOpenCvK1K2P1P2K3(:)).';
validateattributes(values, {'numeric'}, {'size', [1 5], 'finite'});
end

function validate_rotation(rotation, name)
orthogonalityError = norm(rotation * rotation.' - eye(3), 'fro');
if orthogonalityError > 1e-4 || abs(det(rotation) - 1) > 1e-4
    error('%s 不是合法旋转矩阵：正交误差 %.3g，det %.6f', ...
        name, orthogonalityError, det(rotation));
end
end

function C = load_radar_measurement_correction(path)
%LOAD_RADAR_MEASUREMENT_CORRECTION Load radar range/angle bias estimates.
%
% Bias convention:
%   correctedRange = measuredRange - rangeBiasM
%   correctedAz    = measuredAz    - azimuthBiasDeg
%   correctedEl    = measuredEl    - elevationBiasDeg

if nargin < 1 || isempty(path)
    path = fullfile(fileparts(mfilename('fullpath')), ...
        'radar_measurement_correction.json');
end
if ~isfile(path)
    error('找不到雷达测量补偿文件：%s', path);
end

C = jsondecode(fileread(path));
required = {'rangeBiasM', 'azimuthBiasDeg', 'elevationBiasDeg'};
for i = 1:numel(required)
    if ~isfield(C, required{i})
        error('雷达补偿文件缺少字段 %s：%s', required{i}, path);
    end
    validateattributes(C.(required{i}), {'numeric'}, {'scalar', 'finite'});
    C.(required{i}) = double(C.(required{i}));
end
if C.rangeBiasM < 0
    error('rangeBiasM 必须为非负测量偏差：%s', path);
end
if ~isfield(C, 'applicable'), C.applicable = false; end
if ~isfield(C, 'status'), C.status = 'unspecified'; end
if ~isfield(C, 'note'), C.note = ''; end
C.applicable = logical(C.applicable);
C.path = path;
end

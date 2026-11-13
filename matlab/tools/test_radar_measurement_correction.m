function result = test_radar_measurement_correction()
%TEST_RADAR_MEASUREMENT_CORRECTION Validate correction config and convention.
here = fileparts(mfilename('fullpath'));
matlabDir = fileparts(here);
addpath(fullfile(matlabDir, 'algorithm'), '-begin');
addpath(fullfile(matlabDir, 'algorithm', 'lib'), '-begin');
C = load_radar_measurement_correction(fullfile(matlabDir, ...
    'radar_measurement_correction.json'));
E = load_radar_camera_extrinsics(fullfile(matlabDir, ...
    'radar_camera_extrinsics.json'));

measured = [5.149477, 3.2096, 1.9813];
corrected = measured - [C.rangeBiasM, C.azimuthBiasDeg, C.elevationBiasDeg];
expectedR = [1 0 0; 0 0 1; 0 -1 0];

result = struct( ...
    'applicable', C.applicable, ...
    'conventionPass', max(abs(corrected - [5 0 0])) < 1e-9, ...
    'formalExtrinsicsUnchanged', max(abs(E.cameraToRadarR(:)-expectedR(:))) < 1e-12, ...
    'status', C.status);
result.rawFramePass = true;
takeDir = 'E:\leida\11-5\20261105_152723_室外3米';
if isfile(fullfile(takeDir, 'radar_adc.mat'))
    raw = first_complete_frame(fullfile(takeDir, 'radar_adc.mat'));
    baseCfg = struct('xLim', [-10 10], 'yLim', [0 12], 'zLim', [-10 10]);
    [~, rawInfo] = process_final(raw, baseCfg);
    correctedCfg = baseCfg;
    correctedCfg.rangeOffsetM = -C.rangeBiasM;
    correctedCfg.azimuthOffsetDeg = -C.azimuthBiasDeg;
    correctedCfg.elevationOffsetDeg = -C.elevationBiasDeg;
    [~, correctedInfo] = process_final(raw, correctedCfg);
    result.rawFramePass = ...
        max(abs((correctedInfo.candAz-rawInfo.candAz) + C.azimuthBiasDeg)) < 1e-9 && ...
        max(abs((correctedInfo.candEl-rawInfo.candEl) + C.elevationBiasDeg)) < 1e-9 && ...
        abs(correctedInfo.rangeOffsetM + C.rangeBiasM) < 1e-12;
end
result.pass = result.applicable && result.conventionPass && ...
    result.formalExtrinsicsUnchanged && result.rawFramePass;
fprintf('雷达补偿配置：range %.6f m, az %.4f deg, el %.4f deg, %s\n', ...
    C.rangeBiasM, C.azimuthBiasDeg, C.elevationBiasDeg, pass_text(result.pass));
if ~result.pass, error('雷达测量补偿配置测试未通过。'); end
end

function raw = first_complete_frame(matFile)
S = load(matFile, 'ADCData');
adt = S.ADCData;
per = 2048;
payloadBytes = numel(adt{1, 4});
ids = zeros(size(adt, 1), 1);
for i = 1:size(adt, 1)
    v = adt{i, 2};
    if iscell(v) && numel(v) == 1, v = v{1}; end
    ids(i) = double(v(1));
end
[sortedIds, sortIdx] = sort(ids);
fidx = floor(sortedIds / per) + 1;
[uf, ~, grp] = unique(fidx);
cnt = accumarray(grp, 1);
frame = uf(find(cnt == per, 1, 'first'));
lo = (frame - 1) * per;
p1 = find(sortedIds >= lo, 1, 'first');
raw = zeros(payloadBytes, per, 'uint8');
for i = 1:per
    raw(:, i) = uint8(adt{sortIdx(p1+i-1), 4}(:));
end
end

function s = pass_text(v)
if v, s = 'PASS'; else, s = 'FAIL'; end
end

function report = validate_outdoor_radar(rootDir, outputDir, opts)
%VALIDATE_OUTDOOR_RADAR Validate multi-distance outdoor radar captures.
%
% report = validate_outdoor_radar('E:\leida\11-5')
%
% The formal radar-camera extrinsics are used only for RGB overlays. Radar
% range estimates are kept raw; the rejected candidate range bias is never
% applied to the measurements.

if nargin < 1 || isempty(rootDir), rootDir = 'E:\leida\11-5'; end
here = fileparts(mfilename('fullpath'));
matlabDir = fileparts(here);
if nargin < 2 || isempty(outputDir)
    outputDir = fullfile(fileparts(matlabDir), 'results', '11-5_validation');
end
if nargin < 3, opts = struct(); end
opts = set_defaults(opts);

addpath(fullfile(matlabDir, 'algorithm'), '-begin');
addpath(fullfile(matlabDir, 'algorithm', 'lib'), '-begin');
if ~isfolder(outputDir), mkdir(outputDir); end

E = load_radar_camera_extrinsics(fullfile(matlabDir, 'radar_camera_extrinsics.json'));
takes = dir(rootDir);
takes = takes([takes.isdir] & ~ismember({takes.name}, {'.', '..'}));
expected = arrayfun(@(d) parse_distance(d.name), takes);
takes = takes(isfinite(expected));
expected = expected(isfinite(expected));
[expected, order] = sort(expected);
takes = takes(order);
if isempty(takes), error('No distance-labelled take directories found in %s', rootDir); end

n = numel(takes);
rows = repmat(empty_row(), n, 1);
profiles = cell(n, 1);
profileRanges = cell(n, 1);
overlayFiles = strings(n, 1);

fprintf('\nOutdoor radar validation: %s\n', rootDir);
for i = 1:n
    takeDir = fullfile(takes(i).folder, takes(i).name);
    fprintf('\n[%d/%d] %s\n', i, n, takes(i).name);
    meta = jsondecode(fileread(fullfile(takeDir, 'meta.json')));
    S = load(fullfile(takeDir, 'radar_adc.mat'), 'ADCData');
    adt = S.ADCData;
    [sortedIds, sortIdx, done, payloadBytes] = packet_index(adt, opts.packetsPerFrame);
    if isempty(done), error('%s has no complete radar frames', takes(i).name); end
    pick = unique(done(round(linspace(1, numel(done), min(opts.framesPerTake, numel(done))))));

    framePeaks = nan(numel(pick), 1);
    frameProminence = nan(numel(pick), 1);
    frameProfiles = [];
    targetPoints = zeros(0, 6);
    allPointCount = zeros(numel(pick), 1);

    for k = 1:numel(pick)
        raw = assemble(adt, sortedIds, sortIdx, pick(k), opts.packetsPerFrame, payloadBytes);
        [rangeM, profile] = static_range_profile(raw, opts);
        if isempty(frameProfiles), frameProfiles = nan(numel(profile), numel(pick)); end
        frameProfiles(:, k) = profile;
        [framePeaks(k), frameProminence(k)] = target_peak(rangeM, profile, expected(i), opts);

        cfg = struct('detectMode', '8T', 'relativeGateRatio', 0.5, ...
                     'rangeOffsetM', 0, 'xLim', [-5 5], 'yLim', [0.5 9], ...
                     'zLim', [-2 2]);
        try
            P = process_final(raw, cfg);
        catch ex
            warning('process_final failed for frame %d: %s', pick(k), ex.message);
            P = zeros(0, 6);
        end
        allPointCount(k) = size(P, 1);
        if ~isempty(P) && isfinite(framePeaks(k))
            keep = abs(P(:, 5) - framePeaks(k)) <= opts.pointRangeHalfWidthM;
            targetPoints = [targetPoints; P(keep, :)]; %#ok<AGROW>
        end
    end

    aggProfile = median(frameProfiles, 2, 'omitnan');
    [peakM, prominenceDb] = target_peak(rangeM, aggProfile, expected(i), opts);
    profiles{i} = aggProfile;
    profileRanges{i} = rangeM;

    [boardCenter, boardSize, colorFile] = find_board_image(takeDir, opts.colorFrame);
    expectedPoint = board_ray_to_radar(boardCenter, peakM, meta, E);
    [bestPoint, angularErrorDeg] = closest_angular_point(targetPoints, expectedPoint);
    overlayFile = fullfile(outputDir, sprintf('%dm_radar_rgb_overlay.png', round(expected(i))));
    make_overlay(colorFile, targetPoints, boardCenter, E, meta, overlayFile);
    overlayFiles(i) = string(overlayFile);

    r = empty_row();
    r.take = string(takes(i).name);
    r.nominalM = expected(i);
    r.completeFrames = numel(done);
    r.selectedFrames = numel(pick);
    r.lossRate = meta.radar.lossRate;
    r.lossRateExcludingTail = meta.radar.lossRateExcludingTail;
    r.peakM = peakM;
    r.errorM = peakM - expected(i);
    r.framePeakMedianM = median(framePeaks, 'omitnan');
    r.framePeakMadM = median(abs(framePeaks - r.framePeakMedianM), 'omitnan');
    r.prominenceDb = prominenceDb;
    r.frameProminenceMedianDb = median(frameProminence, 'omitnan');
    r.pointsPerFrameMedian = median(allPointCount, 'omitnan');
    r.targetPointCount = size(targetPoints, 1);
    r.boardDetected = all(isfinite(boardCenter));
    r.boardCenterU = boardCenter(1);
    r.boardCenterV = boardCenter(2);
    r.boardRows = boardSize(1);
    r.boardCols = boardSize(2);
    r.expectedX = expectedPoint(1);
    r.expectedY = expectedPoint(2);
    r.expectedZ = expectedPoint(3);
    r.bestPointX = bestPoint(1);
    r.bestPointY = bestPoint(2);
    r.bestPointZ = bestPoint(3);
    [r.expectedAzDeg, r.expectedElDeg] = point_angles(expectedPoint);
    [r.bestAzDeg, r.bestElDeg] = point_angles(bestPoint);
    r.azErrorDeg = r.bestAzDeg - r.expectedAzDeg;
    r.elErrorDeg = r.bestElDeg - r.expectedElDeg;
    r.angularErrorDeg = angularErrorDeg;
    r.rangeStatus = string(range_status(r.errorM, r.framePeakMadM, prominenceDb, opts));
    r.projectionStatus = string(projection_status(angularErrorDeg));
    rows(i) = r;
    fprintf('  peak %.3f m, error %+.3f m, frame MAD %.3f m, prominence %.1f dB, %s\n', ...
        peakM, r.errorM, r.framePeakMadM, prominenceDb, r.rangeStatus);
end

T = struct2table(rows);
fitOk = isfinite(T.nominalM) & isfinite(T.peakM);
p = polyfit(T.nominalM(fitOk), T.peakM(fitOk), 1);
pred = polyval(p, T.nominalM(fitOk));
rmse = sqrt(mean((T.peakM(fitOk) - pred).^2));
offsetOnly = median(T.peakM(fitOk) - T.nominalM(fitOk));
offsetResidual = T.peakM(fitOk) - (T.nominalM(fitOk) + offsetOnly);
offsetRmse = sqrt(mean(offsetResidual.^2));

% Convert angular offsets into approximate Cartesian errors at the taped
% distance. These are the quantities relevant to coordinate alignment.
T.rangeCorrectedM = T.peakM - offsetOnly;
T.correctedRangeErrorM = T.rangeCorrectedM - T.nominalM;
T.horizontalErrorM = T.nominalM .* tand(T.azErrorDeg);
T.verticalErrorM = T.nominalM .* tand(T.elErrorDeg);
T.totalErrorRawM = sqrt(T.errorM.^2 + T.horizontalErrorM.^2 + T.verticalErrorM.^2);
T.totalErrorRangeCorrectedM = sqrt(T.correctedRangeErrorM.^2 + ...
    T.horizontalErrorM.^2 + T.verticalErrorM.^2);
T.alignmentStatus = repmat("FAIL", height(T), 1);
T.alignmentStatus(T.totalErrorRawM <= 0.15) = "PASS";
writetable(T, fullfile(outputDir, 'radar_validation.csv'));

rangeProfileFile = fullfile(outputDir, 'range_profiles.png');
distanceFitFile = fullfile(outputDir, 'distance_fit.png');
plot_profiles(profileRanges, profiles, T, rangeProfileFile);
plot_fit(T, p, offsetOnly, distanceFitFile);

report = struct('rootDir', rootDir, 'outputDir', outputDir, 'table', T, ...
    'linearSlope', p(1), 'linearInterceptM', p(2), 'linearRmseM', rmse, ...
    'offsetOnlyM', offsetOnly, 'offsetOnlyRmseM', offsetRmse, ...
    'formalExtrinsics', E, 'overlayFiles', overlayFiles, ...
    'rangeProfileFile', rangeProfileFile, 'distanceFitFile', distanceFitFile);
save(fullfile(outputDir, 'radar_validation.mat'), 'report', '-v7.3');
write_markdown_report(report, fullfile(outputDir, '验证报告.md'));

fprintf('\nFit: radar = %.5f * nominal %+.5f m, RMSE %.3f m\n', p(1), p(2), rmse);
fprintf('Offset-only: %+.3f m, RMSE %.3f m\n', offsetOnly, offsetRmse);
fprintf('Report: %s\n', fullfile(outputDir, '验证报告.md'));
end

function o = set_defaults(o)
d = struct('framesPerTake', 9, 'packetsPerFrame', 2048, ...
    'targetSearchHalfWidthM', 0.65, 'pointRangeHalfWidthM', 0.18, ...
    'rangePassErrorM', 0.20, 'rangeWarnErrorM', 0.35, ...
    'maxStableMadM', 0.08, 'minProminenceDb', 3.0, 'colorFrame', 300, ...
    'nfftR', 1024, 'staticDopplerHalfWidth', 4);
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(o, f{i}), o.(f{i}) = d.(f{i}); end
end
end

function r = empty_row()
r = struct('take', "", 'nominalM', NaN, 'completeFrames', NaN, ...
    'selectedFrames', NaN, 'lossRate', NaN, 'lossRateExcludingTail', NaN, ...
    'peakM', NaN, 'errorM', NaN, 'framePeakMedianM', NaN, ...
    'framePeakMadM', NaN, 'prominenceDb', NaN, ...
    'frameProminenceMedianDb', NaN, 'pointsPerFrameMedian', NaN, ...
    'targetPointCount', NaN, 'boardDetected', false, 'boardCenterU', NaN, ...
    'boardCenterV', NaN, 'boardRows', NaN, 'boardCols', NaN, ...
    'expectedX', NaN, 'expectedY', NaN, 'expectedZ', NaN, ...
    'bestPointX', NaN, 'bestPointY', NaN, 'bestPointZ', NaN, ...
    'expectedAzDeg', NaN, 'expectedElDeg', NaN, 'bestAzDeg', NaN, ...
    'bestElDeg', NaN, 'azErrorDeg', NaN, 'elErrorDeg', NaN, ...
    'angularErrorDeg', NaN, 'rangeStatus', "", 'projectionStatus', "");
end

function d = parse_distance(name)
d = NaN;
tok = regexp(name, '室外\s*(\d+(?:\.\d+)?)\s*米', 'tokens', 'once');
if ~isempty(tok), d = str2double(tok{1}); end
end

function [sortedIds, sortIdx, done, payloadBytes] = packet_index(adt, per)
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
done = sort(uf(cnt == per));
end

function raw = assemble(adt, sortedIds, sortIdx, frameNumber, per, payloadBytes)
lo = (frameNumber - 1) * per;
p1 = find(sortedIds >= lo, 1, 'first');
raw = zeros(payloadBytes, per, 'uint8');
for i = 1:per
    raw(:, i) = uint8(adt{sortIdx(p1 + i - 1), 4}(:));
end
end

function [rangeM, profile] = static_range_profile(raw, o)
Nadc = 1024; Nrx = 8; Ntx = 8; nChirp = 16; Ndop = 128;
raw = reshape(uint8(raw), 2048, 2048);
adc = double(uint16(raw(1:2:end, :))) + 256 * double(uint16(raw(2:2:end, :))) - 2048;
wR = single(hamming(Nadc));
wD = single(hamming(nChirp)).';
adc8 = reshape(adc(:, 1025:2048), Nadc, Ntx, nChirp, Nrx);
adc8 = permute(adc8, [1, 4, 2, 3]);
adc8 = reshape(adc8, Nadc, Nrx * Ntx, nChirp);
rangeFft = fft(single(adc8) .* reshape(wR, Nadc, 1, 1), o.nfftR, 1);
rangeFft = rangeFft(1:o.nfftR/2, :, :);
rd = fft(rangeFft .* reshape(wD, 1, 1, nChirp), Ndop, 3);
rd = rd(:, :, [65:128, 1:64]);
RD = sum(abs(permute(rd, [1, 3, 2])), 3);
dop = (65-o.staticDopplerHalfWidth):(65+o.staticDopplerHalfWidth);
profile = median(double(RD(:, dop)), 2);
c = 3e8; Fs = 16.667e6; bandwidth = 4e9; rampTime = 60e-6;
dR = (c / (2 * (bandwidth / rampTime))) * (Fs / o.nfftR);
rangeM = (0:numel(profile)-1).' * dR;
end

function [peakM, prominenceDb] = target_peak(rangeM, profile, nominalM, o)
window = rangeM >= nominalM-o.targetSearchHalfWidthM & ...
         rangeM <= nominalM+o.targetSearchHalfWidthM;
peakM = NaN; prominenceDb = NaN;
if ~any(window) || ~any(isfinite(profile(window))), return; end
idx = find(window);
[peakValue, local] = max(profile(idx));
peakIdx = idx(local);
peakM = rangeM(peakIdx);
baseline = median(profile(idx), 'omitnan');
prominenceDb = 20 * log10(max(peakValue, eps) / max(baseline, eps));
end

function [center, boardSize, colorFile] = find_board_image(takeDir, frameNo)
center = [NaN NaN]; boardSize = [NaN NaN];
colorFile = fullfile(takeDir, 'color', sprintf('%06d.png', frameNo));
if ~isfile(colorFile)
    files = dir(fullfile(takeDir, 'color', '*.png'));
    if isempty(files), return; end
    colorFile = fullfile(files(round(numel(files)/2)).folder, files(round(numel(files)/2)).name);
end
try
    I = imread(colorFile);
    [points, boardSize] = detectCheckerboardPoints(I);
    if ~isempty(points), center = mean(points, 1); end
catch ex
    warning('Checkerboard detection failed for %s: %s', colorFile, ex.message);
end
end

function pRadar = board_ray_to_radar(center, rangeM, meta, E)
pRadar = [NaN NaN NaN];
if any(~isfinite(center)) || ~isfinite(rangeM), return; end
K = meta.camera.depthIntrinsics;
pCamera = [(center(1)-K.cx)*rangeM/K.fx, ...
           (center(2)-K.cy)*rangeM/K.fy, rangeM];
pRadar = (E.cameraToRadarR * pCamera.').' + reshape(E.cameraOriginInRadarM, 1, 3);
end

function [best, errorDeg] = closest_angular_point(points, expectedPoint)
best = [NaN NaN NaN]; errorDeg = NaN;
if isempty(points) || any(~isfinite(expectedPoint)), return; end
u = points(:, 1:3) ./ vecnorm(points(:, 1:3), 2, 2);
ue = expectedPoint / norm(expectedPoint);
angles = acosd(max(-1, min(1, u * ue.')));
[errorDeg, idx] = min(angles);
best = points(idx, 1:3);
end

function [azDeg, elDeg] = point_angles(point)
azDeg = NaN; elDeg = NaN;
if any(~isfinite(point)) || norm(point) == 0, return; end
azDeg = atan2d(point(1), point(2));
elDeg = atan2d(point(3), hypot(point(1), point(2)));
end

function make_overlay(colorFile, points, center, E, meta, outputFile)
if ~isfile(colorFile), return; end
I = imread(colorFile);
f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 size(I,2) size(I,1)]);
imshow(I); hold on;
if ~isempty(points)
    cameraPoints = (E.cameraToRadarR.' * ...
        (points(:, 1:3) - reshape(E.cameraOriginInRadarM, 1, 3)).').';
    K = meta.camera.depthIntrinsics;
    good = cameraPoints(:, 3) > 0.1;
    u = K.fx * cameraPoints(good, 1) ./ cameraPoints(good, 3) + K.cx;
    v = K.fy * cameraPoints(good, 2) ./ cameraPoints(good, 3) + K.cy;
    in = u >= 1 & u <= size(I,2) & v >= 1 & v <= size(I,1);
    scatter(u(in), v(in), 28, 'r', 'filled', 'MarkerFaceAlpha', 0.45);
end
if all(isfinite(center)), plot(center(1), center(2), 'g+', 'MarkerSize', 18, 'LineWidth', 2); end
title('Formal-extrinsics radar projection (red); checkerboard center (green)', 'Interpreter', 'none');
exportgraphics(f, outputFile, 'Resolution', 150);
close(f);
end

function status = range_status(errorM, madM, prominenceDb, o)
if ~isfinite(errorM) || ~isfinite(madM) || ~isfinite(prominenceDb)
    status = 'FAIL';
elseif abs(errorM) <= o.rangePassErrorM && madM <= o.maxStableMadM && prominenceDb >= o.minProminenceDb
    status = 'PASS';
elseif abs(errorM) <= o.rangeWarnErrorM && madM <= 2*o.maxStableMadM
    status = 'WARN';
else
    status = 'FAIL';
end
end

function status = projection_status(errorDeg)
% RGB uses approximate intrinsics, so this is a diagnostic classification.
if ~isfinite(errorDeg)
    status = 'NO_DATA';
elseif errorDeg <= 2.0
    status = 'CONSISTENT';
else
    status = 'OFFSET';
end
end

function plot_profiles(ranges, profiles, T, outputFile)
f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1000 700]);
tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
for i = 1:height(T)
    nexttile;
    r = ranges{i}; p = profiles{i};
    db = 20*log10(max(p, eps)); db = db - max(db(r >= 1 & r <= 8));
    plot(r, db, 'k-', 'LineWidth', 1.1); hold on;
    xline(T.nominalM(i), 'b--', 'Nominal');
    xline(T.peakM(i), 'r-', sprintf('Peak %.2f m', T.peakM(i)));
    xlim([max(0.5,T.nominalM(i)-1.2), min(8.5,T.nominalM(i)+1.2)]); ylim([-35 2]);
    grid on; xlabel('Range (m)'); ylabel('Relative amplitude (dB)');
    title(sprintf('%g m, error %+.2f m, %s', T.nominalM(i), T.errorM(i), T.rangeStatus(i)));
end
exportgraphics(f, outputFile, 'Resolution', 160); close(f);
end

function plot_fit(T, p, offset, outputFile)
f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 850 620]);
x = linspace(min(T.nominalM)-0.2, max(T.nominalM)+0.2, 100);
plot(T.nominalM, T.peakM, 'ko', 'MarkerFaceColor', [0.85 0.15 0.1], 'MarkerSize', 8); hold on;
plot(x, x, 'k--', 'DisplayName', 'Ideal');
plot(x, polyval(p,x), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Linear fit');
plot(x, x+offset, 'Color', [0.1 0.55 0.2], 'LineWidth', 1.5, 'DisplayName', 'Offset-only fit');
grid on; axis equal; xlim([min(x) max(x)]); ylim([min(x) max(x)+0.5]);
xlabel('Nominal distance (m)'); ylabel('Radar peak distance (m)');
title(sprintf('Radar = %.4f x %+.3f m', p(1), p(2)));
legend('Measured', 'Ideal', 'Linear fit', 'Offset-only fit', 'Location', 'northwest');
exportgraphics(f, outputFile, 'Resolution', 160); close(f);
end

function write_markdown_report(report, outputFile)
T = report.table;
fid = fopen(outputFile, 'w', 'n', 'UTF-8');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# 11-5 室外五组雷达验证报告\n\n');
fprintf(fid, '数据目录：`%s`  \n', report.rootDir);
fprintf(fid, '验证重点：雷达原始 ADC 的距离峰、跨帧稳定性和正式外参投影。Astra Pro 室外深度不作为测距真值。\n\n');
fprintf(fid, '## 直接结论：坐标对齐未通过\n\n');
fprintf(fid, '按昨天标定说明中的 `0.15 m` 验收门限，五组当前三维坐标误差估算为 `%.3f–%.3f m`，全部 FAIL。距离轴的固定偏差可以修正，但方位和俯仰误差会随距离放大，因此当前整体坐标误差不可控。\n\n', ...
    min(T.totalErrorRawM), max(T.totalErrorRawM));
fprintf(fid, '| 真实距离 | 前后误差 Y | 左右误差 X | 高度误差 Z | 当前三维误差 | 修正固定距离偏差后 | 判定 |\n');
fprintf(fid, '|---:|---:|---:|---:|---:|---:|:---:|\n');
for i = 1:height(T)
    fprintf(fid, '| %.1f m | %+.3f m | %+.3f m | %+.3f m | %.3f m | %.3f m | %s |\n', ...
        T.nominalM(i), T.errorM(i), T.horizontalErrorM(i), T.verticalErrorM(i), ...
        T.totalErrorRawM(i), T.totalErrorRangeCorrectedM(i), T.alignmentStatus(i));
end
fprintf(fid, '\n说明：X/Z 误差由棋盘中心方向与最近雷达点的方位/俯仰差换算；由于 RGB 使用近似内参，这是一项误差估算，但足以判断是否达到 0.15 m 门限。\n\n');
fprintf(fid, '## 结论数据\n\n');
fprintf(fid, '| 标称距离 | 完整帧 | 中段丢包率 | 雷达峰 | 误差 | 跨帧 MAD | 峰显著度 | 状态 |\n');
fprintf(fid, '|---:|---:|---:|---:|---:|---:|---:|:---:|\n');
for i = 1:height(T)
    fprintf(fid, '| %.1f m | %d | %.3f%% | %.3f m | %+.3f m | %.3f m | %.1f dB | %s |\n', ...
        T.nominalM(i), T.completeFrames(i), 100*T.lossRateExcludingTail(i), ...
        T.peakM(i), T.errorM(i), T.framePeakMadM(i), T.prominenceDb(i), T.rangeStatus(i));
end
fprintf(fid, '\n## 五距离拟合\n\n');
fprintf(fid, '- 自由线性拟合：`radar = %.6f * nominal %+.6f m`，RMSE `%.3f m`。\n', ...
    report.linearSlope, report.linearInterceptM, report.linearRmseM);
fprintf(fid, '- 仅固定偏差模型：偏差 `%+.3f m`，RMSE `%.3f m`。\n', ...
    report.offsetOnlyM, report.offsetOnlyRmseM);
fprintf(fid, '- 应用固定偏差后，五组距离轴残差最大仅 `%.3f m`；距离轴可控。\n', ...
    max(abs(T.correctedRangeErrorM)));
fprintf(fid, '- 正式外参和正式 `radarRangeBiasM=0` 未修改；未通过验收的候选偏差没有应用。\n\n');
fprintf(fid, '![距离峰](range_profiles.png)\n\n');
fprintf(fid, '![距离拟合](distance_fit.png)\n\n');
fprintf(fid, '## RGB 弱验证\n\n');
fprintf(fid, '红点是正式外参投影的目标距离附近雷达点，绿十字是棋盘中心。RGB 使用的是深度流近似内参；同时 `manifest.csv` 的彩色流持续时间异常长于 20 秒雷达记录，因此只能用于弱验证，不能单独作为外参通过依据。\n\n');
fprintf(fid, '| 标称距离 | 方位误差 | 俯仰误差 | 最近角距 | 诊断 |\n');
fprintf(fid, '|---:|---:|---:|---:|:---:|\n');
for i = 1:height(T)
    fprintf(fid, '| %.1f m | %+.2f deg | %+.2f deg | %.2f deg | %s |\n', ...
        T.nominalM(i), T.azErrorDeg(i), T.elErrorDeg(i), ...
        T.angularErrorDeg(i), T.projectionStatus(i));
end
fprintf(fid, '\n五组最近角距均大于 2 deg，且投影点整体位于棋盘右上方。正式外参的空间投影未由本批数据验证通过；由于雷达测角算法本身已有已知偏差，不能把该偏差直接归因于机械外参，也不能据此应用未通过验收的候选旋转。\n\n');
for i = 1:height(T)
    fprintf(fid, '### %.0f m\n\n![%.0f m 投影](%dm_radar_rgb_overlay.png)\n\n', ...
        T.nominalM(i), T.nominalM(i), round(T.nominalM(i)));
end
fprintf(fid, '## 判定规则\n\n');
fprintf(fid, '- PASS：距离误差不超过 0.20 m、跨帧 MAD 不超过 0.08 m、峰显著度至少 3 dB。\n');
fprintf(fid, '- WARN：距离误差不超过 0.35 m 且稳定性没有明显失控。\n');
fprintf(fid, '- 以上只判定雷达测距数据质量，不等价于六自由度雷达-相机外参验收。\n');
end

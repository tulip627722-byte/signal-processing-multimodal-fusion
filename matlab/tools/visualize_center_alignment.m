function results = visualize_center_alignment(rootDir, validationCsv, outputDir)
%VISUALIZE_CENTER_ALIGNMENT Overlay checkerboard and radar target centers.
%
% results = visualize_center_alignment(rootDir, validationCsv, outputDir)
%
% The left column shows the radar center before measurement correction. The
% right column shows the same center after range/azimuth/elevation correction.
% This is an image-plane diagnostic; RGB projection uses the depth-stream
% approximate intrinsics recorded in meta.json.

here = fileparts(mfilename('fullpath'));
matlabDir = fileparts(here);
repoRoot = fileparts(matlabDir);
if nargin < 1 || isempty(rootDir)
    rootDir = uigetdir(pwd, 'Select the directory containing distance-labelled takes');
    if isequal(rootDir, 0), error('No data directory selected.'); end
end
if nargin < 2 || isempty(validationCsv)
    validationCsv = fullfile(repoRoot, 'docs', 'experiments', ...
        '2026-11-05', 'radar_validation.csv');
end
if nargin < 3 || isempty(outputDir)
    outputDir = fullfile(repoRoot, 'results', 'center_alignment');
end
if ~isfolder(outputDir), mkdir(outputDir); end

E = load_radar_camera_extrinsics(fullfile(matlabDir, 'radar_camera_extrinsics.json'));
C = load_radar_measurement_correction(fullfile(matlabDir, ...
    'radar_measurement_correction.json'));
T = readtable(validationCsv, 'TextType', 'string');
required = {'take','nominalM','bestPointX','bestPointY','bestPointZ'};
if ~all(ismember(required, T.Properties.VariableNames))
    error('Validation CSV does not contain the required target-center columns: %s', validationCsv);
end

n = height(T);
results = repmat(empty_result(), n, 1);
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [20 20 1500 max(900, 450*n)]);
tiledlayout(n, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
for i = 1:n
    takeDir = fullfile(rootDir, T.take(i));
    meta = jsondecode(fileread(fullfile(takeDir, 'meta.json')));
    [colorFile, boardPoints, boardCenter] = find_checkerboard_frame(takeDir);

    raw = [T.bestPointX(i), T.bestPointY(i), T.bestPointZ(i)];
    corrected = correct_radar_center(raw, C);
    expected = board_center_to_radar(boardCenter, T.nominalM(i), meta, E);
    rawPixel = project_radar_to_image(raw, meta, E);
    correctedPixel = project_radar_to_image(corrected, meta, E);

    rawPixelError = norm(rawPixel-boardCenter);
    correctedPixelError = norm(correctedPixel-boardCenter);
    rawAngleError = angle_between(raw, expected);
    correctedAngleError = angle_between(corrected, expected);
    raw3dError = norm(raw-expected);
    corrected3dError = norm(corrected-expected);
    image = imread(colorFile);

    draw_panel(nexttile, image, boardPoints, boardCenter, rawPixel, ...
        sprintf('%g m before | %.1f px | %.2f deg | %.3f m', ...
        T.nominalM(i), rawPixelError, rawAngleError, raw3dError), [0.95 0.25 0.12]);
    draw_panel(nexttile, image, boardPoints, boardCenter, correctedPixel, ...
        sprintf('%g m corrected | %.1f px | %.2f deg | %.3f m', ...
        T.nominalM(i), correctedPixelError, correctedAngleError, corrected3dError), ...
        [0.05 0.45 0.95]);

    results(i) = struct('take', T.take(i), 'nominalM', T.nominalM(i), ...
        'colorFile', string(colorFile), 'boardCenterPixel', boardCenter, ...
        'rawRadarCenterM', raw, 'correctedRadarCenterM', corrected, ...
        'expectedRadarCenterM', expected, 'rawProjectedPixel', rawPixel, ...
        'correctedProjectedPixel', correctedPixel, ...
        'rawPixelError', rawPixelError, 'correctedPixelError', correctedPixelError, ...
        'rawAngleErrorDeg', rawAngleError, 'correctedAngleErrorDeg', correctedAngleError, ...
        'raw3dErrorM', raw3dError, 'corrected3dErrorM', corrected3dError);
end

imageFile = fullfile(outputDir, 'center_alignment_before_after.png');
exportgraphics(fig, imageFile, 'Resolution', 150); close(fig);
save(fullfile(outputDir, 'center_alignment_before_after.mat'), 'results');
write_summary(results, fullfile(outputDir, 'center_alignment_summary.csv'));
fprintf('Center-alignment image: %s\n', imageFile);
end

function r = empty_result()
r = struct('take', "", 'nominalM', NaN, 'colorFile', "", ...
    'boardCenterPixel', nan(1, 2), 'rawRadarCenterM', nan(1, 3), ...
    'correctedRadarCenterM', nan(1, 3), 'expectedRadarCenterM', nan(1, 3), ...
    'rawProjectedPixel', nan(1, 2), 'correctedProjectedPixel', nan(1, 2), ...
    'rawPixelError', NaN, 'correctedPixelError', NaN, ...
    'rawAngleErrorDeg', NaN, 'correctedAngleErrorDeg', NaN, ...
    'raw3dErrorM', NaN, 'corrected3dErrorM', NaN);
end

function corrected = correct_radar_center(point, C)
r = norm(point);
az = atan2d(point(1), point(2));
el = atan2d(point(3), hypot(point(1), point(2)));
rc = max(r-C.rangeBiasM, 0.1);
azc = az-C.azimuthBiasDeg;
elc = el-C.elevationBiasDeg;
corrected = [rc*cosd(elc)*sind(azc), ...
    rc*cosd(elc)*cosd(azc), rc*sind(elc)];
end

function pRadar = board_center_to_radar(center, distanceM, meta, E)
K = meta.camera.depthIntrinsics;
pCamera = [(center(1)-K.cx)*distanceM/K.fx, ...
    (center(2)-K.cy)*distanceM/K.fy, distanceM];
pRadar = (E.cameraToRadarR*pCamera.').' + E.cameraOriginInRadarM;
end

function pixel = project_radar_to_image(point, meta, E)
pCamera = (E.cameraToRadarR.'*(point-E.cameraOriginInRadarM).').';
K = meta.camera.depthIntrinsics;
pixel = [K.fx*pCamera(1)/pCamera(3)+K.cx, ...
    K.fy*pCamera(2)/pCamera(3)+K.cy];
end

function deg = angle_between(a, b)
deg = acosd(max(-1, min(1, dot(a, b)/(norm(a)*norm(b)))));
end

function [colorFile, points, center] = find_checkerboard_frame(takeDir)
files = dir(fullfile(takeDir, 'color', '*.png'));
if isempty(files), error('No color frames found in %s', takeDir); end
sample = unique(round(linspace(1, numel(files), min(31, numel(files)))));
ordered = unique([round(numel(files)/2), sample], 'stable');
points = []; center = [NaN NaN]; colorFile = '';
for idx = ordered
    candidate = fullfile(files(idx).folder, files(idx).name);
    image = imread(candidate);
    [p, ~] = detectCheckerboardPoints(image);
    if ~isempty(p)
        colorFile = candidate;
        points = p;
        center = mean(p, 1);
        return;
    end
end
error('Checkerboard was not detected in sampled color frames: %s', takeDir);
end

function draw_panel(ax, image, boardPoints, boardCenter, radarPixel, titleText, radarColor)
imshow(image, 'Parent', ax); hold(ax, 'on');
plot(ax, boardPoints(:, 1), boardPoints(:, 2), '.', ...
    'Color', [0.1 0.8 0.25], 'MarkerSize', 7);
plot(ax, [boardCenter(1), radarPixel(1)], [boardCenter(2), radarPixel(2)], '-', ...
    'Color', [1 0.85 0.1], 'LineWidth', 2);
plot(ax, boardCenter(1), boardCenter(2), '+', 'Color', [0.1 1 0.2], ...
    'MarkerSize', 18, 'LineWidth', 3);
plot(ax, radarPixel(1), radarPixel(2), 'x', 'Color', radarColor, ...
    'MarkerSize', 18, 'LineWidth', 3);
text(ax, boardCenter(1)+8, boardCenter(2)+15, 'checkerboard center', ...
    'Color', [0.1 1 0.2], 'FontWeight', 'bold', 'FontSize', 10, ...
    'BackgroundColor', [0 0 0 0.55]);
text(ax, radarPixel(1)+8, radarPixel(2)-10, 'radar center', ...
    'Color', radarColor, 'FontWeight', 'bold', 'FontSize', 10, ...
    'BackgroundColor', [1 1 1 0.70]);
title(ax, titleText, 'Interpreter', 'none', 'FontSize', 11);
end

function write_summary(R, path)
take = string({R.take}).'; nominalM = [R.nominalM].';
rawPixelError = [R.rawPixelError].'; correctedPixelError = [R.correctedPixelError].';
rawAngleErrorDeg = [R.rawAngleErrorDeg].'; correctedAngleErrorDeg = [R.correctedAngleErrorDeg].';
raw3dErrorM = [R.raw3dErrorM].'; corrected3dErrorM = [R.corrected3dErrorM].';
T = table(take, nominalM, rawPixelError, correctedPixelError, ...
    rawAngleErrorDeg, correctedAngleErrorDeg, raw3dErrorM, corrected3dErrorM);
writetable(T, path);
end

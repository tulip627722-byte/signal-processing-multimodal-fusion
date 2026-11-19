function result = test_rgb_projection()
%TEST_RGB_PROJECTION Synthetic regression for radar-to-RGB projection.
meta = struct('cameraToRadarR', [1 0 0; 0 0 1; 0 -1 0], ...
    'cameraOriginInRadarM', [0 0 0]);
calibration = struct('depthToColorR', eye(3), 'depthToColorT', [0 0 0], ...
    'colorK', struct('fx', 500, 'fy', 500, 'cx', 319.5, 'cy', 239.5), ...
    'colorDistortion', zeros(1,5));
cameraPoints = [0 0 2; 0.2 -0.1 2; 10 0 1; 0 0 -1];
radarPoints = (meta.cameraToRadarR * cameraPoints.').';
pixels = project_radar_points_to_rgb(radarPoints, meta, calibration, [480 640]);
expected = [320.5 240.5; 370.5 215.5];
assert(isequal(size(pixels), [2 2]));
assert(max(abs(pixels(:) - expected(:))) < 1e-12);
result = struct('pass', true, 'pixels', pixels);
fprintf('PASS: radar/depth coordinates project to RGB pixels with clipping.\n');
end

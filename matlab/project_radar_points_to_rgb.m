function pixels = project_radar_points_to_rgb(pointsRadar, metaOut, calibration, imageSize)
%PROJECT_RADAR_POINTS_TO_RGB Project ADT7880 coordinates onto the RGB image.
pixels = zeros(0, 2);
if isempty(pointsRadar), return; end
validateattributes(imageSize, {'numeric'}, {'numel', 2, 'positive', 'finite'});
pointsRadar = double(pointsRadar(:,1:3));
pointsRadar = pointsRadar(all(isfinite(pointsRadar), 2), :);
if isempty(pointsRadar), return; end

cameraToRadarR = double(metaOut.cameraToRadarR);
cameraOriginInRadar = reshape(double(metaOut.cameraOriginInRadarM), 1, 3);
pointsDepth = (cameraToRadarR.' * (pointsRadar - cameraOriginInRadar).').';
pointsColor = (calibration.depthToColorR * pointsDepth.').';
pointsColor = pointsColor + calibration.depthToColorT;

pointsColor = pointsColor(pointsColor(:,3) > 0.05, :);
if isempty(pointsColor), return; end
x = pointsColor(:,1) ./ pointsColor(:,3);
y = pointsColor(:,2) ./ pointsColor(:,3);
distortion = calibration.colorDistortion;
r2 = x.^2 + y.^2;
radial = 1 + distortion(1)*r2 + distortion(2)*r2.^2 + distortion(5)*r2.^3;
xd = x.*radial + 2*distortion(3).*x.*y + distortion(4).*(r2 + 2*x.^2);
yd = y.*radial + distortion(3).*(r2 + 2*y.^2) + 2*distortion(4).*x.*y;
u = calibration.colorK.fx * xd + calibration.colorK.cx + 1;
v = calibration.colorK.fy * yd + calibration.colorK.cy + 1;
inside = u >= 1 & u <= imageSize(2) & v >= 1 & v <= imageSize(1);
pixels = [u(inside), v(inside)];
end

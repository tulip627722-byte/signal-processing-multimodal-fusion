function result = test_fit_lidar_board_plane()
%TEST_FIT_LIDAR_BOARD_PLANE Synthetic tilted-board regression.

rng(71, 'twister');
angles = [3 10 20];
errors = nan(size(angles));
tiltErrors = nan(size(angles));
for index = 1:numel(angles)
    angle = deg2rad(angles(index));
    normal = [0 cos(angle) -sin(angle)];
    axisU = [1 0 0]; axisV = cross(normal, axisU); axisV = axisV/norm(axisV);
    expectedCenter = [.12 4.5 -.18] + [index*.03 index*.5 index*.02];
    count = 5000;
    u = .48*(2*rand(count,1)-1);
    v = .34*(2*rand(count,1)-1);
    keep = rand(count,1) < (.35 + .65*(u-min(u))/(max(u)-min(u)));
    u = u(keep); v = v(keep);
    board = expectedCenter + u*axisU + v*axisV + .003*randn(nnz(keep),1).*normal;
    outliers = expectedCenter + [1.2*(rand(1300,1)-.5), ...
        .6*(rand(1300,1)-.5), 1.0*(rand(1300,1)-.5)];
    points = [board;outliers];
    [estimated, info] = fit_lidar_board_plane(points, struct('randomSeed',70+index));
    errors(index) = norm(estimated-expectedCenter);
    tiltErrors(index) = abs(info.tiltDeg-angles(index));
end
result = struct('pass',all(errors<.020) && all(tiltErrors<1.0), ...
    'anglesDeg',angles,'centerErrorsM',errors,'tiltErrorsDeg',tiltErrors);
fprintf('Tilted-board center errors [mm]: %s; tilt errors [deg]: %s\n', ...
    mat2str(errors*1000,3), mat2str(tiltErrors,3));
if ~result.pass, error('Synthetic tilted-board center recovery failed.'); end
end

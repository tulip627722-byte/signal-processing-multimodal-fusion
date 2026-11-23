function result = test_clean_depth_cloud()
%TEST_CLEAN_DEPTH_CLOUD Verify deterministic flyer and isolated-point removal.
[u, v] = meshgrid(20:60, 15:45);
z = 3 + 0.001 * (u - 40);
points = [(u(:)-40).*z(:)/100, z(:), -(v(:)-30).*z(:)/100];
uv = [u(:) v(:)];
flyerUv = [25 20; 35 25; 55 40];
flyers = [zeros(3,1), [1.2; 5.5; 0.8], zeros(3,1)];
isolatedUv = [3 3; 75 55];
isolated = [0 2 0; 0 4 0];
inputPoints = [points; flyers; isolated];
inputUv = [uv; flyerUv; isolatedUv];
[clean, ~, stats] = clean_depth_cloud(inputPoints, inputUv, [60 80], ...
    struct('voxelM', 0, 'minVoxelNeighbors', 0));
centerShift = norm(median(clean,1)-median(points,1));
retainedRatio = size(clean,1) / size(points,1);
badPointCount = sum(clean(:,2) < 2 | clean(:,2) > 4.5);
result = struct('pass', retainedRatio >= 0.98 && badPointCount == 0 && centerShift < 0.005, ...
    'inputCount', stats.inputCount, 'outputCount', stats.outputCount, ...
    'retainedRatio', retainedRatio, 'badPointCount', badPointCount, 'centerShiftM', centerShift);
fprintf('深度清理合成测试：%d -> %d 点，主体保留 %.1f%%，坏点 %d，中心偏移 %.2f mm，%s\n', ...
    stats.inputCount, stats.outputCount, 100*retainedRatio, badPointCount, ...
    centerShift*1000, pass_text(result.pass));
if ~result.pass, error('深度清理合成测试未通过。'); end
end

function text = pass_text(value)
if value, text = 'PASS'; else, text = 'FAIL'; end
end

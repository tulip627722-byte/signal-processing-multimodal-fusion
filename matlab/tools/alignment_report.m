function report = alignment_report(resFile, takeDir)
%ALIGNMENT_REPORT 定量检查雷达点投影到同步深度图后的轴向残差。
%
%   alignment_report('..._res.mat')
%   report = alignment_report('..._res.mat', 'E:\leida\11-4\take目录')
%
% 正残差表示雷达点比同一视线上的深度表面更远。毫米波和结构光对材质
% 的响应不同，因此本工具用于发现明显的外参/镜像/单位错误，不把残差强行
% 当作完整 6 自由度标定结果。

if nargin < 1 || isempty(resFile)
    error('请传入 compute_take 生成的 *_res.mat');
end
S = load(resFile, 'pts', 'depthFrame', 'depthDt', 'metaOut');
if nargin < 2 || isempty(takeDir)
    takeDir = S.metaOut.takeDir;
end
if ~isfolder(takeDir)
    error('找不到原始 take 目录：%s', takeDir);
end

required = {'cameraToRadarR', 'cameraOriginInRadarM'};
for i = 1:numel(required)
    if ~isfield(S.metaOut, required{i})
        error('结果不含 %s，请用当前 compute_take 重新计算。', required{i});
    end
end

Rrc = double(S.metaOut.cameraToRadarR);
t = reshape(double(S.metaOut.cameraOriginInRadarM), 1, 3);
fx = double(S.metaOut.fx); fy = double(S.metaOut.fy);
cx = double(S.metaOut.cx); cy = double(S.metaOut.cy);
scale = 0.001;
if isfield(S.metaOut, 'depthScale'), scale = double(S.metaOut.depthScale); end
mirror = isfield(S.metaOut, 'mirrorDepth') && S.metaOut.mirrorDepth;

residual = [];
projected = 0;
for k = 1:numel(S.pts)
    radarPoints = double(S.pts{k}(:, 1:3));
    if isempty(radarPoints), continue; end
    cameraPoints = (Rrc.' * (radarPoints - t).').';
    inFront = cameraPoints(:, 3) > 0.3;
    cameraPoints = cameraPoints(inFront, :);
    if isempty(cameraPoints), continue; end

    u = round(fx * cameraPoints(:, 1) ./ cameraPoints(:, 3) + cx) + 1;
    v = round(fy * cameraPoints(:, 2) ./ cameraPoints(:, 3) + cy) + 1;
    depthPath = fullfile(takeDir, 'depth', sprintf('%06d.png', S.depthFrame(k)));
    if ~isfile(depthPath), continue; end
    D = imread(depthPath);
    if size(D, 3) > 1, D = D(:, :, 1); end
    if mirror
        D = fliplr(D);
        cxMirror = (size(D, 2) - 1) - cx;
        u = round(fx * cameraPoints(:, 1) ./ cameraPoints(:, 3) + cxMirror) + 1;
    end
    inside = u >= 1 & u <= size(D, 2) & v >= 1 & v <= size(D, 1);
    u = u(inside); v = v(inside);
    predicted = cameraPoints(inside, 3);
    projected = projected + numel(predicted);
    if isempty(predicted), continue; end
    measured = double(D(sub2ind(size(D), v, u))) * scale;
    valid = measured > 0.3 & measured < 12;
    residual = [residual; predicted(valid) - measured(valid)]; %#ok<AGROW>
end

if isempty(residual)
    error('没有得到有效投影残差，请检查深度帧路径和外参。');
end
absResidual = abs(residual);
report = struct('frames', numel(S.pts), 'projectedRadarPoints', projected, ...
    'validDepthComparisons', numel(residual), ...
    'medianSignedM', median(residual), 'medianAbsM', median(absResidual), ...
    'p90AbsM', prctile(absResidual, 90), ...
    'within20cmRatio', mean(absResidual <= 0.20), ...
    'medianTimeAbsMs', median(abs(double(S.depthDt))));

fprintf('\n雷达 -> 同步深度图投影检查（雷达坐标为基准）\n');
fprintf('  外参状态：%s\n', S.metaOut.spatialAlignmentStatus);
fprintf('  R = [%g %g %g; %g %g %g; %g %g %g]\n', Rrc.');
fprintf('  相机原点（雷达系）= [%+.3f %+.3f %+.3f] m\n', t);
fprintf('  时间匹配 |dt| 中位：%.1f ms\n', report.medianTimeAbsMs);
fprintf('  有效比较：%d / 投影 %d 点\n', report.validDepthComparisons, projected);
fprintf('  轴向残差：有符号中位 %+.3f m，绝对中位 %.3f m，P90 %.3f m\n', ...
    report.medianSignedM, report.medianAbsM, report.p90AbsM);
fprintf('  |残差| <= 0.20 m：%.1f%%\n\n', 100 * report.within20cmRatio);
end

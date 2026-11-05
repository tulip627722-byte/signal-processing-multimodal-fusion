function [resFile, summary] = compute_take(takeDir, opts)
%COMPUTE_TAKE 计算脚本：把一次采集算成可视化能直接播的 res.mat
%
%   compute_take('D:\7880\Realtime\capture\takes\20261027_181021_室内3米站立-10s')
%   compute_take(takeDir, struct('stride',4,'eps',0.25,'minPts',3,'floorZ',-1.15))
%
% 做的事（只算不画）：
%   1) 读 radar_adc.mat，逐帧拼回原始 ADC
%   2) 调 algorithm\process_final（3.0 主链：8T 检测 + 1T 门限 + 自写近场 Bartlett）
%   3) 调 algorithm\cluster_points 做 DBSCAN
%   4) 按 align.csv 找同一时刻的深度/彩色帧，反投影成带颜色的点云
%   5) 全部存进 results\<take>_res.mat，交给 view_take 播
%
% 产物字段：
%   frames      雷达完整帧号
%   pts{i}      N×6 [X Y Z 强度dB 距离 速度]
%   labels{i}   N×1 簇编号（0=噪点）
%   nClusters   簇数
%   depthPts{i} M×3 米（雷达坐标系，固定安装变换）
%   depthCol{i} M×3 uint8 RGB
%   depthFrame / colorFrame / depthDt / ms

here = fileparts(mfilename('fullpath'));
PT = paths();          % 注意：变量名不能叫 P，下文 P 是点云
if nargin < 2, opts = struct(); end
o = setdef(opts, here);
addpath(fullfile(here, 'algorithm'), '-begin');
addpath(fullfile(here, 'algorithm', 'lib'), '-begin');   % hamming / pctile / CFAR mex 都在这里

if ~isfolder(takeDir), error('找不到 take 目录：%s', takeDir); end
[~, takeName] = fileparts(takeDir);
fprintf('\n===== 计算 %s =====\n', takeName);

% ---------------- meta / 内参 ----------------
meta = jsondecode(fileread(fullfile(takeDir, 'meta.json')));
fx = 570.3422; fy = 570.3422; cx = 319.5; cy = 239.5;
depthScale = 0.001;
if isfield(meta, 'camera') && isfield(meta.camera, 'depthIntrinsics')
    K = meta.camera.depthIntrinsics;
    if isfield(K, 'fx'), fx = K.fx; fy = K.fy; cx = K.cx; cy = K.cy; end
    if isfield(K, 'depthScale'), depthScale = K.depthScale; end
end
if isfield(meta, 'depth') && isfield(meta.depth, 'depthScaleMetersPerUnit')
    depthScale = meta.depth.depthScaleMetersPerUnit;
end

% ---------------- 雷达 ADC ----------------
S = load(fullfile(takeDir, 'radar_adc.mat'), 'ADCData');
adt = S.ADCData;
per = 2048;
payloadBytes = numel(adt{1, 4});

ids = zeros(size(adt, 1), 1);
for i = 1:size(adt, 1)
    v = adt{i, 2}; if iscell(v) && numel(v) == 1, v = v{1}; end
    ids(i) = double(v(1));
end
[sortedIds, sortIdx] = sort(ids);
fidx = floor(sortedIds / per) + 1;
[uf, ~, grp] = unique(fidx);
cnt = accumarray(grp, 1);
done = sort(uf(cnt == per));
if isempty(done), error('radar_adc.mat 里没有完整帧'); end
if ~isempty(o.maxFrames) && numel(done) > o.maxFrames
    sel = round(linspace(1, numel(done), o.maxFrames));
    done = done(sel);
end
fprintf('完整帧 %d 个\n', numel(done));
frames = done(:);                      % 存档用

% ---------------- 对齐表 ----------------
al = read_align(fullfile(takeDir, 'align.csv'));

% ---------------- 逐帧计算 ----------------
cfg = struct('detectMode', '8T', 'relativeGateRatio', 0.5, ...
             'rangeOffsetM', -o.radarRangeBiasM, ...
             'azimuthOffsetDeg', -o.radarAzimuthBiasDeg, ...
             'elevationOffsetDeg', -o.radarElevationBiasDeg); % 门限走默认值（1.53）
nF = numel(done);
pts = cell(nF, 1); labels = cell(nF, 1); nClusters = zeros(nF, 1);
depthPts = cell(nF, 1); depthCol = cell(nF, 1); depthUV = cell(nF, 1);
depthFrame = zeros(nF, 1); colorFrame = zeros(nF, 1);
depthDt = nan(nF, 1); colorDt = nan(nF, 1);
ms = zeros(nF, 1); radarT = nan(nF, 1);

for i = 1:nF
    f = done(i);
    raw = assemble(adt, sortedIds, sortIdx, f, per, payloadBytes);

    t0 = tic;
    try
        [P, ~] = process_final(raw, cfg);
        [lab, cl] = cluster_points(P, o.eps, o.minPts);
    catch ex
        fprintf(2, '  帧 %d 算法失败：%s\n', f, ex.message);
        P = zeros(0, 6); lab = zeros(0, 1); cl = struct('n', {});
    end
    ms(i) = toc(t0) * 1000;
    pts{i} = P; labels{i} = lab; nClusters(i) = numel(cl);
    % 对齐：优先按雷达帧号查表
    row = find(al.radar == f, 1);
    if isempty(row)
        row = max(1, min(numel(al.radar), round(i / nF * numel(al.radar))));
    end
    radarT(i) = al.radarT(row);
    depthFrame(i) = al.depth(row);
    colorFrame(i) = al.color(row);
    depthDt(i) = al.depthDt(row);
    colorDt(i) = al.colorDt(row);

    [dp, dc, uv] = build_depth_cloud(takeDir, depthFrame(i), colorFrame(i), ...
        fx, fy, cx, cy, depthScale, o.stride, o.withColor, o.mirrorDepth, ...
        o.cameraToRadarR, o.cameraOriginInRadarM, o.useFloorAlignment, o.floorZ);
    depthPts{i} = single(dp);
    depthCol{i} = dc;
    depthUV{i} = uv;

    if mod(i, 5) == 0 || i == nF
        fprintf('  %d/%d  帧 %d  %d 点 / %d 簇  %d 深度点  %.0f ms\n', ...
            i, nF, f, size(P, 1), nClusters(i), size(dp, 1), ms(i));
    end
end

% ---------------- 存盘 ----------------
resDir = PT.takesDir;
if ~isfolder(resDir), mkdir(resDir); end
resFile = fullfile(resDir, [takeName '_res.mat']);
metaOut = struct('takeDir', takeDir, 'takeName', takeName, ...
                 'fx', fx, 'fy', fy, 'cx', cx, 'cy', cy, 'depthScale', depthScale, ...
                 'floorZ', o.floorZ, 'stride', o.stride, 'mirrorDepth', o.mirrorDepth, ...
                 'useFloorAlignment', o.useFloorAlignment, ...
                 'cameraToRadarR', o.cameraToRadarR, ...
                 'cameraOriginInRadarM', o.cameraOriginInRadarM, ...
                 'extrinsicsFile', o.extrinsicsFile, ...
                 'radarCorrectionFile', o.radarCorrectionFile, ...
                 'radarRangeBiasM', o.radarRangeBiasM, ...
                 'radarAzimuthBiasDeg', o.radarAzimuthBiasDeg, ...
                 'radarElevationBiasDeg', o.radarElevationBiasDeg, ...
                 'radarCorrectionStatus', o.radarCorrectionStatus, ...
                 'radarCorrectionNote', o.radarCorrectionNote, ...
                 'spatialAlignmentStatus', o.spatialAlignmentStatus, ...
                 'spatialAlignmentNote', o.spatialAlignmentNote, ...
                 'temporalAlignmentMethod', 'radar_receive_window_midpoint_nearest_camera', ...
                 'eps', o.eps, 'minPts', o.minPts, ...
                 'cfg', cfg, 'createdAt', datestr(now, 'yyyy-mm-dd HH:MM:SS'), ...
                 'algorithm', '3.0 process_final + angle_nf_bartlett（无 MEX 测角）');
% 源数据时间戳：用来判断 res.mat 是不是过期了
stamp = 0;
stampFiles = {'radar_adc.mat', 'meta.json', 'align.csv'};
for ii = 1:numel(stampFiles)
    fp = fullfile(takeDir, stampFiles{ii});
    if isfile(fp)
        d = dir(fp);
        stamp = max(stamp, d.datenum);
    end
end
if isfile(o.extrinsicsFile)
    d = dir(o.extrinsicsFile);
    stamp = max(stamp, d.datenum);
end
if isfile(o.radarCorrectionFile)
    d = dir(o.radarCorrectionFile);
    stamp = max(stamp, d.datenum);
end
metaOut.sourceStamp = stamp;
save(resFile, 'frames', 'radarT', 'pts', 'labels', 'nClusters', ...
     'depthPts', 'depthCol', 'depthUV', 'depthFrame', 'colorFrame', ...
     'depthDt', 'colorDt', 'ms', ...
     'metaOut', '-v7.3');

summary = struct('take', takeName, 'frames', nF, 'msMedian', median(ms), ...
                 'ptsMedian', median(cellfun(@(x) size(x, 1), pts)), ...
                 'cluMedian', median(nClusters), 'file', resFile);
fprintf('点云中位 %d 点 / %d 簇，单帧中位 %.0f ms\n', ...
    summary.ptsMedian, summary.cluMedian, summary.msMedian);
fprintf('已存：%s\n', resFile);
fprintf('看：view_take(''%s'')\n', resFile);
end

% ============================================================
function [dp, dc, uv] = build_depth_cloud(takeDir, depthNo, colorNo, fx, fy, cx, cy, ...
    depthScale, stride, withColor, mirror, Rrc, cameraOriginRadar, useFloorAlignment, floorZ)
if nargin < 10 || isempty(mirror), mirror = true; end   % 深度图默认左右镜像
if nargin < 9, withColor = false; end
dp = zeros(0, 3); dc = zeros(0, 3, 'uint8'); uv = zeros(0, 2, 'uint16');
p = fullfile(takeDir, 'depth', sprintf('%06d.png', depthNo));
if ~isfile(p), return; end
D = imread(p);
if size(D, 3) > 1, D = D(:, :, 1); end
if mirror
    % 设备输出的深度图左右是反的（和彩色、和雷达 X 方向相反），这里翻回来。
    % 主点跟着镜像一起翻，否则非对称主点会引入横向偏移。
    D = fliplr(D);
    cx = (size(D, 2) - 1) - cx;
end
Zm = double(D) * depthScale;
[dh, dw] = size(Zm);

[U, V] = meshgrid(0:dw-1, 0:dh-1);
Xc = (U - cx) .* Zm / fx;
Yc = (V - cy) .* Zm / fy;
Zc = Zm;

valid0 = Zm > 0.3 & Zm < 12;
if ~any(valid0(:)), return; end

% OpenNI 相机系：X 右 / Y 下 / Z 前。通过固定外参转到雷达系：
% X 右 / Y 前 / Z 上。默认外参来自装机尺寸，不是精密标定值。
cameraPoints = [Xc(valid0), Yc(valid0), Zc(valid0)];
radarPoints = (Rrc * cameraPoints.').';
radarPoints = radarPoints + reshape(cameraOriginRadar, 1, 3);

if useFloorAlignment
    % 仅供复现旧结果：场景遮挡会使此补偿在不同 take 间漂移。
    radarPoints(:, 3) = radarPoints(:, 3) + ...
        (floorZ - pctile(radarPoints(:, 3), 2));
end

valid = valid0;
if stride > 1
    keep = false(size(valid));
    keep(1:stride:end, 1:stride:end) = true;
    valid = valid & keep;
end

keepIndex = valid(valid0);
dp = radarPoints(keepIndex, :);
uv = uint16([U(valid), V(valid)]);
dc = repmat(uint8([150 170 200]), size(dp, 1), 1);

cp = fullfile(takeDir, 'color', sprintf('%06d.png', colorNo));
if withColor && colorNo > 0 && isfile(cp)
    C = imread(cp);
    R = double(C(:, :, 1)); G = double(C(:, :, 2)); B = double(C(:, :, 3));
    if size(C, 1) ~= dh || size(C, 2) ~= dw
        R = resize_nn(R, dh, dw); G = resize_nn(G, dh, dw); B = resize_nn(B, dh, dw);
    end
    dc = uint8(cat(3, R(valid), G(valid), B(valid)));
end
end

% ============================================================
function rows = read_align(path)
rows = struct('radar', [], 'radarT', [], 'depth', [], 'color', [], ...
              'depthDt', [], 'colorDt', []);
if isfile(path)
    fid = fopen(path, 'r'); fgetl(fid);
    A = textscan(fid, '%f%f%f%f%f%f', 'Delimiter', ','); fclose(fid);
    rows.radar = A{1}; rows.radarT = A{2}; rows.depth = A{3};
    rows.depthDt = A{4}; rows.color = A{5}; rows.colorDt = A{6};
    return;
end

% 没有 align.csv 就用 manifest.csv 现场算（换机器、没装采集程序也能跑）
rows = align_from_manifest(fileparts(path));
fprintf('  （没找到 align.csv，已用 manifest.csv 现场对齐 %d 帧）\n', numel(rows.radar));
end

% ============================================================
function raw = assemble(adt, sortedIds, sortIdx, frameNumber, per, payloadBytes)
lo = (frameNumber - 1) * per;
p1 = find(sortedIds >= lo, 1, 'first');
raw = zeros(payloadBytes, per, 'uint8');
for i = 1:per
    raw(:, i) = uint8(adt{sortIdx(p1 + i - 1), 4}(:));
end
end

% ============================================================
function o = setdef(o, here)
hasR = isfield(o, 'cameraToRadarR');
hasT = isfield(o, 'cameraOriginInRadarM');
hasRangeBias = isfield(o, 'radarRangeBiasM');
hasAzBias = isfield(o, 'radarAzimuthBiasDeg');
hasElBias = isfield(o, 'radarElevationBiasDeg');
d = struct('stride', 4, 'eps', 0.25, 'minPts', 3, 'floorZ', -1.15, 'maxFrames', [], ...
           'withColor', false, 'mirrorDepth', true, ...
           'extrinsicsFile', fullfile(here, 'radar_camera_extrinsics.json'), ...
           'radarCorrectionFile', fullfile(here, 'radar_measurement_correction.json'), ...
           'useFloorAlignment', false);
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(o, f{i}), o.(f{i}) = d.(f{i}); end
end
E = load_radar_camera_extrinsics(o.extrinsicsFile);
if ~E.applicable
    error('候选外参未通过质检，拒绝应用：%s', E.note);
end
if ~hasR, o.cameraToRadarR = E.cameraToRadarR; end
if ~hasT, o.cameraOriginInRadarM = E.cameraOriginInRadarM; end
C = load_radar_measurement_correction(o.radarCorrectionFile);
if ~C.applicable
    error('雷达测量补偿未获准应用：%s', C.note);
end
if ~hasRangeBias, o.radarRangeBiasM = C.rangeBiasM; end
if ~hasAzBias, o.radarAzimuthBiasDeg = C.azimuthBiasDeg; end
if ~hasElBias, o.radarElevationBiasDeg = C.elevationBiasDeg; end
o.radarCorrectionStatus = C.status;
o.radarCorrectionNote = C.note;
o.spatialAlignmentStatus = E.status;
o.spatialAlignmentNote = E.note;
if hasR || hasT
    o.spatialAlignmentStatus = 'runtime_override';
    o.spatialAlignmentNote = 'cameraToRadarR 或 cameraOriginInRadarM 由 compute_take 的 opts 覆盖。';
end
validateattributes(o.cameraToRadarR, {'numeric'}, {'size', [3 3], 'finite'});
validateattributes(o.cameraOriginInRadarM, {'numeric'}, {'numel', 3, 'finite'});
validateattributes(o.radarRangeBiasM, {'numeric'}, {'scalar', 'finite', 'nonnegative'});
validateattributes(o.radarAzimuthBiasDeg, {'numeric'}, {'scalar', 'finite'});
validateattributes(o.radarElevationBiasDeg, {'numeric'}, {'scalar', 'finite'});
o.cameraOriginInRadarM = reshape(double(o.cameraOriginInRadarM), 1, 3);
end

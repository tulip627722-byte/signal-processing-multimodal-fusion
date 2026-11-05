function al = align_from_manifest(takeDir)
%ALIGN_FROM_MANIFEST 没有 align.csv 时，用 manifest.csv 现场算一份对齐关系
%
%   al = align_from_manifest(takeDir)
%   al.radar / al.radarT / al.depth / al.depthDt / al.color / al.colorDt
%
% 每个雷达帧使用首包/末包接收时刻的中点，再找时间上最近的深度帧和彩色帧，
% 残差 = 图像时刻 − 雷达时刻（正数表示图像比雷达晚）。
%
% 这样换机器、或者对方没装采集程序时，计算链路也不会因为缺 align.csv 直接死掉。

manifest = fullfile(takeDir, 'manifest.csv');
if ~isfile(manifest)
    error('既没有 align.csv，也没有 manifest.csv：%s', takeDir);
end

fid = fopen(manifest, 'r');
if fid < 0
    error('打不开 manifest.csv：%s', manifest);
end
fgetl(fid);
C = textscan(fid, '%s %f %f %s %s', 'Delimiter', ',');
fclose(fid);

stream = C{1};
frame = C{2};
tms = C{3};

ri = strcmp(stream, 'radar');
di = strcmp(stream, 'depth');
ci = strcmp(stream, 'color');

rt = tms(ri); rf = frame(ri);
[rt, ord] = sort(rt);
rf = rf(ord);

% manifest 中的 radar 时刻是单个包时刻，而一个物理帧的接收窗约 0.26 s。
% 使用 radar_frames.csv 的接收窗中点，避免系统性的半帧时偏。
frameCsv = fullfile(takeDir, 'radar_frames.csv');
if isfile(frameCsv)
    M = readmatrix(frameCsv, 'NumHeaderLines', 1);
    M = M(:, 1:3);
    good = all(isfinite(M), 2);
    M = M(good, :);
    rf = M(:, 1);
    rt = mean(M(:, 2:3), 2);
end
dt = tms(di); df = frame(di);
ct = tms(ci); cf = frame(ci);

n = numel(rt);
al = struct();
al.radar = rf;
al.radarT = rt;
al.depth = zeros(n, 1);
al.depthDt = zeros(n, 1);
al.color = zeros(n, 1);
al.colorDt = zeros(n, 1);

for i = 1:n
    if ~isempty(dt)
        [~, j] = min(abs(dt - rt(i)));
        al.depth(i) = df(j);
        al.depthDt(i) = dt(j) - rt(i);
    end
    if ~isempty(ct)
        [~, j] = min(abs(ct - rt(i)));
        al.color(i) = cf(j);
        al.colorDt(i) = ct(j) - rt(i);
    end
end
end

function verify_out3m()
% 验证室外3m组第10帧的真实数据质量
cd('G:\3.2\matlab'); addpath('tools','algorithm','algorithm/lib');
P = paths();
S = load(fullfile(P.takesDir, '20261105_152723_室外3米_res.mat'));
k = min(numel(S.frames), 10);

% 深度图质量
df = fullfile(S.metaOut.takeDir, 'depth', sprintf('%06d.png', S.depthFrame(k)));
D = imread(df); if size(D,3)>1, D=D(:,:,1); end
D = fliplr(double(D)) * 0.001;
Draw = D;
D(D < 0.3 | D > 9) = NaN;
nTot = numel(D);
nValid = nnz(isfinite(D));
fprintf('深度图总像素: %d\n', nTot);
fprintf('有效像素: %d (%.1f%%)\n', nValid, 100*nValid/nTot);
if nValid > 0
    fprintf('有效深度 min=%.2f max=%.2f 中位=%.2f m\n', min(D(isfinite(D))), max(D(isfinite(D))), median(D(isfinite(D))));
    % 深度图的空间分辨力：4邻域深度差>5cm的像素占比（衡量纹理丰富度）
    Dz = abs(diff(D,1,2));
    nEdge = nnz(Dz > 0.05);
    fprintf('深度跳变像素(>5cm): %d (%.2f%%)  <- 越高说明深度图有真实结构\n', nEdge, 100*nEdge/nValid);
end

% 雷达点
Pts = double(S.pts{k});
fprintf('\n雷达点总数: %d\n', size(Pts,1));
fprintf('雷达 Y 范围: %.2f ~ %.2f m\n', min(Pts(:,2)), max(Pts(:,2)));
fprintf('Y<3.5m(板区距离段)的点: %d\n', nnz(Pts(:,2) < 3.5 & Pts(:,2) > 2.5));

% 原始深度值直方（看是不是集中在一个值 = 假对齐）
if nValid > 0
    edges = 0.3:0.1:4.0;
    cnt = histcounts(D(isfinite(D)), edges);
    fprintf('\n深度直方(0.3~4.0m, 0.1m/格):\n');
    for i = 1:numel(cnt)
        if cnt(i) > 0, fprintf('  %.1f~%.1fm: %d\n', edges(i), edges(i+1), cnt(i)); end
    end
end
end

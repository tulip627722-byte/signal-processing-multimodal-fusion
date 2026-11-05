function [labels, clusters] = cluster_points(pts, eps, minPts)
%CLUSTER_POINTS 3D DBSCAN：把点云分组，而不是删点。
%
%   [labels, clusters] = cluster_points(pts, eps, minPts)
%
% 输入
%   pts     N×6 或 N×3，前三维是 X/Y/Z（米）；有第 6 列就顺带统计速度
%   eps     邻域半径（米），默认 0.20
%   minPts  成为核心点所需的最少邻居数（含自己），默认 3
%
% 输出
%   labels  N×1，0 = 噪点（不属于任何簇），k = 第 k 个簇
%   clusters  struct 数组，每个簇的几何与运动特征
%
% 为什么不直接用半径去删点：
%   人体是"扩展目标"，躯干、手臂、腿在距离和微多普勒上本来就分开，
%   半径一压就把部位之间的空隙填平，姿态信息全没了。DBSCAN 只给点分组，
%   哪个点属于哪个目标、哪些是孤立噪点，都保留可追溯。

if nargin < 2 || isempty(eps),    eps = 0.20; end
if nargin < 3 || isempty(minPts), minPts = 3; end

N = size(pts,1);
labels = zeros(N,1);
clusters = struct('id',{},'n',{},'centroid',{},'bbox',{},'extent',{}, ...
                  'meanInt',{},'meanVel',{},'stdVel',{},'meanRange',{});
if N == 0, return; end

% 距离矩阵（点云规模在几百到几千，直接算完全够）
X = pts(:,1); Y = pts(:,2); Z = pts(:,3);
D2 = (X - X.').^2 + (Y - Y.').^2 + (Z - Z.').^2;
nb = D2 <= eps^2;
nNeighbors = sum(nb, 2);
isCore = nNeighbors >= minPts;

cid = 0;
for i = 1:N
    if labels(i) ~= 0 || ~isCore(i), continue; end
    cid = cid + 1;
    labels(i) = cid;
    queue = i;
    while ~isempty(queue)
        j = queue(1); queue(1) = [];
        if ~isCore(j), continue; end          % 边界点不再扩张
        add = find(nb(j,:) & labels.' == 0);
        labels(add) = cid;
        queue = [queue, add];                 %#ok<AGROW>
    end
end

% 特征
for k = 1:cid
    idx = find(labels == k);
    P = pts(idx, :);
    bmin = min(P(:,1:3), [], 1);
    bmax = max(P(:,1:3), [], 1);
    c = struct();
    c.id = k;
    c.n = numel(idx);
    c.centroid = mean(P(:,1:3), 1);
    c.bbox = [bmin; bmax];
    c.extent = bmax - bmin;
    c.meanInt = mean(P(:,4));
    if size(P,2) >= 6
        c.meanVel = mean(P(:,6));
        c.stdVel  = std(P(:,6));
        c.meanRange = mean(P(:,5));
    else
        c.meanVel = NaN; c.stdVel = NaN; c.meanRange = NaN;
    end
    clusters(k) = c;
end
if ~isempty(clusters)
    [~,ord] = sort([clusters.n],'descend');
    clusters = clusters(ord);
end
end

function compare_dump(f30, f31)
%COMPARE_DUMP 对比两份 dump_points 的结果：检出一致性 + 点云层面差异
S0 = load(f30); S1 = load(f31);
r0 = S0.res; r1 = S1.res;

fprintf('\n%-34s %7s %7s %8s %8s %8s %6s %6s\n', ...
    '帧', 'CFAR0', 'CFAR1', '点0', '点1', '匹配率', '簇0', '簇1');
fprintf('%s\n', repmat('-', 1, 96));

ratios = [];
for i = 1:numel(r0)
    P0 = r0(i).P; P1 = r1(i).P;
    if isempty(P1) || isempty(P0)
        m = NaN;
    else
        D = nn_dist(P1(:,1:3), P0(:,1:3));
        m = mean(min(D, [], 2) < 0.15);       % 3.1 的点在 3.0 点云 15cm 内有邻居的比例
    end
    [~, c0] = cluster_points(P0, 0.25, 3);
    [~, c1] = cluster_points(P1, 0.25, 3);
    fprintf('%-34s %7d %7d %8d %8d %7.1f%% %6d %6d\n', ...
        sprintf('%s/%d', r0(i).take(10:22), r0(i).frame), ...
        r0(i).cfar, r1(i).cfar, size(P0,1), size(P1,1), 100*m, numel(c0), numel(c1));
    ratios(end+1) = size(P1,1) / max(size(P0,1), 1); %#ok<AGROW>
end
fprintf('\n点数比 3.1/3.0：中位 %.3f，最小 %.3f，最大 %.3f\n', median(ratios), min(ratios), max(ratios));
fprintf('CFAR 比：中位 %.3f\n', median([r1.cfar] ./ [r0.cfar]));
fprintf('耗时：3.0 中位 %.0f ms，3.1 中位 %.0f ms\n', median([r0.ms]), median([r1.ms]));
end

% ============================================================
function D = nn_dist(A, B)
D = zeros(size(A,1), size(B,1));
for i = 1:size(A,1)
    D(i,:) = sqrt(sum((B - A(i,:)).^2, 2)).';
end
end

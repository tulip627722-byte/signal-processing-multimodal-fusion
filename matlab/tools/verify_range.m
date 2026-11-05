function verify_range()
%VERIFY_RANGE 用 3.2 的结果核对距离修正：雷达 Y vs 深度板子距离
here = fileparts(mfilename('fullpath'));
ml = fileparts(here);
addpath(fullfile(ml, 'algorithm'), '-begin');
addpath(fullfile(ml, 'algorithm', 'lib'), '-begin');
P = paths();

takes = { ...
    '20261104_133930_室内2米_1',   2.0; ...
    '20261104_134227_室内2米_2',   2.0; ...
    '20261104_132932_室内2米5_1',  2.5; ...
    '20261104_133030_室内2米5_2',  2.5; ...
    '20261104_134904_室内3米_1',   3.0; ...
    '20261104_134957_室内3米_2',   3.0};

fprintf('\n%-30s %7s %9s %9s %9s %9s\n', 'take', '标注m', '雷达Y(修正后)', '雷达Y(修正前)', '后-标注', '前-标注');
fprintf('%s\n', repmat('-', 1, 82));

for i = 1:size(takes, 1)
    f = fullfile(P.takesDir, [takes{i,1} '_res.mat']);
    if ~isfile(f), continue; end
    S = load(f, 'pts', 'metaOut');
    allY = [];
    for k = 1:numel(S.pts)
        Q = S.pts{k};
        if isempty(Q), continue; end
        allY = [allY; Q(abs(Q(:,6)) < 0.15, 2)];   % 静态点（板子）
    end
    if isempty(allY), continue; end
    edges = 0.5:0.02:6;
    h = histcounts(allY, edges);
    [~, bi] = max(h);
    yAfter = (edges(bi) + edges(bi+1)) / 2;
    sc = 0.8375;
    if isfield(S.metaOut, 'rangeScale'), sc = S.metaOut.rangeScale; end
    yBefore = yAfter / sc;
    tape = takes{i,2};
    fprintf('%-30s %7.2f %9.2f %9.2f %+9.2f %+9.2f\n', takes{i,1}, tape, ...
        yAfter, yBefore, yAfter - tape, yBefore - tape);
end
end

function analyze_range(root)
%ANALYZE_RANGE 逐组比较：标注距离 vs 深度相机 Y vs 毫米波雷达 Y
%
%   analyze_range('D:\7880\Realtime\capture\takes\11-4')

if nargin < 1
    root = 'D:\7880\Realtime\capture\takes\11-4';
end
here = fileparts(mfilename('fullpath'));
ml = fileparts(here);
addpath(fullfile(ml, 'algorithm'), '-begin');
addpath(fullfile(ml, 'algorithm', 'lib'), '-begin');

dirs = dir(root);
dirs = dirs([dirs.isdir]);
dirs = dirs(~ismember({dirs.name}, {'.', '..'}));

fprintf('\n%-38s %6s %8s %8s %8s %8s\n', 'take', '标注m', '深度Y', '雷达Y', '雷达-标注', '雷达-深度');
fprintf('%s\n', repmat('-', 1, 84));

rows = [];
for d = 1:numel(dirs)
    nm = dirs(d).name;
    td = fullfile(root, nm);
    if ~isfile(fullfile(td, 'radar_adc.mat')), continue; end

    Dtrue = parse_dist(nm);
    dY = depth_y(td);
    [rY, nPts] = radar_y(td);
    fprintf('%-38s %6.2f %8.3f %8.3f %+8.3f %+8.3f  (%d 点)\n', ...
        nm, Dtrue, dY, rY, rY - Dtrue, rY - dY, nPts);
    rows(end+1, :) = [Dtrue, dY, rY]; %#ok<AGROW>
end

if size(rows, 1) >= 2
    ok = ~isnan(rows(:,3)) & ~isnan(rows(:,1));
    p = polyfit(rows(ok,1), rows(ok,3), 1);
    fprintf('\n雷达 Y 对 标注距离 线性拟合：Y_radar = %.4f * Y_true + %.4f\n', p(1), p(2));
    fprintf('  即：斜率偏差 %+.1f%%，零点偏移 %+.0f cm\n', 100*(p(1)-1), 100*p(2));
    ok2 = ~isnan(rows(:,3)) & ~isnan(rows(:,2));
    p2 = polyfit(rows(ok2,2), rows(ok2,3), 1);
    fprintf('雷达 Y 对 深度 Y 线性拟合：Y_radar = %.4f * Y_depth + %.4f\n', p2(1), p2(2));
end
end

% ============================================================
function D = parse_dist(nm)
D = NaN;
tok = regexp(nm, '室内(\d)米(?:(\d))?', 'tokens', 'once');
if isempty(tok), return; end
D = str2double(tok{1});
if numel(tok) > 1 && ~isempty(tok{2})
    D = D + str2double(tok{2}) / 10;
end
end

% ============================================================
function Y = depth_y(td)
% 中央区域的深度中位数（板子放在正前方，占画面中间）
Y = NaN;
dfiles = dir(fullfile(td, 'depth', '*.png'));
if isempty(dfiles), return; end
pick = dfiles(round(linspace(1, numel(dfiles), 5)));
vals = [];
for i = 1:numel(pick)
    Dm = imread(fullfile(td, 'depth', pick(i).name));
    if size(Dm,3) > 1, Dm = Dm(:,:,1); end
    Z = double(Dm) * 0.001;
    c = Z(180:300, 250:390);            % 中央 120x140
    c = c(c > 0.3 & c < 8);
    if ~isempty(c), vals(end+1) = median(c); end %#ok<AGROW>
end
if ~isempty(vals), Y = median(vals); end
end

% ============================================================
function [rY, n] = radar_y(td)
% 雷达：取几帧，找距离直方图的峰（板子是最大反射面，静态）
rY = NaN; n = 0;
try
    hasGpu = false; %#ok<NASGU>
    S = load(fullfile(td, 'radar_adc.mat'), 'ADCData');
catch
    return;
end
adt = S.ADCData; per = 2048; payloadBytes = numel(adt{1,4});
ids = zeros(size(adt,1),1);
for i = 1:size(adt,1)
    v = adt{i,2}; if iscell(v) && numel(v)==1, v = v{1}; end
    ids(i) = double(v(1));
end
[sIds, sIdx] = sort(ids);
fidx = floor(sIds/per)+1; [uf,~,grp] = unique(fidx); cnt = accumarray(grp,1);
done = sort(uf(cnt==per));
if isempty(done), return; end
pick = done(round(linspace(1, numel(done), 3)));

cfg = struct('detectMode','8T','relativeGateRatio',0.5);
allY = [];
for k = pick(:).'
    raw = assemble(adt, sIds, sIdx, k, per, payloadBytes);
    try
        [P, ~] = process_final(raw, cfg);
    catch
        continue;
    end
    if isempty(P), continue; end
    allY = [allY; P(P(:,5) > 0.8 & P(:,5) < 6, 2)]; %#ok<AGROW>
end
if isempty(allY), return; end
n = numel(allY);
edges = 0.1:0.05:6;
h = histcounts(allY, edges);
[~, bi] = max(h);
rY = (edges(bi) + edges(bi+1)) / 2;
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

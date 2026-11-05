function peek_peaks(root, names)
%PEEK_PEAKS 打印深度与雷达的距离直方图峰值（定位板子那个峰）
if nargin < 2
    names = {'20261104_133930_室内2米_1', '20261104_132932_室内2米5_1', '20261104_134904_室内3米_1'};
end
here = fileparts(mfilename('fullpath'));
ml = fileparts(here);
addpath(fullfile(ml, 'algorithm'), '-begin');
addpath(fullfile(ml, 'algorithm', 'lib'), '-begin');

for i = 1:numel(names)
    td = fullfile(root, names{i});
    if ~isfolder(td), continue; end
    fprintf('\n===== %s =====\n', names{i});

    % ---- 深度：整幅图（去掉地面/太远）----
    dfiles = dir(fullfile(td, 'depth', '*.png'));
    pick = dfiles(round(linspace(1, numel(dfiles), 5)));
    vz = [];
    for j = 1:numel(pick)
        D = imread(fullfile(td, 'depth', pick(j).name));
        if size(D,3) > 1, D = D(:,:,1); end
        Z = double(D) * 0.001;
        Z = Z(Z > 0.8 & Z < 6);
        vz = [vz; Z(:)]; %#ok<AGROW>
    end
    edges = 0.8:0.02:6;
    h = histcounts(vz, edges);
    [~, ord] = sort(h, 'descend');
    fprintf('深度 距离直方图前 5 峰：\n');
    shown = 0;
    for q = 1:numel(ord)
        if shown >= 5, break; end
        if h(ord(q)) < 0.02 * max(h), break; end
        fprintf('   %.2f m  (%d 点)\n', (edges(ord(q)) + edges(ord(q)+1)) / 2, h(ord(q)));
        shown = shown + 1;
    end

    % ---- 雷达 ----
    S = load(fullfile(td, 'radar_adc.mat'), 'ADCData');
    adt = S.ADCData; per = 2048; payloadBytes = numel(adt{1,4});
    ids = zeros(size(adt,1),1);
    for m = 1:size(adt,1)
        v = adt{m,2}; if iscell(v) && numel(v)==1, v = v{1}; end
        ids(m) = double(v(1));
    end
    [sIds, sIdx] = sort(ids);
    fidx = floor(sIds/per)+1; [uf,~,grp] = unique(fidx); cnt = accumarray(grp,1);
    done = sort(uf(cnt==per));
    pickr = done(round(linspace(1, numel(done), 3)));
    cfg = struct('detectMode','8T','relativeGateRatio',0.5);
    vy = []; vyAng = [];
    for k = pickr(:).'
        raw = assemble(adt, sIds, sIdx, k, per, payloadBytes);
        try
            P = process_final(raw, cfg);
        catch
            continue;
        end
        if isempty(P), continue; end
        vy = [vy; P(:,2)]; %#ok<AGROW>
        vyAng = [vyAng; P(:,5)]; %#ok<AGROW>
    end
    if isempty(vy), fprintf('雷达：无点\n'); continue; end
    edges2 = 0.8:0.02:6;
    h2 = histcounts(vy, edges2);
    [~, ord2] = sort(h2, 'descend');
    fprintf('雷达 Y 直方图前 5 峰：\n');
    shown = 0;
    for q = 1:numel(ord2)
        if shown >= 5, break; end
        if h2(ord2(q)) < 0.02 * max(h2), break; end
        fprintf('   %.2f m  (%d 点)\n', (edges2(ord2(q)) + edges2(ord2(q)+1)) / 2, h2(ord2(q)));
        shown = shown + 1;
    end
    fprintf('雷达全部点 %d，Y 中位 %.2f，距离中位 %.2f\n', numel(vy), median(vy), median(vyAng));
end
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

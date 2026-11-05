function dump_points(root, outFile, nFrames)
%DUMP_POINTS 用指定版本的算法跑几帧，把结果存下来（供 3.0 vs 3.1 对比）
%
%   dump_points('D:\...\3.0\matlab', 'D:\...\cmp30.mat', 4)
%   dump_points('D:\...\3.1\matlab', 'D:\...\cmp31.mat', 4)

if nargin < 3, nFrames = 4; end
cap = 'D:\7880\Realtime\capture\takes';
takes = {'20261027_194851_室内4-2米移动5s', '20261027_194908_室内2-4米移动5s'};

addpath(fullfile(root, 'algorithm'), '-begin');
addpath(fullfile(root, 'algorithm', 'lib'), '-begin');

cfg = struct('detectMode', '8T', 'cfarThreshold', 1.6, 'relativeGateRatio', 0.5);
res = struct('take', {}, 'frame', {}, 'P', {}, 'cfar', {}, 'gate', {}, 'roi', {}, 'ms', {});

for t = 1:numel(takes)
    td = fullfile(cap, takes{t});
    S = load(fullfile(td, 'radar_adc.mat'), 'ADCData');
    adt = S.ADCData; per = 2048; payloadBytes = numel(adt{1,4});
    ids = zeros(size(adt,1),1);
    for i = 1:size(adt,1)
        v = adt{i,2}; if iscell(v) && numel(v)==1, v = v{1}; end
        ids(i) = double(v(1));
    end
    [sIds, sIdx] = sort(ids);
    fidx = floor(sIds/per)+1; [uf,~,grp] = unique(fidx); cnt = accumarray(grp,1);
    done = sort(uf(cnt==per));
    pick = done(round(linspace(1, numel(done), nFrames)));

    for k = pick(:).'
        raw = assemble(adt, sIds, sIdx, k, per, payloadBytes);
        t0 = tic;
        [P, inf1] = process_final(raw, cfg);
        ms = toc(t0) * 1000;
        res(end+1) = struct('take', takes{t}, 'frame', k, 'P', P, ...
                            'cfar', inf1.cfar, 'gate', inf1.gate, 'roi', inf1.roi, 'ms', ms); %#ok<AGROW>
        fprintf('  %s 帧 %d: CFAR %d → 门限 %d → ROI %d → 点 %d (%.0f ms)\n', ...
            takes{t}(1:15), k, inf1.cfar, inf1.gate, inf1.roi, size(P,1), ms);
    end
end
save(outFile, 'res', '-v7.3');
fprintf('已存：%s\n', outFile);
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

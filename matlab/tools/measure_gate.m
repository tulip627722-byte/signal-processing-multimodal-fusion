function measure_gate(nFrames)
%MEASURE_GATE 站立 + 移动数据上，比较三种 CFAR 设置（决定门限要不要动）
%
%   measure_gate        % 每组 3 帧
%   measure_gate(5)
%
% 三种设置：
%   MEX            原厂（bias 无意义，走 mex_RD_CACFAR）
%   local 1.60     自写，bias = 1（干净版）
%   local 1.53     自写，bias = 1，但门限按 1.6*0.955 下调
%
% 输出：CFAR 检出数、最终点数、簇数、以及相对 MEX 的点匹配率

if nargin < 1, nFrames = []; end
if isempty(nFrames), nFrames = 3; end

here = fileparts(mfilename('fullpath'));
ml = fileparts(here);
addpath(fullfile(ml, 'algorithm'), '-begin');
addpath(fullfile(ml, 'algorithm', 'lib'), '-begin');
addpath('D:\7880\Realtime\RealtimeOnPC\3.1\results\archive\mex_reference', '-begin');  % 原厂 MEX 仅供对照

P = paths();
cap = P.captureTakes;
takes = { ...
    '20261027_181050_室内2米站立-10s', '20261027_181021_室内3米站立-10s', '20261027_180947_室内4米站立-10s', ...
    '20261027_192011_室内2-4米移动10s', '20261027_192050_室内4-2米移动10s', ...
    '20261027_194851_室内4-2米移动5s', '20261027_194908_室内2-4米移动5s'};

variants = { ...
    struct('name', 'MEX', 'cfarSource', 'mex', 'cfarThreshold', 1.6, 'cfarBias', 1.0), ...
    struct('name', 'local1.60', 'cfarSource', 'local', 'cfarThreshold', 1.60, 'cfarBias', 1.0), ...
    struct('name', 'local1.53', 'cfarSource', 'local', 'cfarThreshold', 1.53, 'cfarBias', 1.0)};

fprintf('\n%-30s %8s | %6s %6s %6s | %6s %6s %6s | %6s %6s\n', ...
    'take/帧', '类型', 'CFAR m', '1.60', '1.53', '点 m', '1.60', '1.53', '匹配1.60', '匹配1.53');
fprintf('%s\n', repmat('-', 1, 118));

summary = zeros(0, 5);
for t = 1:numel(takes)
    td = fullfile(cap, takes{t});
    if ~isfolder(td), continue; end
    kind = '移动'; if contains(takes{t}, '站立'), kind = '站立'; end
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
        res = cell(1, 3);
        for v = 1:3
            cfg = variants{v};
            cfg.detectMode = '8T';
            cfg.relativeGateRatio = 0.5;
            try
                [Pp, inf1] = process_final(raw, cfg);
            catch ex
                fprintf(2, '  %s 失败：%s\n', cfg.name, ex.message);
                Pp = zeros(0,6); inf1 = struct('cfar',0);
            end
            res{v} = struct('P', Pp, 'cfar', inf1.cfar);
        end
        m160 = match_rate(res{2}.P, res{1}.P);
        m153 = match_rate(res{3}.P, res{1}.P);
        fprintf('%-30s %8s | %6d %6d %6d | %6d %6d %6d | %6.1f%% %6.1f%%\n', ...
            sprintf('%s/%d', takes{t}(10:min(end,24)), k), kind, ...
            res{1}.cfar, res{2}.cfar, res{3}.cfar, ...
            size(res{1}.P,1), size(res{2}.P,1), size(res{3}.P,1), 100*m160, 100*m153);
        summary(end+1,:) = [res{1}.cfar, res{2}.cfar, res{3}.cfar, m160, m153]; %#ok<AGROW>
    end
end

fprintf('\n=== 汇总（相对 MEX）===\n');
fprintf('  CFAR 计数比：local1.60 %.3f   local1.53 %.3f\n', ...
    median(summary(:,2)./summary(:,1)), median(summary(:,3)./summary(:,1)));
fprintf('  点匹配率中位：local1.60 %.1f%%   local1.53 %.1f%%\n', ...
    100*median(summary(:,4)), 100*median(summary(:,5)));
end

% ============================================================
function m = match_rate(A, B)
if isempty(A) || isempty(B), m = NaN; return; end
D = zeros(size(A,1), size(B,1));
for i = 1:size(A,1)
    D(i,:) = sqrt(sum((B(:,1:3) - A(i,1:3)).^2, 2)).';
end
m = mean(min(D, [], 2) < 0.15);
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

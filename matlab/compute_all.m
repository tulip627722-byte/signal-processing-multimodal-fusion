function compute_all(count, opts)
%COMPUTE_ALL 批量计算：默认把所有还没算过的 take 全算一遍
%
%   compute_all                     % 全部 take（已算过的自动跳过）
%   compute_all(5)                  % 只看最新 5 组
%   compute_all([], struct('force', true))     % 全部强制重算
%   compute_all(5, struct('stride', 4))
%
% 「算过了」的判定（三条件同时成立才跳过）：
%   1) results\<take>_res.mat 存在
%   2) 里面的参数（stride / eps / minPts / floorZ）与本次一致
%   3) res.mat 比源数据（radar_adc.mat / meta.json / align.csv）新
% 任一条不满足就重算：参数改了就重算、重新采过数据也重算。

if nargin < 1 || isempty(count), count = inf; end   % 默认：全部
if nargin < 2, opts = struct(); end

force = false;
if isfield(opts, 'force')
    force = logical(opts.force);
    opts = rmfield(opts, 'force');
end

here = fileparts(mfilename('fullpath'));
addpath(here, '-begin');

P = paths();
takesRoot = P.captureTakes;
if isfield(opts, 'root') && ~isempty(opts.root)
    takesRoot = opts.root;      % 允许指定别的采集根目录（比如 takes\11-4）
end
resDir = P.takesDir;
if ~isfolder(takesRoot)
    error(['采集目录不存在：%s' char(10) ...
           '请检查 paths.m 里的 captureTakes，或传入 opts.root。'], takesRoot);
end

list = dir(takesRoot);
list = list([list.isdir]);
list = list(~ismember({list.name}, {'.', '..', '_report'}));
% 只处理真实采集目录，避免把同级的 FFT 结果目录当成 take。
isTake = arrayfun(@(x) isfile(fullfile(x.folder, x.name, 'radar_adc.mat')) && ...
                        isfile(fullfile(x.folder, x.name, 'meta.json')), list);
list = list(isTake);
[~, ord] = sort([list.datenum], 'descend');
names = {list.name};
names = names(ord);
targets = names(1:min(count, numel(names)));

fprintf('\n===== 批量计算 %d 组 =====\n', numel(targets));

doneNames = {}; skipped = {};
fprintf('\n%-34s %8s %7s %8s\n', 'take', '点数中位', '簇中位', '耗时中位');
fprintf('%s\n', repmat('-', 1, 62));
for i = 1:numel(targets)
    takeDir = fullfile(takesRoot, targets{i});
    [ok, why] = is_up_to_date(resDir, targets{i}, takeDir, opts);
    if ~force && ok
        fprintf('  [跳过] %-30s （%s）\n', targets{i}, why);
        skipped{end+1} = targets{i}; %#ok<SAGROW>
        continue;
    end

    try
        [~, s] = compute_take(takeDir, opts);
        fprintf('  [新算] %-30s %8.0f %7.0f %8.0f\n', targets{i}, ...
            s.ptsMedian, s.cluMedian, s.msMedian);
        doneNames{end+1} = targets{i}; %#ok<SAGROW>
    catch ex
        fprintf(2, '  [失败] %s：%s\n', targets{i}, ex.message);
    end
end

if ~isempty(skipped)
    fprintf('\n跳过 %d 组（要重算加 force = true）\n', numel(skipped));
end
if ~isempty(doneNames)
    fprintf('新算 %d 组\n', numel(doneNames));
end
fprintf('\n播放：view_take\n');
end

% ============================================================
function [ok, why] = is_up_to_date(resDir, takeName, takeDir, o)
ok = false; why = '';
resFile = fullfile(resDir, [takeName '_res.mat']);
if ~isfile(resFile)
    why = '还没有 res.mat';
    return;
end

try
    m = load(resFile, 'metaOut');
catch
    why = 'res.mat 读不出来';
    return;
end
if ~isfield(m, 'metaOut')
    why = 'res.mat 里没有 metaOut';
    return;
end
meta = m.metaOut;

% 默认外参也参与缓存判断，避免修改 JSON 后仍误用旧点云。
try
    extPath = [];
    if isfield(o, 'extrinsicsFile'), extPath = o.extrinsicsFile; end
    E = load_radar_camera_extrinsics(extPath);
    if ~isfield(meta, 'cameraToRadarR') || ~isfield(meta, 'cameraOriginInRadarM') || ...
            any(abs(double(meta.cameraToRadarR(:)) - E.cameraToRadarR(:)) > 1e-9) || ...
            any(abs(double(meta.cameraOriginInRadarM(:)) - E.cameraOriginInRadarM(:)) > 1e-9)
        why = '雷达/相机外参已新增或发生变化';
        return;
    end
    dExt = dir(E.path);
    dRes = dir(resFile);
    if dRes.datenum < dExt.datenum
        why = '外参文件比 res.mat 新';
        return;
    end
catch ex
    why = ['外参检查失败：' ex.message];
    return;
end

% 参数比对（只有传进来的字段才比）
fields = {'stride', 'eps', 'minPts', 'floorZ', 'mirrorDepth', 'useFloorAlignment', ...
          'cameraToRadarR', 'cameraOriginInRadarM', ...
          'radarRangeBiasM', 'radarAzimuthBiasDeg', 'radarElevationBiasDeg'};
for i = 1:numel(fields)
    f = fields{i};
    if isfield(o, f) && isfield(meta, f)
        a = double(o.(f));
        b = double(meta.(f));
        if ~isequal(size(a), size(b)) || any(abs(a(:) - b(:)) > 1e-9)
            why = sprintf('%s 变了（%g → %g）', f, meta.(f), o.(f));
            return;
        end
    end
end

% 源数据时间戳比对
if isfield(meta, 'sourceStamp') && meta.sourceStamp > 0
    d = dir(resFile);
    if d.datenum < meta.sourceStamp
        why = '源数据比 res.mat 新';
        return;
    end
end

ok = true;
why = sprintf('算于 %s', meta.createdAt);
end

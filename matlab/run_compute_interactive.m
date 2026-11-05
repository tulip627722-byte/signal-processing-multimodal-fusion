function results = run_compute_interactive()
%RUN_COMPUTE_INTERACTIVE Select captured data and run the MATLAB pipeline.
%
% Usage from the MATLAB command window:
%   cd('E:\leida\3.2\matlab')
%   run_compute_interactive

if ~usejava('desktop')
    error('此入口需要 MATLAB Desktop。无界面环境请直接调用 compute_take 或 compute_all。');
end

here = fileparts(mfilename('fullpath'));
addpath(here, '-begin');
results = struct('mode', '', 'cancelled', false, 'output', []);

mode = questdlg('请选择计算方式', '雷达数据计算', ...
    '单组：选择 radar_adc.mat', '批量：选择采集父目录', '取消', ...
    '单组：选择 radar_adc.mat');
if isempty(mode) || strcmp(mode, '取消')
    results.cancelled = true;
    return;
end

[opts, cancelled] = choose_correction(here);
if cancelled
    results.cancelled = true;
    return;
end

if startsWith(mode, '单组')
    [fileName, takeDir] = uigetfile( ...
        {'radar_adc.mat', '雷达采集文件 (radar_adc.mat)'; '*.mat', 'MAT 文件 (*.mat)'}, ...
        '选择一次采集中的 radar_adc.mat');
    if isequal(fileName, 0)
        results.cancelled = true;
        return;
    end
    if ~strcmpi(fileName, 'radar_adc.mat')
        error('请选择采集目录中的 radar_adc.mat，当前选择：%s', fileName);
    end
    validate_take_dir(takeDir);
    [resFile, summary] = compute_take(takeDir, opts);
    results.mode = 'single';
    results.output = summary;
    msgbox(sprintf('计算完成。\n\n结果：%s', resFile), ...
        '雷达数据计算', 'help');
else
    rootDir = uigetdir(fileparts(here), '选择包含多组采集目录的父目录');
    if isequal(rootDir, 0)
        results.cancelled = true;
        return;
    end
    opts.root = rootDir;
    compute_all([], opts);
    results.mode = 'batch';
    results.output = rootDir;
    msgbox(sprintf('批量计算完成。\n\n结果目录：%s', ...
        fullfile(fileparts(here), 'results', 'takes')), ...
        '雷达数据计算', 'help');
end
end

function [opts, cancelled] = choose_correction(~)
choice = questdlg(['稳定主分支默认不应用实验补偿。' newline ...
    '如需试算，可手动输入距离、方位和俯仰偏差。'], '雷达测量补偿', ...
    '原始数据（零补偿）', '手动输入补偿', '取消', '原始数据（零补偿）');
cancelled = isempty(choice) || strcmp(choice, '取消');
opts = struct();
if cancelled, return; end
if strcmp(choice, '手动输入补偿')
    values = inputdlg({'距离测量偏差 (m)', '方位测量偏差 (deg)', ...
        '俯仰测量偏差 (deg)'}, '手动补偿', [1 36], {'0', '0', '0'});
    if isempty(values), cancelled = true; return; end
    nums = str2double(values);
    if any(~isfinite(nums)) || nums(1) < 0
        error('补偿参数必须是有限数值，距离偏差不能为负。');
    end
    opts.radarRangeBiasM = nums(1);
    opts.radarAzimuthBiasDeg = nums(2);
    opts.radarElevationBiasDeg = nums(3);
end
end

function validate_take_dir(takeDir)
required = {'radar_adc.mat', 'meta.json', 'manifest.csv'};
missing = required(~cellfun(@(f) isfile(fullfile(takeDir, f)), required));
if ~isempty(missing)
    error('所选目录缺少采集文件：%s', strjoin(missing, ', '));
end
end

%% run_calibration.m —— 雷达-相机外参标定总控脚本
%
% 用法：
%   1) 打开 MATLAB，Current Folder 切到 G:\3.2\matlab
%   2) 先改 paths.m 里的 P.captureTakes（采集数据目录，当前已设为 G:\11-5\11-5）
%   3) 在命令行直接：run_calibration
%      或者一步一步 Eval Section 跑
%
% 输出：
%   G:\3.2\results\takes\              —— 每组的 _res.mat（雷达点云 + 对齐深度帧号）
%   G:\3.2\results\tall_result.json     —— 逐组解 + 加权平均 t
%   G:\3.2\results\overlay_check\      —— 三画面投影检查图（彩色 + 深度 + 3D）
%
% 最终 t 值：脚本末尾打印，同时 tall_result.json 里也有。

clear; clc;

%% ============ 0. 环境 ============
here = fileparts(mfilename('fullpath'));
cd(here);
addpath(here, fullfile(here,'algorithm'), fullfile(here,'algorithm','lib'), fullfile(here,'tools'));
fprintf('当前目录: %s\n', here);

P = paths();
fprintf('采集目录: %s\n', P.captureTakes);
fprintf('结果目录: %s\n', P.takesDir);

%% ============ 1. 批量计算雷达点云 ============
% 把采集目录下所有 take 都跑一遍 process_final，输出 _res.mat
% （已算过的自动跳过，想重算加 force=true）
fprintf('\n==== 第 1 步：批量计算 compute_all ====\n');
compute_all([], struct('force', true));

%% ============ 2. 逐组解外参 t + 姿态 ============
% 对每组 _res.mat，用"雷达点投影到彩色图后其光轴距离应等于深度读数"的判据
% 解出该组的 (tx, ty, tz, yaw, pitch, roll)，然后残差 < 0.5 的组加权平均
fprintf('\n==== 第 2 步：逐组解外参 solve_t_all ====\n');
solve_t_all;

%% ============ 3. 导出三画面投影检查图 ============
% 用 tall_result.json 里逐组解的姿态 + 加权平均 t，把雷达点投影到彩色图/深度图
fprintf('\n==== 第 3 步：导出投影检查图 export_all ====\n');
export_all;

%% ============ 4. 读取并打印最终结果 ============
fprintf('\n==== 最终结果 ====\n');
resultFile = fullfile(P.resultsDir, 'tall_result.json');
if isfile(resultFile)
    js = jsondecode(fileread(resultFile));
    fprintf('参与统计 %d 组\n', numel(js.groups));
    fprintf('加权平均 t = [%+.4f, %+.4f, %+.4f] m\n', js.t_weighted_mean(1), js.t_weighted_mean(2), js.t_weighted_mean(3));
    fprintf('中位数     t = [%+.4f, %+.4f, %+.4f] m\n', js.t_median(1), js.t_median(2), js.t_median(3));
    fprintf('标准差     σ = [%.4f, %.4f, %.4f] m\n', js.sigma(1), js.sigma(2), js.sigma(3));
    fprintf('\n单组 t 值:\n');
    for g = js.groups(:)'
        fprintf('  %-35s t=[%+.3f, %+.3f, %+.3f]  残差=%.3fm\n', g.name, g.t(1), g.t(2), g.t(3), g.resid);
    end
else
    fprintf('没找到 tall_result.json，可能 solve_t_all 没成功\n');
end

fprintf('\n完成。\n');

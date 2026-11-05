function P = paths()
%PATHS 唯一需要你改的地方：采集数据在哪。
%
% 3.0 包内的所有东西（算法、CFAR 的 MEX、hamming、结果目录）全部用相对路径，
% 整包搬到别的机器、别的盘符都不用动。唯一依赖外部的是"采集数据在哪"，
% 因为那是采集程序产出的、可以放在任何地方。
%
%   P = paths();
%   P.captureTakes   % ← 只改这里（采集数据目录）
%   P.resultsDir     % 3.0\results
%   P.takesDir       % 3.0\results\takes   （*_res.mat 放这里）
%   P.libDir         % 3.0\matlab\algorithm\lib（MEX、hamming 等依赖，包内）

here = fileparts(mfilename('fullpath'));          % ...\3.0\matlab
P.matlabDir = here;
P.root30 = fileparts(here);                       % ...\3.0
P.resultsDir = fullfile(P.root30, 'results');
P.takesDir = fullfile(P.resultsDir, 'takes');
P.algorithmDir = fullfile(here, 'algorithm');
P.libDir = fullfile(P.algorithmDir, 'lib');

% ============================================================
% 采集数据目录：改这一行（或者直接改下面的候选列表）
% ============================================================
P.captureTakes = fullfile(P.root30, '..', '11-4');
% ============================================================

% 找不到就按几个常见布局自动猜，省得每台机器都手改
if ~isfolder(P.captureTakes)
    guesses = { ...
        fullfile(P.root30, '..', 'capture', 'takes'), ...      % 3.0 和 capture 并排
        fullfile(P.root30, '..', '..', 'capture', 'takes'), ...% 3.0 在 RealtimeOnPC 里
        fullfile(P.root30, 'capture', 'takes')};               % capture 在 3.0 里面
    for i = 1:numel(guesses)
        if isfolder(guesses{i})
            P.captureTakes = guesses{i};
            break;
        end
    end
end

if ~isfolder(P.takesDir), mkdir(P.takesDir); end
end

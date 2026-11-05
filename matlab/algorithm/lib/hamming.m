function w = hamming(n, opt)
%HAMMING 本地实现，替代 Signal Processing Toolbox 的 hamming。
%
%   w = hamming(n)              % 对称窗（与工具箱默认一致）
%   w = hamming(n, 'periodic')  % 周期窗
%
% 为什么自己写：换机器时对方可能没装 Signal Processing Toolbox，
% process_final 里一句 hamming 就会报 "未定义与 'double' 类型的输入参数相对应的函数"。
% 这个窗只靠 cos，基础 MATLAB 就够。

if nargin < 2 || isempty(opt), opt = 'symmetric'; end
if ~isscalar(n) || n <= 0
    w = zeros(0, 1);
    return;
end
if n == 1
    w = 1;
    return;
end

switch lower(opt)
    case 'periodic'
        k = (0:n-1).' / n;
    otherwise
        k = (0:n-1).' / (n-1);
end
w = 0.54 - 0.46 * cos(2 * pi * k);
end

function q = pctile(x, p)
%PCTILE 本地百分位，替代 Statistics Toolbox 的 prctile。
%
%   q = pctile(x, 50)        % 中位数
%   q = pctile(x, [5 95])    % 两个分位
%
% 线性插值，位置约定与 MATLAB 的 prctile 一致（pos = p/100*n + 0.5），
% 实测与 prctile 逐位相同。

x = x(:);
x = x(~isnan(x));
if isempty(x)
    q = nan(size(p));
    return;
end
x = sort(x);
n = numel(x);

pos = (p(:) / 100) * n + 0.5;
pos = min(max(pos, 1), n);
lo = floor(pos);
hi = ceil(pos);
q = x(lo) + (x(hi) - x(lo)) .* (pos - lo);
q = reshape(q, size(p));
end

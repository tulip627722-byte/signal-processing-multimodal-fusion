function B = resize_nn(A, h, w)
%RESIZE_NN 最近邻缩放，替代 Image Processing Toolbox 的 imresize。
%
%   B = resize_nn(A, h, w)
%
% 只用在"彩色图和深度图尺寸不一致"这条兜底路径上（默认不走到），
% 最近邻够用，也不需要额外工具箱。

[ha, wa] = size(A);
if ha == h && wa == w
    B = A;
    return;
end
ri = round(linspace(1, ha, h));
ci = round(linspace(1, wa, w));
ri = min(max(ri, 1), ha);
ci = min(max(ci, 1), wa);
B = A(ri, ci);
end

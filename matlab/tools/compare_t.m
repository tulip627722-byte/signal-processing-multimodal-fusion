function compare_t(t_use)
%COMPARE_T 用指定 t 跑全部组，对比逐组解 vs 固定 t 的效果

cd('G:\3.2\matlab'); addpath('tools','algorithm','algorithm/lib');
P = paths(); K = [491.4 655.7 319.5 239.5];
js = jsondecode(fileread(fullfile(P.resultsDir, 'tall_result.json')));
files = dir(fullfile(P.takesDir, '*_res.mat'));

fprintf('\n=== 用固定 t=[%.3f %.3f %.3f] 重跑所有组 ===\n', t_use);
fprintf('%-35s %8s | %8s %8s %8s | %8s %8s %8s | %8s\n', ...
    '组', '姿态来源', 'yaw', 'pitch', 'roll', '中位cm', '绿', '黄', '红');
fprintf('%s\n', repmat('-', 1, 120));

for i = 1:numel(files)
    nm = strrep(files(i).name, '_res.mat', '');
    if contains(nm, '无板'), continue; end
    S = load(fullfile(P.takesDir, files(i).name));
    k = min(numel(S.frames), 10);
    Pts = double(S.pts{k});

    df = fullfile(S.metaOut.takeDir, 'depth', sprintf('%06d.png', S.depthFrame(k)));
    D = imread(df); if size(D,3)>1, D=D(:,:,1); end
    D = fliplr(double(D)) * 0.001; D(D < 0.3 | D > 9) = NaN;

    % 强制用 t_use + solve_t_all 解的姿态
    prm = struct('tx', t_use(1), 'ty', t_use(2), 'tz', t_use(3), ...
                 'yaw', 0, 'pitch', 0, 'roll', 0);
    for gi = 1:numel(js.groups)
        if strcmp(js.groups(gi).name, nm)
            prm.yaw = js.groups(gi).yaw; prm.pitch = js.groups(gi).pitch; prm.roll = js.groups(gi).roll;
            break;
        end
    end

    Rm = Rz(prm.yaw) * Rx(prm.pitch) * Ry(prm.roll);
    Pc = (Pts(:,1:3) - [prm.tx prm.ty prm.tz]) * Rm;
    xo = Pc(:,1); yo = -Pc(:,3); zo_all = Pc(:,2);
    ok = zo_all > 0.25;
    u = nan(size(ok)); v = nan(size(ok));
    u(ok) = K(1) .* xo(ok) ./ zo_all(ok) + K(3);
    v(ok) = K(2) .* yo(ok) ./ zo_all(ok) + K(4);
    ok = ok & u >= 1 & u <= 640 & v >= 1 & v <= 480;

    resid_cm = nan(size(ok));
    if any(ok)
        dv = interp2(D, u(ok), v(ok), 'linear', NaN);
        resid_cm(ok) = abs(zo_all(ok) - dv) * 100;
    end
    valid = ok & isfinite(resid_cm) & resid_cm < 100 & zo_all > 0.3;

    n_g = nnz(valid & resid_cm <= 15);
    n_y = nnz(valid & resid_cm > 15 & resid_cm <= 30);
    n_r = nnz(valid & resid_cm > 30);
    md = median(resid_cm(valid & isfinite(resid_cm)));

    fprintf('%-35s %-8s | %+7.2f %+7.2f %+7.2f | %7.2f %5d %5d %5d\n', ...
        nm, '固定t', prm.yaw, prm.pitch, prm.roll, md, n_g, n_y, n_r);
end

    function R = Rz(a), s=sind(a); c=cosd(a); R=[c -s 0; s c 0; 0 0 1]; end
    function R = Ry(a), s=sind(a); c=cosd(a); R=[c 0 s; 0 1 0; -s 0 c]; end
    function R = Rx(a), s=sind(a); c=cosd(a); R=[1 0 0; 0 c -s; 0 s c]; end
end

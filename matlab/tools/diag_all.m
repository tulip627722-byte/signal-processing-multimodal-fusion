function diag_all()
%DIAG_ALL 诊断所有组在 15cm 阈值下的绿/黄/红分布

cd('G:\3.2\matlab'); addpath('tools','algorithm','algorithm/lib');
P = paths(); K = [491.4 655.7 319.5 239.5];
js = jsondecode(fileread(fullfile(P.resultsDir, 'tall_result.json')));
files = dir(fullfile(P.takesDir, '*_res.mat'));
Kthr = 15;

fprintf('%-35s %8s %8s %8s %8s %8s %8s | %8s %8s %8s | %8s\n', ...
    '组', 't[1]', 't[2]', 't[3]', 'yaw', 'pitch', 'roll', ...
    '残差', '深度', '中位cm', '绿', '黄', '红', '总有效');
fprintf('%s\n', repmat('-', 1, 150));

for i = 1:numel(files)
    nm = strrep(files(i).name, '_res.mat', '');
    if contains(nm, '无板'), continue; end
    S = load(fullfile(P.takesDir, files(i).name));
    k = min(numel(S.frames), 10);
    Pts = double(S.pts{k});

    df = fullfile(S.metaOut.takeDir, 'depth', sprintf('%06d.png', S.depthFrame(k)));
    D = imread(df); if size(D,3)>1, D=D(:,:,1); end
    D = fliplr(double(D)) * 0.001; D(D < 0.3 | D > 9) = NaN;

    % 取 solve_t_all 解
    prm = struct('tx', 0.008, 'ty', -0.048, 'tz', 0.130, 'yaw', 0, 'pitch', 0, 'roll', 0);
    found = false;
    for gi = 1:numel(js.groups)
        if strcmp(js.groups(gi).name, nm)
            prm.tx = js.groups(gi).t(1); prm.ty = js.groups(gi).t(2); prm.tz = js.groups(gi).t(3);
            prm.yaw = js.groups(gi).yaw; prm.pitch = js.groups(gi).pitch; prm.roll = js.groups(gi).roll;
            found = true; break;
        end
    end
    if ~found, continue; end

    % 投影 + 残差
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

    n_green = nnz(valid & resid_cm <= Kthr);
    n_yellow = nnz(valid & resid_cm > Kthr & resid_cm <= 2*Kthr);
    n_red = nnz(valid & resid_cm > 2*Kthr);
    md = median(resid_cm(valid & isfinite(resid_cm)));
    dMax = max(D(isfinite(D)));
    resid_group = 0;
    for gi = 1:numel(js.groups)
        if strcmp(js.groups(gi).name, nm), resid_group = js.groups(gi).resid; break; end
    end

    flag = '';
    if n_yellow > n_green * 0.3, flag = ' ⚠️ Y>30%'; end
    if resid_group > 0.2, flag = ' ⚠️ 解差'; end

    fprintf('%-35s %+8.3f %+8.3f %+8.3f | %+7.2f %+7.2f %+7.2f | %7.3f %6.2f %7.2f | %5d %5d %5d %5d%s\n', ...
        nm, prm.tx, prm.ty, prm.tz, prm.yaw, prm.pitch, prm.roll, ...
        resid_group, dMax, md, n_green, n_yellow, n_red, nnz(valid), flag);
end

    function R = Rz(a), s=sind(a); c=cosd(a); R=[c -s 0; s c 0; 0 0 1]; end
    function R = Ry(a), s=sind(a); c=cosd(a); R=[c 0 s; 0 1 0; -s 0 c]; end
    function R = Rx(a), s=sind(a); c=cosd(a); R=[1 0 0; 0 c -s; 0 s c]; end
end

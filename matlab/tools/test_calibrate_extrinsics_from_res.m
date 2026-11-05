function result = test_calibrate_extrinsics_from_res()
%TEST_CALIBRATE_EXTRINSICS_FROM_RES 合成点验证外参候选求解器。
rng(42, 'twister');
R0 = [1 0 0;0 0 1;0 -1 0];
angles = [3.2 -4.5 2.4];
t = [0.025 -0.018 0.105];
bias = 0.14;
camera = [-0.35 0.35 1.8; 0.30 0.40 2.0; -0.25 0.60 2.5; ...
           0.32 0.52 2.8; -0.10 0.25 3.2; 0.18 0.68 3.6; 0.05 0.45 4.0];
R = local_rotation(angles)*R0;
trueRadar = (R*camera.').'+t;
measured = trueRadar .* ((vecnorm(trueRadar,2,2)+bias)./vecnorm(trueRadar,2,2));
candidates = cell(size(camera,1),1);
for i=1:size(camera,1)
    good = measured(i,:)+0.008*randn(1,3);
    bad = good+[0.35 -0.25 0.30]+0.03*randn(1,3);
    candidates{i}=struct('center',{bad,good},'confidence',{0.65,0.95});
end
solution=solve_extrinsics_candidates(camera,candidates,struct('multiStarts',18,'randomSeed',42));
angleError=max(abs(solution.anglesDegRadarXYZ-angles));
tError=norm(solution.t-t);
biasError=abs(solution.rangeBiasM-bias);
result=struct('pass',angleError<=1 && tError<=0.03 && biasError<=0.02, ...
    'angleErrorDeg',angleError,'translationErrorM',tError,'biasErrorM',biasError, ...
    'solution',solution);
fprintf('合成测试：角度 %.3f deg，平移 %.3f m，测距偏差 %.3f m，%s\n', ...
    angleError,tError,biasError,pass_text(result.pass));
fprintf('  真值角度 [%+.3f %+.3f %+.3f]，估计 [%+.3f %+.3f %+.3f]\n', ...
    angles,result.solution.anglesDegRadarXYZ);
if ~result.pass, error('合成外参测试未通过。'); end
end

function R=local_rotation(a)
rx=deg2rad(a(1));ry=deg2rad(a(2));rz=deg2rad(a(3));
Rx=[1 0 0;0 cos(rx) -sin(rx);0 sin(rx) cos(rx)];
Ry=[cos(ry) 0 sin(ry);0 1 0;-sin(ry) 0 cos(ry)];
Rz=[cos(rz) -sin(rz) 0;sin(rz) cos(rz) 0;0 0 1];
R=Rz*Ry*Rx;
end

function s=pass_text(v)
if v,s='PASS';else,s='FAIL';end
end

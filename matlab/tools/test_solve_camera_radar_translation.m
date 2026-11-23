function result = test_solve_camera_radar_translation()
%TEST_SOLVE_CAMERA_RADAR_TRANSLATION Recover a known translation.
rng(22,'twister'); rotation=[1 0 0;0 0 1;0 -1 0];
camera=[-.4 .1 2.0;.3 .2 2.8;-.2 -.1 3.7;.45 .05 4.8;-.35 .15 6.0;.15 -.2 7.0];
truth=[.018 -.014 .108]; radar=(rotation*camera.').'+truth+0.002*randn(size(camera));
solution=solve_camera_radar_translation(camera,radar,rotation,[0 0 .10]);
errorM=norm(solution.translation-truth);
result=struct('pass',errorM<=.01,'errorM',errorM,'solution',solution);
fprintf('相机/雷达平移合成测试：误差 %.2f mm，%s\n',errorM*1000,pass_text(result.pass));
if ~result.pass,error('相机/雷达平移合成测试未通过。');end
end

function text=pass_text(value)
if value,text='PASS';else,text='FAIL';end
end

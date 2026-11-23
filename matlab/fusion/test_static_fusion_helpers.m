function result = test_static_fusion_helpers()
%TEST_STATIC_FUSION_HELPERS Basic transform, voxel, and FOV smoke test.
addpath(fileparts(mfilename('fullpath')),'-begin');
points=[0 1 2;1 2 3;1.01 2.01 3.01];[reduced,attrs]=voxel_reduce_points(points,[1;2;3],.1);
transformed=transform_points([0 0 1],[1 0 0;0 0 1;0 -1 0],[.1 .2 .3]);
result=struct('pass',size(reduced,1)==2 && norm(transformed-[.1 1.2 .3])<1e-9,'reducedCount',size(reduced,1));
fprintf('fusion helper test: %s\n',ternary(result.pass,'PASS','FAIL'));if ~result.pass,error('fusion helper test failed');end
end
function value=ternary(condition,yesValue,noValue),if condition,value=yesValue;else,value=noValue;end;end

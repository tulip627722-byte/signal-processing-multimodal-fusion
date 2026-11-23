function result = refine_depth_lidar_extrinsics(data, diagnosis, opts)
%REFINE_DEPTH_LIDAR_EXTRINSICS Optional candidate-only point-to-plane ICP.
result=struct('enabled',true,'status','NOT_AVAILABLE','note','');
if exist('pcregistericp','file')~=2 || exist('pointCloud','class')~=8
    result.note='未检测到 pointCloud/pcregistericp，跳过 ICP；保留现有外参。'; return;
end
depth=pointCloud(data.depth.radar); lidar=pointCloud(data.lidar.radar);
depth=pcdownsample(depth,'gridAverage',opts.depthVoxelM); lidar=pcdownsample(lidar,'gridAverage',opts.lidarVoxelM);
depth=pcnormals(depth,10); lidar=pcnormals(lidar,10);
[~,tform,rmse]=pcregistericp(depth,lidar,'Metric','pointToPlane', ...
    'MaxIterations',opts.refinementIterations,'InlierRatio',.7);
result.status='CANDIDATE_ONLY'; result.tform=tform; result.rmseM=rmse;
result.inputDiagnosis=diagnosis; result.note='ICP 结果仅保存为候选，不自动改正式外参。';
end

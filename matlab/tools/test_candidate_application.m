function result=test_candidate_application()
%TEST_CANDIDATE_APPLICATION 验证 FAIL 候选不能被计算链误用。
here=fileparts(mfilename('fullpath'));root32=fileparts(fileparts(here));
candidate=fullfile(root32,'results', ...
 'extrinsics_calibration','radar_camera_extrinsics_candidate.json');
E=load_radar_camera_extrinsics(candidate);
result=struct('exists',isfile(candidate),'applicable',logical(E.applicable), ...
 'rejected',false,'message','');
try
 take=fullfile(fileparts(root32),'11-4','20261104_162154_室内4米');
 compute_take(take,struct('extrinsicsFile',candidate,'maxFrames',1));
catch ex
 result.rejected=contains(ex.message,'未通过质检');result.message=ex.message;
end
fprintf('FAIL 候选应用保护：%s\n',pass_text(result.exists&&~result.applicable&&result.rejected));
if ~(result.exists&&~result.applicable&&result.rejected),error('FAIL 候选未被正确拒绝。');end
end

function s=pass_text(v)
if v,s='PASS';else,s='FAIL';end
end

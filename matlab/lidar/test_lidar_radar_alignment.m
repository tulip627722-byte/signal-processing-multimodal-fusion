function result=test_lidar_radar_alignment()
%TEST_LIDAR_RADAR_ALIGNMENT Synthetic regression for the radar-centred fit.
rng(70,'twister'); angles=deg2rad([4 -6 2.5]);
Rx=[1 0 0;0 cos(angles(1)) -sin(angles(1));0 sin(angles(1)) cos(angles(1))];
Ry=[cos(angles(2)) 0 sin(angles(2));0 1 0;-sin(angles(2)) 0 cos(angles(2))];
Rz=[cos(angles(3)) -sin(angles(3)) 0;sin(angles(3)) cos(angles(3)) 0;0 0 1];R=Rz*Ry*Rx;t=[.08 -.04 .12];
L=[-.5 -.3 1.8;.4 -.2 2;-.3 .4 2.4;.5 .35 2.8;-.6 .15 3.2;.25 -.45 3.6;.65 .1 4;-.2 -.5 4.4;.1 .55 4.8];
Q=(R*L.').'+t+.003*randn(size(L));Q(3,:)=Q(3,:)+[.12 -.08 .06];split=repmat("fit",size(L,1),1);split(end-1:end)="holdout";
T=table(L(:,1),L(:,2),L(:,3),Q(:,1),Q(:,2),Q(:,3),ones(size(split)),split, ...
 'VariableNames',{'lidar_x','lidar_y','lidar_z','radar_x','radar_y','radar_z','weight','split'});
report=solve_lidar_radar_extrinsics(T,struct('acceptanceP95M',.03));
rotationErrorDeg=rad2deg(acos(max(-1,min(1,(trace(report.extrinsics.lidarToRadarR*R.')-1)/2))));
translationErrorM=norm(report.extrinsics.lidarOriginInRadarM-t);result=struct('pass',rotationErrorDeg<.4&&translationErrorM<.015, ...
 'rotationErrorDeg',rotationErrorDeg,'translationErrorM',translationErrorM,'report',report);
fprintf('Synthetic LiDAR->radar: rotation %.3f deg, translation %.1f mm, %s\n',rotationErrorDeg,translationErrorM*1000,pass_text(result.pass));
if ~result.pass,error('Synthetic LiDAR/radar alignment failed.');end
end
function value=pass_text(passed)
if passed,value='PASS';else,value='FAIL';end
end

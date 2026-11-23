function report = run_static_fusion_demo(takeDir, opts)
%RUN_STATIC_FUSION_DEMO One-command static three-modal fusion demo.
if nargin < 1 || isempty(takeDir), error('请提供静态 take 目录。'); end
if nargin < 2, opts = struct(); end
here=fileparts(mfilename('fullpath')); o=fusion_defaults(opts,fileparts(here));
addpath(fileparts(here), '-begin'); addpath(here, '-begin'); addpath(fullfile(fileparts(here), 'lidar'), '-begin');
if ~isfolder(o.outputDir), mkdir(o.outputDir); end
data=preprocess_static_take(takeDir,o); diagnosis=diagnose_depth_lidar_alignment(data,o);
if o.runRefinement, refinement=refine_depth_lidar_extrinsics(data,diagnosis,o); else, refinement=struct('enabled',false,'status','NOT_RUN'); end
[fusedPc,associations]=fuse_camera_fov_pointclouds(data,o);
[planeAnalysis]=analyze_board_plane_alignment(data,associations,o);
[~,name]=fileparts(takeDir); base=fullfile(o.outputDir,name);
save([base '_fused.mat'],'fusedPc','associations','data','diagnosis','refinement','planeAnalysis','-v7.3');
if o.savePly, write_fusion_ply([base '_fused.ply'],fusedPc); end
plot_fusion_views(data,fusedPc,associations,diagnosis,o,[base '_overview.png']);
report=struct('takeDir',takeDir,'outputBase',base,'summary',data.summary,'diagnosis',diagnosis,'refinement',refinement,'planeAnalysis',planeAnalysis,'fusedPc',fusedPc,'associations',associations,'data',data);
save([base '_report.mat'],'report','-v7.3');
if planeAnalysis.valid, fprintf('板面法向误差：中位 %.0f mm，P95 %.0f mm，平面内点 %.1f%%\n',planeAnalysis.normal_median_m*1000,planeAnalysis.normal_p95_m*1000,100*planeAnalysis.inlier_ratio); end
if isfield(planeAnalysis,'boardCandidateClusterIndices'), fprintf('板面候选雷达簇：%s\n',mat2str(planeAnalysis.boardCandidateClusterIndices)); end
fprintf('静态融合完成：%s\n',base); fprintf('Depth=%d, MID-70=%d, Radar=%d（镜头内）\n',data.summary.depthPoints,size(data.lidar.radar,1),size(data.radar.xyz,1)); fprintf('Depth→MID-70 最近邻中位 %.1f mm，P95 %.1f mm\n',diagnosis.before.medianM*1000,diagnosis.before.p95M*1000);
end

function write_fusion_ply(path,pc)
file=fopen(path,'w');if file<0,error('无法写入 %s',path);end;cleanup=onCleanup(@()fclose(file));n=size(pc.xyz,1);
fprintf(file,'ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nproperty uchar sensor_id\nend_header\n',n);
for i=1:n,fprintf(file,'%.6f %.6f %.6f %d %d %d %d\n',pc.xyz(i,:),pc.rgb(i,:),pc.sensor_id(i));end
end

function plot_fusion_views(data,pc,associations,diagnosis,opts,path)
limits=display_limits(opts.displayProfile);figure('Visible','off','Color','w','Position',[30 30 1600 720]);tiledlayout(1,2);
nexttile;hold on;D=display_sample(data.depth.radar,limits(1));L=display_sample(data.lidar.radar,limits(2));
scatter3(D(:,1),D(:,2),D(:,3),2,[.1 .65 .3],'filled','MarkerFaceAlpha',.35);scatter3(L(:,1),L(:,2),L(:,3),2,[.1 .35 .95],'filled','MarkerFaceAlpha',.28);scatter3(data.radar.xyz(:,1),data.radar.xyz(:,2),data.radar.xyz(:,3),16,[1 .55 .4],'filled','MarkerFaceAlpha',.55);draw_associations(data.radar.clusters,associations);grid on;axis equal;view(-35,22);title(sprintf('统一雷达坐标 | 持续点 %d / 簇心 %d',size(data.radar.xyz,1),numel(data.radar.clusters)));xlabel('X');ylabel('Y');zlabel('Z');
nexttile;hold on;colors=[.1 .65 .3;.1 .35 .95;1 .55 .4];handles=gobjects(1,3);for id=1:3,m=find(pc.sensor_id==id);m=display_indices(m,limits(id));handles(id)=scatter3(pc.xyz(m,1),pc.xyz(m,2),pc.xyz(m,3),2,repmat(colors(id,:),numel(m),1),'filled','MarkerFaceAlpha',ternary(id==3,.6,.32));end;associationHandles=draw_associations(data.radar.clusters,associations);grid on;axis equal;view(-35,22);title(sprintf('镜头下融合 | NN中位 %.0f mm | %s密度',diagnosis.before.medianM*1000,opts.displayProfile));xlabel('X');ylabel('Y');zlabel('Z');legend([handles associationHandles],{'Depth','MID-70','雷达持续点','雷达簇心','关联几何中心'},'Location','southoutside','Orientation','horizontal');exportgraphics(gcf,path,'Resolution',160);close(gcf);
end

function handles=draw_associations(clusters,associations)
handles=[plot3(nan,nan,nan,'rp','MarkerFaceColor','r','MarkerSize',12) plot3(nan,nan,nan,'ko','MarkerSize',9,'LineWidth',1.4)];
for index=1:numel(clusters)
    radar=clusters(index).centroid;
    plot3(radar(1),radar(2),radar(3),'rp','MarkerFaceColor','r','MarkerSize',12);
    text(radar(1),radar(2),radar(3),sprintf('  C%d',index),'Color',[.65 0 0],'FontWeight','bold');
end
if ~isfield(associations,'cluster')||isempty(associations.cluster),return;end
for index=1:numel(associations.cluster)
    item=associations.cluster(index);r=item.radar_centroid;g=item.geometry_centroid;
    plot3(g(1),g(2),g(3),'ko','MarkerSize',9,'LineWidth',1.4);plot3([r(1) g(1)],[r(2) g(2)],[r(3) g(3)],'k--','LineWidth',1);text(r(1),r(2),r(3),sprintf('  C%d %.0fmm',item.cluster_index,item.error_norm_m*1000),'Color',[.65 0 0],'FontWeight','bold');
end
end

function limits=display_limits(profile)
switch lower(char(profile)),case 'sparse',limits=[8000 8000 200];case 'fine',limits=[35000 35000 500];otherwise,limits=[15000 15000 300];end
end
function points=display_sample(points,maximum),indices=display_indices((1:size(points,1)).',maximum);points=points(indices,:);end
function indices=display_indices(indices,maximum),indices=indices(:);if numel(indices)>maximum,indices=indices(unique(round(linspace(1,numel(indices),maximum))));end;end
function value=ternary(condition,a,b),if condition,value=a;else,value=b;end;end

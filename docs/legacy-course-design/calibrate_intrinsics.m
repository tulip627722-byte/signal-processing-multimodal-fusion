%% 相机内参标定：两组视频棋盘格
% 需要 Computer Vision Toolbox
% 实际检测结果：棋盘格内角点为 7x7（以 MATLAB 检测结果为准）
% 单格边长：5 cm = 50 mm

clear; close all; clc;

%% 1. 路径和参数
videoFiles = {
    'E:\xinhaochuli\savedata\Camera\20261014_163316_camera.mp4'
    'E:\xinhaochuli\savedata\Camera\20261014_163615_camera.mp4'
};

outDir = 'E:\xinhaochuli\calibration_frames';
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

squareSize = 50;       % mm
innerCorners = [7 7];  % 实测内角点为7x7
framesPerVideo = 30;   % 每个视频均匀抽取30帧作为候选

%% 2. 从两个视频均匀抽帧
imageFiles = {};
for v = 1:numel(videoFiles)
    if ~isfile(videoFiles{v})
        error('找不到视频文件：%s', videoFiles{v});
    end

    vr = VideoReader(videoFiles{v});
    nFrames = max(1, floor(vr.Duration * vr.FrameRate));
    sampleIdx = round(linspace(1, nFrames, framesPerVideo));
    sampleIdx = unique(sampleIdx);

    [~, videoName, ~] = fileparts(videoFiles{v});
    for k = 1:numel(sampleIdx)
        vr.CurrentTime = max(0, (sampleIdx(k)-1) / vr.FrameRate);
        if ~hasFrame(vr)
            continue;
        end
        frame = readFrame(vr);
        outName = fullfile(outDir, sprintf('%s_%03d.jpg', videoName, k));
        imwrite(frame, outName, 'Quality', 95);
        imageFiles{end+1,1} = outName; %#ok<SAGROW>
    end
end

fprintf('候选图像数量：%d\n', numel(imageFiles));

%% 3. 检测棋盘格角点
% MATLAB 会自动返回实际检测到的棋盘格尺寸。
[imagePoints, detectedBoardSize, imagesUsed] = ...
    detectCheckerboardPoints(imageFiles);

fprintf('检测到角点的图像数量：%d / %d\n', ...
    sum(imagesUsed), numel(imageFiles));
fprintf('检测到的内角点尺寸：%d x %d\n', ...
    detectedBoardSize(1), detectedBoardSize(2));

if detectedBoardSize(1) ~= innerCorners(1) || ...
        detectedBoardSize(2) ~= innerCorners(2)
    warning(['程序检测到的内角点尺寸不是预期的 6x6。' ...
        '请根据实际棋盘格修改 innerCorners，并重新运行。']);
end

validFiles = imageFiles(imagesUsed);
if numel(validFiles) < 12
    error('有效棋盘格图像少于12张，请补采或调整抽帧范围。');
end

%% 4. 读取图像尺寸并生成世界坐标
I = imread(validFiles{1});
imageSize = [size(I,1), size(I,2)];
worldPoints = generateCheckerboardPoints(detectedBoardSize, squareSize);

%% 5. 计算相机内参和畸变参数
[cameraParams, imagesUsed2, estimationErrors] = estimateCameraParameters( ...
    imagePoints, worldPoints, ...
    'ImageSize', imageSize, ...
    'EstimateSkew', false, ...
    'EstimateTangentialDistortion', true, ...
    'NumRadialDistortionCoefficients', 2, ...
    'WorldUnits', 'mm');

fprintf('\n===== 标定结果 =====\n');
disp('相机内参矩阵 K：');
disp(cameraParams.IntrinsicMatrix');
disp('径向畸变系数：');
disp(cameraParams.RadialDistortion);
disp('切向畸变系数：');
disp(cameraParams.TangentialDistortion);
fprintf('平均重投影误差：%.4f 像素\n', cameraParams.MeanReprojectionError);
fprintf('最大重投影误差：%.4f 像素\n', max(vecnorm(cameraParams.ReprojectionErrors, 2, 2), [], 'all'));

%% 6. 显示角点和重投影误差
figure('Name','棋盘格角点检测');
showReprojectionErrors(cameraParams);
grid on;

figure('Name','相机外参姿态');
showExtrinsics(cameraParams);
grid on;

% 显示部分检测结果
figure('Name','检测到的棋盘格角点');
showDetectedCorners = min(12, numel(validFiles));
montage(validFiles(1:showDetectedCorners), 'Size', [3 4]);
title('有效棋盘格图像');

%% 7. 保存结果
save(fullfile(outDir, 'camera_intrinsics_result.mat'), ...
    'cameraParams', 'estimationErrors', 'imagePoints', ...
    'worldPoints', 'validFiles', 'detectedBoardSize', 'squareSize');

% 同时导出一个可读文本
fid = fopen(fullfile(outDir, 'camera_intrinsics_summary.txt'), 'w');
fprintf(fid, '有效图像数量: %d\n', sum(imagesUsed2));
fprintf(fid, '图像尺寸: %d x %d\n', imageSize(2), imageSize(1));
fprintf(fid, '棋盘格内角点: %d x %d\n', detectedBoardSize(1), detectedBoardSize(2));
fprintf(fid, '单格尺寸: %.2f mm\n', squareSize);
fprintf(fid, '平均重投影误差: %.6f pixel\n', cameraParams.MeanReprojectionError);
fprintf(fid, '相机内参矩阵 K:\n');
fprintf(fid, '%.10f %.10f %.10f\n', cameraParams.IntrinsicMatrix');
fprintf(fid, '径向畸变: ');
fprintf(fid, '%.10f ', cameraParams.RadialDistortion);
fprintf(fid, '\n切向畸变: ');
fprintf(fid, '%.10f ', cameraParams.TangentialDistortion);
fprintf(fid, '\n');
fclose(fid);

fprintf('\n结果已保存到：%s\n', outDir);



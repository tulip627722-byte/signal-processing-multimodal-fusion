function summary = export_livox_take(takeDir, outputDir, opts)
%EXPORT_LIVOX_TAKE Convert saved raw Livox batches to PLY, MAT and index CSV.

if nargin < 2 || isempty(outputDir), outputDir = fullfile(takeDir, 'lidar_cloud'); end
if nargin < 3, opts = struct(); end
if ~isfield(opts, 'writePerBatch'), opts.writePerBatch = true; end
if ~isfield(opts, 'writeMerged'), opts.writeMerged = true; end
if ~isfield(opts, 'maxMergedPoints'), opts.maxMergedPoints = inf; end
inputDir = fullfile(takeDir, 'lidar');
files = dir(fullfile(inputDir, '*.bin'));
if isempty(files), error('No lidar/*.bin batches in %s', takeDir); end
if ~isfolder(outputDir), mkdir(outputDir); end

allPoints = cell(numel(files), 1);
allReflectivity = cell(numel(files), 1);
batch = zeros(numel(files), 1); firstT = zeros(numel(files), 1); lastT = zeros(numel(files), 1);
packets = zeros(numel(files), 1); points = zeros(numel(files), 1); source = strings(numel(files), 1);
for index = 1:numel(files)
    source(index) = string(fullfile('lidar', files(index).name));
    data = read_livox_bin(fullfile(files(index).folder, files(index).name));
    batch(index) = data.sequence; firstT(index) = data.firstRelMs; lastT(index) = data.lastRelMs;
    packets(index) = data.packetCount; points(index) = size(data.points, 1);
    allPoints{index} = data.points; allReflectivity{index} = data.reflectivity;
    if opts.writePerBatch
        write_ply(fullfile(outputDir, replace(files(index).name, '.bin', '.ply')), data.points, data.reflectivity);
    end
end
indexTable = table(batch, firstT, lastT, packets, points, source, ...
    'VariableNames', {'batch','first_t_rel_ms','last_t_rel_ms','packets','points','source'});
writetable(indexTable, fullfile(outputDir, 'index.csv'));
save(fullfile(outputDir, 'lidar_cloud.mat'), 'allPoints', 'allReflectivity', 'indexTable', '-v7.3');

mergedCount = 0;
if opts.writeMerged
    mergedPoints = vertcat(allPoints{:});
    mergedReflectivity = vertcat(allReflectivity{:});
    if size(mergedPoints, 1) > opts.maxMergedPoints
        pick = round(linspace(1, size(mergedPoints, 1), opts.maxMergedPoints));
        mergedPoints = mergedPoints(pick, :); mergedReflectivity = mergedReflectivity(pick);
    end
    write_ply(fullfile(outputDir, 'merged.ply'), mergedPoints, mergedReflectivity);
    mergedCount = size(mergedPoints, 1);
end
summary = struct('batches', numel(files), 'points', sum(points), ...
    'mergedPoints', mergedCount, 'outputDir', outputDir);
fprintf('Livox export: %d batches, %d points -> %s\n', summary.batches, summary.points, outputDir);
end

function write_ply(path, points, reflectivity)
fileId = fopen(path, 'wb', 'ieee-le');
if fileId < 0, error('Cannot write PLY: %s', path); end
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, ['ply\nformat binary_little_endian 1.0\n' ...
    'comment Livox frame X forward Y left Z up, metres\n' ...
    'element vertex %d\nproperty float x\nproperty float y\nproperty float z\n' ...
    'property uchar intensity\nend_header\n'], size(points, 1));
records = zeros(13, size(points, 1), 'uint8');
for axisIndex = 1:3
    bytes = typecast(single(points(:, axisIndex)), 'uint8');
    records((axisIndex-1)*4+(1:4), :) = reshape(bytes, 4, []);
end
records(13, :) = reshape(uint8(reflectivity), 1, []);
fwrite(fileId, records, 'uint8');
end

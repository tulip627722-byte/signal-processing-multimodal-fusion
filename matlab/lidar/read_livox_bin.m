function batch = read_livox_bin(path)
%READ_LIVOX_BIN Read LIVOXBIN V1/V2 point clouds or V3 lossless raw records.

fileId = fopen(path, 'rb', 'ieee-le');
if fileId < 0
    error('Cannot open LiDAR batch: %s', path);
end
cleanup = onCleanup(@() fclose(fileId));

magic = char(fread(fileId, 8, '*uint8').');
if ~strcmp(magic, 'LIVOXBIN')
    error('Invalid LIVOXBIN header: %s', path);
end

version = fread(fileId, 1, 'int32=>double');
if ~ismember(version, [1, 2, 3])
    error('Unsupported LIVOXBIN version %g: %s', version, path);
end

batch = struct('version', version, 'sequence', fread(fileId, 1, 'int32=>double'), ...
    'firstRelMs', fread(fileId, 1, 'double=>double'), 'lastRelMs', NaN, ...
    'hostTimestamp', int64(0), 'deviceTimestampRaw', int64(0), ...
    'timestampType', uint8(0), 'packetCount', 1, 'pointsMillimeters', [], ...
    'points', [], 'reflectivity', [], 'packets', table(), 'path', path);

if version >= 2
    batch.lastRelMs = fread(fileId, 1, 'double=>double');
    batch.hostTimestamp = fread(fileId, 1, 'int64=>int64');
    batch.deviceTimestampRaw = fread(fileId, 1, 'int64=>int64');
    batch.timestampType = fread(fileId, 1, 'uint8=>uint8');
    batch.packetCount = fread(fileId, 1, 'int32=>double');
else
    batch.lastRelMs = batch.firstRelMs;
    batch.deviceTimestampRaw = fread(fileId, 1, 'int64=>int64');
end

pointCount = fread(fileId, 1, 'int32=>double');
if isempty(pointCount) || pointCount < 0 || pointCount ~= floor(pointCount)
    error('Invalid LiDAR point count in %s', path);
end

if version == 3
    storedPacketCount = fread(fileId, 1, 'int32=>double');
    if isempty(storedPacketCount) || storedPacketCount < 0 || storedPacketCount ~= floor(storedPacketCount)
        error('Invalid LiDAR packet count in %s', path);
    end
    hostTimestamp = zeros(storedPacketCount, 1, 'int64');
    deviceTimestampRaw = zeros(storedPacketCount, 1, 'int64');
    timestampType = zeros(storedPacketCount, 1, 'uint8');
    relMs = zeros(storedPacketCount, 1);
    pointStart = zeros(storedPacketCount, 1);
    packetPointCount = zeros(storedPacketCount, 1);
    for index = 1:storedPacketCount
        hostTimestamp(index) = fread(fileId, 1, 'int64=>int64');
        deviceTimestampRaw(index) = fread(fileId, 1, 'int64=>int64');
        timestampType(index) = fread(fileId, 1, 'uint8=>uint8');
        relMs(index) = fread(fileId, 1, 'double=>double');
        pointStart(index) = fread(fileId, 1, 'int32=>double');
        packetPointCount(index) = fread(fileId, 1, 'int32=>double');
    end
    batch.packets = table(hostTimestamp, deviceTimestampRaw, timestampType, relMs, ...
        pointStart, packetPointCount, 'VariableNames', {'host_timestamp', ...
        'device_timestamp_raw', 'timestamp_type', 't_rel_ms', 'point_start_zero_based', 'point_count'});
    packed = fread(fileId, [13, pointCount], '*uint8');
    if size(packed, 2) ~= pointCount
        error('Truncated LiDAR batch: expected %d points in %s', pointCount, path);
    end
    batch.pointsMillimeters = [decode_int32(packed(1:4, :)), ...
        decode_int32(packed(5:8, :)), decode_int32(packed(9:12, :))];
    batch.points = batch.pointsMillimeters / 1000;
    batch.reflectivity = packed(13, :).';
else
    packed = fread(fileId, [13, pointCount], '*uint8');
    if size(packed, 2) ~= pointCount
        error('Truncated LiDAR batch: expected %d points in %s', pointCount, path);
    end
    batch.points = [decode_single(packed(1:4, :)), ...
        decode_single(packed(5:8, :)), decode_single(packed(9:12, :))];
    batch.pointsMillimeters = round(batch.points * 1000);
    batch.reflectivity = packed(13, :).';
    batch.packets = table(batch.hostTimestamp, batch.deviceTimestampRaw, batch.timestampType, ...
        batch.firstRelMs, 0, pointCount, 'VariableNames', {'host_timestamp', ...
        'device_timestamp_raw', 'timestamp_type', 't_rel_ms', 'point_start_zero_based', 'point_count'});
end
end

function values = decode_single(bytes)
values = double(typecast(reshape(bytes, [], 1), 'single'));
values = values(:);
end

function values = decode_int32(bytes)
values = double(typecast(reshape(bytes, [], 1), 'int32'));
values = values(:);
end

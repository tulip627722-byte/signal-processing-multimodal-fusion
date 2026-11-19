function test_inspect_lidar_radar_take()
%TEST_INSPECT_LIDAR_RADAR_TAKE Regression tests for Livox binary decoding.

path = [tempname '.bin'];
cleanup = onCleanup(@() delete_if_exists(path));
fileId = fopen(path, 'wb', 'ieee-le');
assert(fileId >= 0);
fileCleanup = onCleanup(@() fclose(fileId));
fwrite(fileId, uint8('LIVOXBIN'), 'uint8');
fwrite(fileId, int32(3), 'int32');
fwrite(fileId, int32(7), 'int32');
fwrite(fileId, 12.5, 'double');
fwrite(fileId, 13.0, 'double');
fwrite(fileId, int64(123456), 'int64');
fwrite(fileId, int64(987654), 'int64');
fwrite(fileId, uint8(1), 'uint8');
fwrite(fileId, int32(1), 'int32');
fwrite(fileId, int32(2), 'int32');
fwrite(fileId, int32(1), 'int32');
fwrite(fileId, int64(123456), 'int64');
fwrite(fileId, int64(987654), 'int64');
fwrite(fileId, uint8(1), 'uint8');
fwrite(fileId, 12.5, 'double');
fwrite(fileId, int32(0), 'int32');
fwrite(fileId, int32(2), 'int32');
write_point(fileId, 3166, 116, 23, 110);
write_point(fileId, -1000, 2000, -3000, 42);
clear fileCleanup;

batch = read_livox_bin(path);
assert(batch.version == 3);
assert(batch.sequence == 7);
assert(isequal(batch.pointsMillimeters, [3166 116 23; -1000 2000 -3000]));
assert(max(abs(batch.points(:) - [3.166; -1; 0.116; 2; 0.023; -3])) < 1e-12);
assert(isequal(batch.reflectivity, uint8([110; 42])));
assert(height(batch.packets) == 1);
assert(batch.packets.point_count(1) == 2);
fprintf('PASS: LIVOXBIN_V3 int32-mm XYZ and reflectivity decode correctly.\n');
end

function write_point(fileId, x, y, z, reflectivity)
fwrite(fileId, int32(x), 'int32');
fwrite(fileId, int32(y), 'int32');
fwrite(fileId, int32(z), 'int32');
fwrite(fileId, uint8(reflectivity), 'uint8');
end

function delete_if_exists(path)
if isfile(path), delete(path); end
end

function output = transform_points(points, rotation, translation)
%TRANSFORM_POINTS Apply column-vector rigid transform to N-by-3 rows.
points = double(points(:, 1:3));
output = (double(rotation) * points.').';
output = output + reshape(double(translation), 1, 3);
end

% Test voxel map class

%% Clear workspace and mex lib.
close all;
clear;
clear voxel_map;
% clc;

%% Initialize voxel map params.
voxel_size = 1;
free_update = -1;
hit_update = 1;
occupied_threshold = 0;

%% Test set and get voxels.
map = VoxelMap(voxel_size, free_update, hit_update, occupied_threshold);
[x, y, z] = ndgrid(voxel_size*1:100, voxel_size*1:10, voxel_size*1:10);
x = [x(:)'; y(:)'; z(:)'];
clear y z;
x = map.voxel_centers(map.voxels(x));
val = 1;
map.set_voxels(x, val);
assert(map.size() == size(x, 2)); 

[x2, val2] = map.get_voxels();
assert(all(all(unique(x', 'rows')' == unique(x2', 'rows')')));
assert(all(val2 == val));
clear x2 val2;

[x2, val2] = map.get_voxels(x);
assert(all(all(x == x2)));
assert(all(val2 == val));
clear x2 val2;

%% Test update lines.
map = VoxelMap(voxel_size, free_update, hit_update, occupied_threshold);
from = [0 0 0; 1 1 0]' + voxel_size/2;
to =   [0 0 3; 1 1 3]' + voxel_size/2;
from = map.voxel_centers(map.voxels(from));
to = map.voxel_centers(map.voxels(to));
t = tic();
map.update_lines(from, to);
fprintf('Updating %i lines: %.3f s\n', size(from, 2), toc(t));

t = tic();
[hit, val] = map.trace_lines(from, to);
fprintf('Tracing %i lines: %.3f s\n', size(from, 2), toc(t));
assert(all(all(hit == to)));

%% Test collapse.
map = VoxelMap(voxel_size, free_update, hit_update, occupied_threshold);
[x, y, z] = ndgrid(voxel_size*1:5, voxel_size*1:4, voxel_size*1:4);
x = -[x(:)'; y(:)'; z(:)'];
clear y z;
x = map.voxel_centers(map.voxels(x));
val = -1;
map.set_voxels(x, val);
assert(map.size() == size(x, 2)); 

map.collapse();
% assert(map.size() == 1+4*4);

% return;
%% Garbage follows...

%% Test update lines.
map = VoxelMap(voxel_size, free_update, hit_update, occupied_threshold);
% [x, y, z] = ndgrid(voxel_size*1:10, voxel_size*1:10, voxel_size*1:10);
[x, y, z] = ndgrid(voxel_size*1:100, voxel_size*1:100, zeros(1, 10));
from = [x(:)'; y(:)'; z(:)'];
clear x y z;
to = bsxfun(@plus, from, [0; 0; 10]);
from = map.voxel_centers(map.voxels(from));
to = map.voxel_centers(map.voxels(to));
t = tic();
map.update_lines(from, to);
fprintf('Updating %i lines: %.3f s\n', size(from, 2), toc(t));

%%
t = tic();
[hit, val] = map.trace_lines(from, to);
fprintf('Tracing %i lines: %.3f s\n', size(from, 2), toc(t));
assert(all(all(hit == to)));

%%
% [x, val] = map.get_voxels([], nan, nan);
% figure; plot3_color(x', val'); hold on; axis equal;

%%
map = VoxelMap(voxel_size, free_update, hit_update, occupied_threshold);
n = 10;
x = 1/2 * ones([3 n]);
x(2, :) = 1/2:n-1/2;
val = -ones([1 n]);
val(end) = 1;
map.set_voxels(x, val);
[x2, val2] = map.get_voxels(x, -inf, inf);

assert(all(all(x == x2)));
assert(all(val == val2));
clear x2 val2;

from = x(:, 1);
to = x(:, end);
dirs = to - from;
[x3, val3] = map.trace_rays(from, dirs);

assert(all(x3 == x(:, end)));
assert(val3 == 1);
clear x3 val3;

%% Test trace_lines
map = VoxelMap(voxel_size, free_update, hit_update, occupied_threshold);
[x, y, z] = ndgrid(-10:10, -10:10, -10:10);
x = [x(:)'; y(:)'; z(:)'];
clear y z;
x = map.voxel_centers(x);
val = -1;
map.set_voxels(x, val);
[x2, val2] = map.trace_lines([0 0 0]', x);
assert(all(all(x == x2)));
assert(all(val == val2));

%% Create an empty voxel map.
voxel_size = 0.1;
map = VoxelMap(voxel_size, free_update, hit_update, occupied_threshold);
[x, val] = map.get_voxels();
assert(isempty(x));
assert(isempty(val));

%% Create, shift and plot a half-sphere.
% n = 1e6;
% n = 320*240;
n = 640*480;
% Create a sphere of radius 1.
to = randn([3 n]);
to = bsxfun(@rdivide, to, sqrt(sum(to.^2)));
% Convert to half-sphere.
to(1, :) = -abs(to(1, :));
% Translate the sphere to 5-unit distance in x from origin.
to = bsxfun(@plus, to, [5 0 0]');
figure('Name', 'Source'); plot3(to(1,:), to(2,:), to(3,:), 'b.'); axis equal;
t = tic();
map.update_lines([0 0 0]', to);
fprintf('Updating map with %i lines: %.3f s.\n', n, toc(t));

%% Get all voxels.
t = tic();
[x, val] = map.get_voxels([], nan, nan);
fprintf('Getting %i voxels: %.3f s.\n', numel(val), toc(t));

%% Get all voxels within range.
t = tic();
[x, val] = map.get_voxels([], occupied_threshold, inf);
fprintf('Getting %i voxels: %.3f s.\n', numel(val), toc(t));
assert(all(val >= occupied_threshold & val <= inf));
figure('Name', 'Occupied voxels'); plot3(x(1,:), x(2,:), x(3,:), 'b.'); axis equal;

%% Query specific voxels, without range filter.
[~, val_2] = map.get_voxels(x);
assert(all(val == val_2));

%% Trace ray from the origin to the sphere.
max_range = 10;
dir = to;
t = tic();
[x, val] = map.trace_rays([0 0 0]', dir, max_range);
fprintf('Tracing %i rays up to range %.1f: %.3f s.\n', numel(val), max_range, toc(t));
x_occ = x(:, val >= occupied_threshold);
figure('Name', 'Hit occupied voxels'); plot3(x_occ(1,:), x_occ(2,:), x_occ(3,:), 'b.'); axis equal;

%% Trace rays and keep paths and vals therealong.
% close all;
t = tic();
% [x, val, path_x, path_val] = map.trace_rays([0 0 0]', dir, max_range);
origin = rand(size(dir))/1000;
% origin = zeros(size(dir));
[x, val, path_x, path_val] = map.trace_rays(origin, dir, max_range);
fprintf('Tracing %i ray: %.3f s\n', size(dir, 2), toc(t));
figure('Name', 'Number of voxels along path.');
hist(cellfun(@numel, path_val));

r = randi(n);
figure('Name', 'Single Ray');
plot3(origin(1,r), origin(2,r), origin(3,r), '+r');
hold on;
plot3([origin(1,r); origin(1,r) + dir(1,r)/10], ...
      [origin(2,r); origin(2,r) + dir(2,r)/10], ...
      [origin(3,r); origin(3,r) + dir(3,r)/10], '-b');
plot3(path_x{r}(1,:), path_x{r}(2,:), path_x{r}(3,:), '.-g');
plot3(to(1,:), to(2,:), to(3,:), '.k');
legend('Origin', 'Direction', 'Ray Voxels', 'Occupied');
axis equal;

%%
% close all;
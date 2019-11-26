
close all; clear;

voxel_size = 0.1;
free_update = -1;
hit_update = 2;
occupied_threshold = 1;

n = 100;
x = 100 * rand(3, n);
val = 10 * rand(1, n) - 5;

map = VoxelMap(voxel_size, free_update, hit_update, occupied_threshold);
map.set_voxels(x, val);
% Get voxel centers coordinates, and corresponding values.
% This should be saved.
[x, val] = map.get_voxels([], nan, nan);
% Get canonical order.
[~, idx] = sortrows(x');
x = x(:,idx);
val = val(idx);

f = [tempname '.mat'];
save(f, 'map');

% Overwrite and clear the previous map.
map.update_lines([0 0 0]', x);
clear map;

% Load original saved map.
loaded = load(f);
delete(f);
map = loaded.map;
clear loaded;

% Compare 
assert(isa(map, 'VoxelMap'));
assert(map.voxel_size == voxel_size);
assert(map.free_update == free_update);
assert(map.hit_update == hit_update);
assert(map.occupied_threshold == occupied_threshold);

[x2, val2] = map.get_voxels([], nan, nan);
% Get canonical order.
[~, idx] = sortrows(x2');
x2 = x2(:,idx);
val2 = val2(idx);

assert(all(x(:) == x2(:)));
assert(all(val(:) == val2(:)));

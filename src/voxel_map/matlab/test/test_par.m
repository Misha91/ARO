
close all; clear; clc;

voxel_size = 0.1;
map = VoxelMap(voxel_size);
map.set_voxels(rand(3, 10), rand(1, 10));
sz = map.size();

%%
map2 = map.copy();
assert(map2.size() == map.size());
map2.clear();
assert(map2.size() == 0);
assert(map.size() == sz);

%%
p = parpool('local', 2);
parfor i = 1:10
    fprintf('%i: size: %i, numel: %i\n', getfield(getCurrentTask(), 'ID'), ...
        map.size(), numel(map.get_voxels(rand(3, 10))));
    assert(sz == map.size());
end
delete(p);

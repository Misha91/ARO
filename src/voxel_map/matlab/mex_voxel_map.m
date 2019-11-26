function mex_voxel_map()
%MEX_VOXEL_MAP Build voxel map mex libs

cpp_dir = fileparts(mfilename('fullpath'));

inc_dir = fullfile(cpp_dir, '..', 'include');
out_dir = fullfile(cpp_dir, '..', 'matlab');

for f = {'voxel_map.cpp', 'hash_map.cpp'}
    mex('-largeArrayDims', ... %'-v', ...
        'COPTIMFLAGS=-O3 -DNDEBUG -fopenmp', ...
        'CXXOPTIMFLAGS=-O3 -DNDEBUG -fopenmp', ...
        omp_lib(), '-lboost_system', '-lboost_thread', ...
        ['-I' inc_dir], fullfile(cpp_dir, f{1}), '-outdir', out_dir);
end

end

function flags = omp_lib()
    [~, cmdout] = system('g++ -liomp5');
    if ~isempty(strfind(cmdout, 'main'))
        flags = '-liomp5';
    else
        flags = '-lgomp';
    end
end

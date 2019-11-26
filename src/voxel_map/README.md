# Voxel Map #

Simple C++ header-only library with Matlab and Python interfaces for dealing with 3-D voxel maps.

## Compile from Matlab ##

Compiling only the mex may be easier than compiling the complete cmake project below.

```
git clone git@bitbucket.org:tpetricek/voxel_map.git
cd voxel_map/matlab
matlab -r "mex_voxel_map();"
```

## Distutils/Setuptools Installation ##

Python distutils/setuptools installation, including virtual environment.

```
python setup.py install
```

## CMake Build ##

```
git clone git@bitbucket.org:tpetricek/voxel_map.git
cd voxel_map
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

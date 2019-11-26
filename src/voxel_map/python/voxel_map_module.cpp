#include <Python.h>

#define NPY_NO_DEPRECATED_API NPY_API_VERSION

#ifdef _WIN32
#define BOOST_PYTHON_STATIC_LIB
#endif

//#include "array.h"
#include <boost/python.hpp>
//#include <boost/python/numeric.hpp>
#include <numpy/arrayobject.h>
//#include <voxel_map/strided.h>
//#include <voxel_map/wrapper.h>
#include <voxel_map/py_voxel_map.h>

//namespace {

//namespace vm = voxel_map;
//namespace bp = boost::python;
//namespace bpn = boost::python::numeric;

//typedef vm::Voxel<int> Voxel;
//typedef float Value;
//typedef vm::VoxelMap<Voxel, Value> VoxelMap;

//typedef std::vector<Voxel> VoxelList;
//typedef boost::unordered_set<Voxel, typename Voxel::Hash> VoxelSet;



namespace {

#if PY_MAJOR_VERSION >= 3
int init_numpy() { import_array(); }
#else
void init_numpy() { import_array(); }
#endif

using voxel_map::PyVoxelMap;

}

BOOST_PYTHON_MODULE(voxel_map)
{
    init_numpy();
    // TODO: Remove boost python dep completely.
    // Using object/handle etc. require compiling boost.python with -fPIC anyway.
    boost::python::class_<PyVoxelMap>("VoxelMap")
            .def(boost::python::init<const double, const double, const double, const double>())
            .def_readwrite("voxel_size", &PyVoxelMap::voxel_size)
            .def_readwrite("free_update", &PyVoxelMap::free_update)
            .def_readwrite("hit_update", &PyVoxelMap::hit_update)
            .def_readwrite("occupied_threshold", &PyVoxelMap::occupied_threshold)
            .def_readwrite("__getstate_manages_dict__", &PyVoxelMap::getstate_manages_dict)
            .def("size", &PyVoxelMap::size)
            .def("clear", &PyVoxelMap::clear)
            .def("get_voxels", static_cast<PyObject *(PyVoxelMap::*)()>(&PyVoxelMap::getVoxels))
            .def("get_voxels", static_cast<PyObject *(PyVoxelMap::*)(PyObject *, PyObject *)>(&PyVoxelMap::getVoxels))
            .def("set_voxels", &PyVoxelMap::setVoxels)
            .def("update_voxels", &PyVoxelMap::updateVoxels)
            .def("update_lines", &PyVoxelMap::updateLines)
            .def("trace_lines", &PyVoxelMap::traceLines)
            .def("trace_rays", &PyVoxelMap::traceRays)
            .def("__getstate__", &PyVoxelMap::getState)
            .def("__setstate__", &PyVoxelMap::setState)
            .enable_pickling()
            ;
}

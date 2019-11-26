#include <Python.h>

#define NPY_NO_DEPRECATED_API NPY_API_VERSION

//#include "array.h"
#include <boost/python.hpp>
#include <numpy/arrayobject.h>
#include <voxel_map/py_array.h>
#include <voxel_map/strided.h>
#include <voxel_map/wrapper.h>

namespace voxel_map {

//namespace vm = voxel_map;
//namespace bp = boost::python;

//typedef voxel_map::Voxel<int> Voxel;
//typedef float Value;
//typedef voxel_map::VoxelMap<Voxel, Value> VoxelMap;

//typedef std::vector<Voxel> VoxelList;
//typedef boost::unordered_set<Voxel, typename Voxel::Hash> VoxelSet;

class PyVoxelMap
{
public:
    typedef voxel_map::Voxel<int> Voxel;
    typedef float Value;
    typedef voxel_map::VoxelMap<Voxel, Value> VoxelMap;

    VoxelMap vox_map;
    double voxel_size;
    Value free_update;
    Value hit_update;
    Value occupied_threshold;
    bool getstate_manages_dict;

    PyVoxelMap():
        vox_map(),
        voxel_size(1.0),
        free_update(-1.0),
        hit_update(1.0),
        occupied_threshold(0.0),
        getstate_manages_dict(true)
    {}

    PyVoxelMap(const double voxel_size,
               const double free_update,
               const double hit_update,
               const double occupied_threshold):
        voxel_size(voxel_size),
        free_update(free_update),
        hit_update(hit_update),
        occupied_threshold(occupied_threshold),
        getstate_manages_dict(true)
    {}

    PyObject * size()
    {
        return PyLong_FromSize_t(vox_map.size());
    }

    void clear()
    {
        vox_map.clear();
    }

    PyObject * getVoxels()
    {
        Array<double> x(3, vox_map.size());
        Array<double> v(vox_map.size());
        Array<double> l(vox_map.size());

        voxel_map::getVoxels(vox_map, x.begin(1), x.begin(1, 1*x.stride(0)), x.begin(1, 2*x.stride(0)),
                             l.begin(), v.begin());

        toVoxelCenters(x.begin(), x.end(), x.begin());

        PyObject *res = PyTuple_New(3);
        PyTuple_SetItem(res, 0, reinterpret_cast<PyObject *>(x.array(true)));
        PyTuple_SetItem(res, 1, reinterpret_cast<PyObject *>(l.array(true)));
        PyTuple_SetItem(res, 2, reinterpret_cast<PyObject *>(v.array(true)));
        return res;
    }
    
    PyObject * getVoxels(PyObject *x_obj, PyObject *l_obj)
    {
        Array<double> x(x_obj);
        Array<double> l(l_obj);
        if (x.size(0) != 3)
            throw std::runtime_error("Argument x must be 2-D array of size 3-by-N.");
        if (l.numel() != x.size(1))
            throw std::runtime_error("Argument level must be vector with N elements.");

        toVoxelIndices(x.begin(), x.end(), x.begin());
        Array<double> v(l.numel());

        voxel_map::getVoxels(vox_map,
                             x.begin(1), x.end(1), x.begin(1, 1*x.stride(0)), x.begin(1, 2*x.stride(0)),
                             l.begin(), v.begin());

        return reinterpret_cast<PyObject *>(v.array(true));
    }

    void setVoxels(PyObject *x_obj, PyObject *l_obj, PyObject *v_obj)
    {
        Array<double> x(x_obj);
        Array<double> l(l_obj);
        Array<double> v(v_obj);
        if (x.size(0) != 3)
            throw std::runtime_error("Argument x must be 2-D array of size 3-by-N.");
        if (l.numel() != x.size(1))
            throw std::runtime_error("Argument level must be vector with N elements.");
        if (v.numel() != x.size(1))
            throw std::runtime_error("Argument value must be vector with N elements.");

        toVoxelIndices(x.begin(), x.end(), x.begin());

        voxel_map::setVoxels(vox_map,
                             x.begin(1), x.end(1), x.begin(1, 1*x.stride(0)), x.begin(1, 2*x.stride(0)),
                             l.begin(), v.begin());
    }

    PyObject * updateVoxels(PyObject *x_obj, PyObject *l_obj, PyObject *v_obj)
    {
        Array<double> x(x_obj);
        Array<double> l(l_obj);
        Array<double> v(v_obj);
        Array<double> v1(v.numel());
        if (x.size(0) != 3)
            throw std::runtime_error("Argument x must be 2-D array of size 3-by-N.");
        if (l.numel() != x.size(1))
            throw std::runtime_error("Argument level must be vector with N elements.");
        if (v.numel() != x.size(1))
            throw std::runtime_error("Argument value must be vector with N elements.");

        toVoxelIndices(x.begin(), x.end(), x.begin());

        voxel_map::updateVoxels(vox_map,
                         x.begin(1), x.end(1), x.begin(1, 1*x.stride(0)), x.begin(1, 2*x.stride(0)),
                         l.begin(), v.begin(), v1.begin());
        return reinterpret_cast<PyObject *>(v1.array(true));
    }

    void updateLines(PyObject *x_obj, PyObject *y_obj)
    {
        Array<double> x(x_obj);
        Array<double> y(y_obj);
        if (x.size(0) != 3)
            throw std::runtime_error("Argument x must be 2-D array of size 3-by-N.");
        if (y.size(0) != 3)
            throw std::runtime_error("Argument y must be 2-D array of size 3-by-N.");
        if (x.size(1) != y.size(1))
            throw std::runtime_error("Arguments x and y must have same size.");

        toVoxelIndices(x.begin(), x.end(), x.begin());
        toVoxelIndices(y.begin(), y.end(), y.begin());

        voxel_map::updateLines(vox_map,
                               x.begin(1), x.end(1), x.begin(1, 1*x.stride(0)), x.begin(1, 2*x.stride(0)),
                               y.begin(1), y.end(1), y.begin(1, 1*y.stride(0)), y.begin(1, 2*y.stride(0)),
                               free_update, hit_update);
    }

    PyObject * traceLines(PyObject *x_obj, PyObject *y_obj,
                          PyObject *min_val_obj, PyObject *max_val_obj,
                          PyObject *check_unknown_obj)
    {
        Array<double> x(x_obj);
        Array<double> y(y_obj);
        if (x.size(0) != 3)
            throw std::runtime_error("Argument x must be 2-D array of size 3-by-N.");
        if (y.size(0) != 3)
            throw std::runtime_error("Argument y must be 2-D array of size 3-by-N.");
        if (x.size(1) != y.size(1))
            throw std::runtime_error("Arguments x and y must have same size.");
        double min_val = PyFloat_AsDouble(min_val_obj);
        double max_val = PyFloat_AsDouble(max_val_obj);
        bool check_unknown = PyFloat_AsDouble(check_unknown_obj);

        toVoxelIndices(x.begin(), x.end(), x.begin());
        toVoxelIndices(y.begin(), y.end(), y.begin());
        Voxel::Filter filter;
        if (std::isnan(min_val) || std::isnan(max_val))
            filter = &voxel_map::alwaysTrue<Voxel>;
        else
            filter = boost::bind(voxel_map::voxelInRange<Voxel, Value>,
                                 boost::cref(vox_map), _1, min_val, max_val, check_unknown);
        Array<double> h(x.size(0), x.size(1));
        Array<double> v(x.size(1));

        voxel_map::traceLines(vox_map,
                       x.begin(1), x.end(1), x.begin(1, 1*x.stride(0)), x.begin(1, 2*x.stride(0)),
                       y.begin(1), y.end(1), y.begin(1, 1*y.stride(0)), y.begin(1, 2*y.stride(0)),
                       filter,
                       h.begin(1), h.begin(1, 1*h.stride(0)), h.begin(1, 2*h.stride(0)),
                       v.begin());

        toVoxelCenters(h.begin(), h.end(), h.begin());

        PyObject *res = PyTuple_New(2);
        PyTuple_SetItem(res, 0, reinterpret_cast<PyObject *>(h.array(true)));
        PyTuple_SetItem(res, 1, reinterpret_cast<PyObject *>(v.array(true)));
        return res;
    }

    PyObject * traceRays(PyObject *x_obj, PyObject *y_obj,
                         PyObject *max_range_obj, PyObject *min_val_obj, PyObject *max_val_obj,
                         PyObject *check_unknown_obj)
    {
        Array<double> x(x_obj);
        Array<double> y(y_obj);
        if (x.size(0) != 3)
            throw std::runtime_error("Argument x must be 2-D array of size 3-by-N.");
        if (y.size(0) != 3)
            throw std::runtime_error("Argument y must be 2-D array of size 3-by-N.");
        if (x.size(1) != y.size(1))
            throw std::runtime_error("Arguments x and y must have same size.");
        double max_range = PyFloat_AsDouble(max_range_obj) / voxel_size;
        double min_val = PyFloat_AsDouble(min_val_obj);
        double max_val = PyFloat_AsDouble(max_val_obj);
        bool check_unknown = PyFloat_AsDouble(check_unknown_obj);

        toVoxelIndices(x.begin(), x.end(), x.begin());
        Voxel::Filter filter;
        if (std::isnan(min_val) || std::isnan(max_val))
            filter = &voxel_map::alwaysTrue<Voxel>;
        else
            filter = boost::bind(voxel_map::voxelInRange<Voxel, Value>,
                                 boost::cref(vox_map), _1, min_val, max_val, check_unknown);
        Array<double> h(x.size(0), x.size(1));
        Array<double> v(x.size(1));

        voxel_map::traceRays(vox_map,
                             x.begin(1), x.end(1), x.begin(1, 1*x.stride(0)), x.begin(1, 2*x.stride(0)),
                             y.begin(1), y.end(1), y.begin(1, 1*y.stride(0)), y.begin(1, 2*y.stride(0)),
                             filter, max_range,
                             h.begin(1), h.begin(1, 1*h.stride(0)), h.begin(1, 2*h.stride(0)),
                             v.begin());

        toVoxelCenters(h.begin(), h.end(), h.begin());

        PyObject *res = PyTuple_New(2);
        PyTuple_SetItem(res, 0, reinterpret_cast<PyObject *>(h.array(true)));
        PyTuple_SetItem(res, 1, reinterpret_cast<PyObject *>(v.array(true)));
        return res;
    }
    
    // Pickle handlers
    // TODO: Get/set member vars too (use dict for state).
    // TODO: Use smaller data types to save space.
    PyObject * getState()
    {
        // return getVoxels();
        PyObject *res = PyDict_New();
        PyDict_SetItemString(res, "voxel_size", PyFloat_FromDouble(voxel_size));
        PyDict_SetItemString(res, "empty_update", PyFloat_FromDouble(free_update));
        PyDict_SetItemString(res, "occupied_update", PyFloat_FromDouble(hit_update));
        PyDict_SetItemString(res, "occupied_threshold", PyFloat_FromDouble(occupied_threshold));
        PyDict_SetItemString(res, "voxels", getVoxels());
        return res;
    }
    void setState(PyObject *state)
    {
        // setVoxels(PyTuple_GetItem(state, 0), PyTuple_GetItem(state, 1), PyTuple_GetItem(state, 2));
        voxel_size = PyFloat_AsDouble(PyDict_GetItemString(state, "voxel_size"));
        free_update = PyFloat_AsDouble(PyDict_GetItemString(state, "empty_update"));
        hit_update = PyFloat_AsDouble(PyDict_GetItemString(state, "occupied_update"));
        occupied_threshold = PyFloat_AsDouble(PyDict_GetItemString(state, "occupied_threshold"));
        setVoxels(PyTuple_GetItem(PyDict_GetItemString(state, "voxels"), 0),
                  PyTuple_GetItem(PyDict_GetItemString(state, "voxels"), 1),
                  PyTuple_GetItem(PyDict_GetItemString(state, "voxels"), 2));
    }

    // TODO: Template?
    Voxel::ElemType voxelIndex(const double x)
    {
        return static_cast<Voxel::ElemType>(floor(x / voxel_size));
    }
    double voxelCenter(const Voxel::ElemType i)
    {
        return voxel_size * (i + 0.5);
    }
    template<typename I, typename O>
    void toVoxelIndices(I from, I to, O dst)
    {
        for (; from != to; ++from, ++dst)
            *dst = voxelIndex(*from);
    }
    template<typename I, typename O>
    void toVoxelCenters(I from, I to, O dst)
    {
        for (; from != to; ++from, ++dst)
            *dst = voxelCenter(*from);
    }
};

}

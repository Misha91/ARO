
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/unordered_set.hpp>
#include <matrix.h>
#include <mex.h>
#include "array.h"
//#include "array2.h"
#include <voxel_map/strided.h>
#include <voxel_map/voxel_map.h>
#include <voxel_map/wrapper.h>

namespace {

typedef int I;
typedef voxel_map::Voxel<I> V;
typedef float T;
typedef voxel_map::VoxelMap<V, T> VoxelMap;
typedef std::vector<V> VoxelList;
typedef boost::unordered_set<V, typename V::Hash> VoxelSet;

boost::unordered_map<size_t, boost::shared_ptr<VoxelMap> > voxel_maps;
// Next voxel map ID to be used.
size_t next_id = 0;
// Lock for modifying the voxel map container.
// NB: Concurent access to the same map should be avoided by the user.
boost::mutex map_container_lock;

std::string getOp(int nrhs, const mxArray* prhs[])
{
    if (nrhs < 1 || !mxIsChar(prhs[0]))
        mexErrMsgTxt("Operation invalid or not provided.");
    return mxArrayToString(prhs[0]);
}

size_t getMapId(int nrhs, const mxArray* prhs[])
{
    if (nrhs < 2)
        mexErrMsgTxt("Map ID not provided.");
    const double map_id = ConstArray<double>(prhs[1]);
    return map_id;
}

VoxelMap & getMap(int nrhs, const mxArray* prhs[])
{
    size_t map_id = getMapId(nrhs, prhs);
    boost::lock_guard<boost::mutex> guard(map_container_lock);
    if (voxel_maps.find(map_id) == voxel_maps.end() || voxel_maps[map_id].get() == NULL)
        mexErrMsgTxt("Map ID invalid.");
    return *voxel_maps[map_id];
}

void createMap(int nlhs, mxArray* plhs[])
{
    boost::lock_guard<boost::mutex> guard(map_container_lock);
    if (nlhs < 1)
        mexErrMsgTxt("Not enough outputs.");
    size_t map_id = next_id++;
    voxel_maps[map_id] = boost::make_shared<VoxelMap>();
    plhs[0] = mxCreateDoubleScalar(map_id);
}

void deleteMap(int nrhs, const mxArray* prhs[])
{
    boost::lock_guard<boost::mutex> guard(map_container_lock);
    voxel_maps.erase(getMapId(nrhs, prhs));
}

void size(int nlhs, mxArray* plhs[], const VoxelMap &map)
{
    if (nlhs < 1)
        mexErrMsgTxt("Not enough outputs.");
    plhs[0] = mxCreateDoubleScalar(map.size());
}

void clear(VoxelMap &map)
{
    map.clear();
}

void traceLines(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[], const VoxelMap &map)
{
    const ConstArray<double> from(prhs[2]);
    const ConstArray<double> to(prhs[3]);
    const double min_val = ConstArray<double>(prhs[4]);
    const double max_val = ConstArray<double>(prhs[5]);
    mexPrintf("min_val: %f, max_val: %f\n", min_val, max_val);
    if (from.size(0) != 3) mexErrMsgTxt("Argument 'from' must have 3 rows.");
    if (to.size(0) != 3) mexErrMsgTxt("Argument 'to' must have 3 rows.");
    if (from.size(1) != to.size(1)) mexErrMsgTxt("Arguments 'from' and 'to' must be same size.");
    size_t n = std::max(from.size(1), to.size(1));
    V::Filter filter;
    if (std::isnan(min_val) || std::isnan(max_val))
        filter = &voxel_map::alwaysTrue<V>;
    else
        filter = boost::bind(voxel_map::voxelInRange<V, T>, boost::cref(map), _1, T(min_val), T(max_val), true);
//        filter = boost::bind(voxel_map::ancestorInRange<V, T>, boost::cref(map), _1, T(min_val), T(max_val));
    Array<double> hit(3, n);
    plhs[0] = hit;
    Array<double> val(1, n);
    plhs[1] = val;
    voxel_map::traceLines(map,
                          from.begin(1), from.end(1), from.begin(1, 1), from.begin(1, 2),
                          to.begin(1), to.end(1), to.begin(1, 1), to.begin(1, 2),
                          filter,
                          hit.begin(1), hit.begin(1, 1), hit.begin(1, 2), val.begin());
}

void traceRays(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[], const VoxelMap &map)
{
    const ConstArray<double> from(prhs[2]);
    const ConstArray<double> dir(prhs[3]);
    const double min_val = ConstArray<double>(prhs[4]);
    const double max_val = ConstArray<double>(prhs[5]);
    const double max_range = ConstArray<double>(prhs[6]);
    assert(from.size(0) == 3);
    assert(dir.size(0) == 3);
    size_t n = std::max(from.size(1), dir.size(1));
    V::Filter filter;
    if (std::isnan(min_val) || std::isnan(max_val))
        filter = &voxel_map::alwaysTrue<V>;
    else
        filter = boost::bind(voxel_map::voxelInRange<V, T>, boost::cref(map), _1, T(min_val), T(max_val), true);
//        filter = boost::bind(voxel_map::ancestorInRange<V, T>, boost::cref(map), _1, T(min_val), T(max_val));
    Array<double> hit(3, n);
    plhs[0] = hit;
    Array<double> val(1, n);
    plhs[1] = val;
    const bool return_rays = (nlhs == 4);
    std::vector<VoxelList> rays;
    voxel_map::traceRays(map,
                         from.begin(1), from.end(1), from.begin(1, 1), from.begin(1, 2),
                         dir.begin(1), dir.end(1), dir.begin(1, 1), dir.begin(1, 2),
                         filter, max_range,
                         hit.begin(1), hit.begin(1, 1), hit.begin(1, 2), val.begin(),
                         return_rays ? &rays : NULL);
    if (return_rays)
    {
        plhs[2] = mxCreateCellMatrix(1, n);
        plhs[3] = mxCreateCellMatrix(1, n);
        for (size_t i = 0; i < n; ++i)
        {
            Array<double> arr_rays(3, rays[i].size());
            mxSetCell(plhs[2], i, arr_rays);
            Array<double> arr_vals(1, rays[i].size());
            mxSetCell(plhs[3], i, arr_vals);
            Array<double>::Iter x = arr_rays.begin(1), y = arr_rays.begin(1, 1), z = arr_rays.begin(1, 2);
            double *v = arr_vals.begin();
            for (VoxelList::const_iterator vox_it = rays[i].begin(); vox_it != rays[i].end(); ++vox_it)
            {
                *x++ = (*vox_it)[0];
                *y++ = (*vox_it)[1];
                *z++ = (*vox_it)[2];
                *v++ = map.voxelKnown(*vox_it) ? map.value(*vox_it) : std::numeric_limits<T>::quiet_NaN();
            }
        }
    }
}

void updateLines(int nrhs, const mxArray* prhs[], VoxelMap &map)
{
    const ConstArray<double> from(prhs[2]);
    const ConstArray<double> to(prhs[3]);
    const double free_update = ConstArray<double>(prhs[4]);
    const double hit_update = ConstArray<double>(prhs[5]);
    voxel_map::updateLines(map,
                           from.begin(1), from.end(1), from.begin(1, 1), from.begin(1, 2),
                           to.begin(1), to.end(1), to.begin(1, 1), to.begin(1, 2),
                           T(free_update), T(hit_update));
}

void getVoxels(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[], const VoxelMap &map)
{
    if (nrhs == 3)  // Query specific voxels?
    {
        const ConstArray<double> query(prhs[2]);
        Array<double> vox(3, query.size(1));
        Array<double> val(1, query.size(1));
        plhs[0] = vox;
        plhs[1] = val;
        // Copy input to output directly.
        std::copy(query.begin(), query.end(), vox.begin());
        voxel_map::getVoxels(map, query.begin(1), query.end(1),
                             query.begin(1, 1), query.begin(1, 2),
                             voxel_map::FixedValueIterator<I>(0), val.begin());
    }
    else if (nrhs == 2)
    {
        size_t num_out = map.size();
        Array<double> vox(3, num_out);
        Array<double> val(1, num_out);
        plhs[0] = vox;
        plhs[1] = val;
        // TODO: Return level too.
        getVoxels(map,
                  vox.begin(1), vox.begin(1, 1), vox.begin(1, 2),
                  voxel_map::Void(), val.begin());
    }
}

void setVoxels(int nrhs, const mxArray* prhs[], VoxelMap &map)
{
    const ConstArray<double> vox(prhs[2]);
    const ConstArray<double> val(prhs[3]);
    if (vox.size(0) != 3) mexErrMsgTxt("Invalid size of voxel array.");
    if (val.size(0) != 1) mexErrMsgTxt("Invalid size of value array.");
    voxel_map::setVoxels(map,
                         vox.begin(1), vox.end(1), vox.begin(1, 1), vox.begin(1, 2),
                         voxel_map::FixedValueIterator<I>(0), val.begin());
}

void updateVoxels(int nrhs, const mxArray* prhs[], VoxelMap &map)
{
    const ConstArray<double> vox(prhs[2]);
    const ConstArray<double> val(prhs[3]);
    if (vox.size(0) != 3) mexErrMsgTxt("Invalid size of voxel array.");
    if (val.size(0) != 1) mexErrMsgTxt("Invalid size of value array.");
    if (vox.size(1) != val.size(1)) mexErrMsgTxt("Voxel and value arrays must have same nuber of columns.");
    voxel_map::updateVoxels(map,
                            vox.begin(1), vox.end(1), vox.begin(1, 1), vox.begin(1, 2),
                            voxel_map::FixedValueIterator<I>(0), val.begin());
}

void collapse(int nrhs, const mxArray* prhs[], VoxelMap &map)
{
    const double min_val = ConstArray<double>(prhs[2]);
    const double max_val = ConstArray<double>(prhs[3]);
    V::Filter filter = boost::bind(voxel_map::voxelInRange<V, T>,
                                   boost::cref(map), _1, min_val, max_val, true);
    VoxelMap::Reduce reduce = std::plus<T>();
    T initial = 0;
    voxel_map::collapseMap(map, filter, reduce, initial);
}

void expand(int nrhs, const mxArray* prhs[], VoxelMap &map)
{
    const double min_val = ConstArray<double>(prhs[2]);
    const double max_val = ConstArray<double>(prhs[3]);
    V::Filter filter = boost::bind(voxel_map::voxelInRange<V, T>,
                                   boost::cref(map), _1, min_val, max_val, true);
    VoxelMap::Reduce reduce = std::plus<T>();
    T initial = 0;
    voxel_map::expandMap(map, filter, reduce, initial);
}

void merge(VoxelMap &map)
{
    VoxelMap::Reduce reduce = std::plus<T>();
    voxel_map::mergeOnce(map, reduce);
}

} // namespace

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    const std::string op = getOp(nrhs, prhs);
//    mexPrintf("op: %s, nlhs: %i, nrhs: %i\n", op.c_str(), nlhs, nrhs);
    if (op == "create_map")
    {
        createMap(nlhs, plhs);
        return;
    }

    if (op == "delete_map")
    {
        deleteMap(nrhs, prhs);
        return;
    }

    VoxelMap &map = getMap(nrhs, prhs);
    if (op == "size")
        size(nlhs, plhs, map);
    else if (op == "clear")
        clear(map);
    else if (op == "trace_rays")
        traceRays(nlhs, plhs, nrhs, prhs, map);
    else if (op == "trace_lines")
        traceLines(nlhs, plhs, nrhs, prhs, map);
    else if (op == "get_voxels")
        getVoxels(nlhs, plhs, nrhs, prhs, map);
    else if (op == "update_lines")
        updateLines(nrhs, prhs, map);
    else if (op == "update_voxels")
        updateVoxels(nrhs, prhs, map);
    else if (op == "set_voxels")
        setVoxels(nrhs, prhs, map);
    else if (op == "collapse")
        collapse(nrhs, prhs, map);
    else if (op == "expand")
        expand(nrhs, prhs, map);
    else if (op == "merge")
        merge(map);
    else
        mexErrMsgTxt("Invalid operation.");
}

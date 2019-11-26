
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/range/adaptor/strided.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/unordered_set.hpp>
#include <matrix.h>
#include <mex.h>
#include <voxel_map/voxel_map.h>

namespace {

template<typename C>
class ContainerHash {
public:
    size_t operator()(const C &c) const
    {
        size_t h = 0;
        for (typename C::const_iterator it = c.begin(); it != c.end(); ++it)
            boost::hash_combine(h, *it);
        return h;
    }
};

template<typename C, typename V>
class ContainerMap {
public:
    typedef boost::unordered_map<C, V, ContainerHash<C> > Type;
};

//typedef ContainerMap<std::vector<double>, double>::Type VectorDDoubleMap;
typedef ContainerMap<std::vector<double>, std::vector<double> >::Type VectorDVectorDMap;
//typedef ContainerMap<voxel_map::Voxel<double>, double>::Type VoxelDDoubleMap;
typedef VectorDVectorDMap Map;

std::vector<boost::shared_ptr<Map> > maps;


} // namespace

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    assert(nrhs >= 1);
    std::string op(mxArrayToString(prhs[0]));
    if (op == "create")
    {
        maps.push_back(boost::make_shared<Map>());
        plhs[0] = mxCreateDoubleScalar(double(maps.size() - 1));
        return;
    }

    assert(nrhs >= 2);
    size_t map_id = size_t(mxGetScalar(prhs[1]));
    if (op == "delete")
    {
        maps.at(size_t(map_id)).reset();
        return;
    }

    Map &map = *maps[map_id];
    if (op == "clear")
    {
        map.clear();
        return;
    }

    if (op == "is")
    {
        size_t m = mxGetM(prhs[2]);
        size_t n = mxGetN(prhs[2]);
        const double *key_ptr = mxGetPr(prhs[2]);
        plhs[0] = mxCreateLogicalMatrix(m, 1);
        bool *value_ptr = static_cast<bool *>(mxGetData(plhs[0]));
        for (size_t i = 0; i < m; ++i)
        {
            std::vector<double> key(n);
            for (size_t k = 0; k < n; ++k)
                key[k] = key_ptr[k*m + i];
            value_ptr[i] = map.find(key) != map.end();
        }
        return;
    }

    if (op == "get")
    {
        if (nlhs == 2 && nrhs == 2)
        {
            if (map.empty())
            {
                plhs[0] = mxCreateDoubleMatrix(0, 0, mxREAL);
                plhs[1] = mxCreateDoubleMatrix(0, 0, mxREAL);
                return;
            }
            size_t m = map.size();
            size_t n = map.begin()->first.size();
            size_t nv = map.begin()->second.size();
            plhs[0] = mxCreateDoubleMatrix(m, n, mxREAL);
            plhs[1] = mxCreateDoubleMatrix(m, nv, mxREAL);

            double *key_ptr = mxGetPr(plhs[0]);
            double *val_ptr = mxGetPr(plhs[1]);
            size_t i = 0;
            for (Map::const_iterator it = map.begin(); it != map.end(); ++it, ++i)
            {
                for (size_t k = 0; k < n; ++k)
                    key_ptr[k*m + i] = it->first[k];
                for (size_t k = 0; k < nv; ++k)
                    val_ptr[k*m + i] = it->second[k];
            }
            assert(i == m);
            return;
        }

        assert(nrhs >= 3);
        if (mxIsEmpty(prhs[2]))
        {
            plhs[0] = mxCreateDoubleMatrix(0, 0, mxREAL);
            return;
        }
//        assert(!map.empty);
        size_t m = mxGetM(prhs[2]);
        size_t n = mxGetN(prhs[2]);
        size_t nv = map.begin()->second.size();
        const double *key_ptr = mxGetPr(prhs[2]);
        plhs[0] = mxCreateDoubleMatrix(m, nv, mxREAL);
        double *val_ptr = mxGetPr(plhs[0]);
#pragma omp parallel for schedule(static)
        for (size_t i = 0; i < m; ++i)
        {
            std::vector<double> key(n);
            for (size_t k = 0; k < n; ++k)
                key[k] = key_ptr[k*m + i];
            for (size_t k = 0; k < nv; ++k)
                val_ptr[k*m + i] = map.at(key)[k];
        }
        return;
    }

    if (op == "set")
    {
        assert(nrhs >= 4);
        size_t m = mxGetM(prhs[2]);
        size_t n = mxGetN(prhs[2]);
        size_t nv = mxGetN(prhs[3]);
//        if (!map.empty())
//            assert(map.begin()->size() == nv);
        const double *key_ptr = mxGetPr(prhs[2]);
        const double *val_ptr = mxGetPr(prhs[3]);
        for (size_t i = 0; i < m; ++i)
        {
            std::vector<double> key(n);
            for (size_t k = 0; k < n; ++k)
                key[k] = key_ptr[k*m + i];
            std::vector<double> val(nv);
            for (size_t k = 0; k < nv; ++k)
                val[k] = val_ptr[k*m + i];
            map[key] = val;
        }
        return;
    }
}

// DEPRECATED: Converted to header only lib.

#include <cmath>
#include <voxel_map/voxel_map.h>

namespace voxel_map
{

template<typename V>
V line(const V &v1, const V &v2, const typename V::Filter &filter, std::vector<V> *voxels)
{
    assert(v1.level == v2.level);
    assert(v1.level == 0);

    typename V::ElemType i, dx, dy, dz, l, m, n, x_inc, y_inc, z_inc, err_1, err_2, dx2, dy2, dz2;
    V pt = v1;

    dx = v2.x - v1.x;
    dy = v2.y - v1.y;
    dz = v2.z - v1.z;
    x_inc = (dx < 0) ? -1 : 1;
    l = abs(dx);
    y_inc = (dy < 0) ? -1 : 1;
    m = abs(dy);
    z_inc = (dz < 0) ? -1 : 1;
    n = abs(dz);
    dx2 = l << 1;
    dy2 = m << 1;
    dz2 = n << 1;
    
    if ((l >= m) && (l >= n))
    {
        err_1 = dy2 - l;
        err_2 = dz2 - l;
        for (i = 0; i < l; i++)
        {
            if (!filter(pt))
                break;
            if (voxels)
                voxels->push_back(pt);
            if (err_1 > 0)
            {
                pt[1] += y_inc;
                err_1 -= dx2;
            }
            if (err_2 > 0)
            {
                pt[2] += z_inc;
                err_2 -= dx2;
            }
            err_1 += dy2;
            err_2 += dz2;
            pt[0] += x_inc;
        }
    }
    else if ((m >= l) && (m >= n))
    {
        err_1 = dx2 - m;
        err_2 = dz2 - m;
        for (i = 0; i < m; i++)
        {
            if (!filter(pt))
                break;
            if (voxels)
                voxels->push_back(pt);
            if (err_1 > 0)
            {
                pt[0] += x_inc;
                err_1 -= dy2;
            }
            if (err_2 > 0)
            {
                pt[2] += z_inc;
                err_2 -= dy2;
            }
            err_1 += dx2;
            err_2 += dz2;
            pt[1] += y_inc;
        }
    }
    else
    {
        err_1 = dy2 - n;
        err_2 = dx2 - n;
        for (i = 0; i < n; i++)
        {
            if (!filter(pt))
                break;
            if (voxels)
                voxels->push_back(pt);
            if (err_1 > 0)
            {
                pt[1] += y_inc;
                err_1 -= dz2;
            }
            if (err_2 > 0)
            {
                pt[0] += x_inc;
                err_2 -= dz2;
            }
            err_1 += dy2;
            err_2 += dx2;
            pt[2] += z_inc;
        }
    }
    if (filter(pt) && voxels)
        voxels->push_back(pt);
    return pt;
}

template<typename V>
V ray(const V &v1, double dx, double dy, double dz, const typename V::Filter &filter, std::vector<V> *voxels)
{
    long double lo = std::numeric_limits<typename V::ElemType>::min();
    long double hi = std::numeric_limits<typename V::ElemType>::max();

    long double p[6][4] = {{1, 0, 0, lo}, {1, 0, 0, hi},
                           {0, 1, 0, lo}, {0, 1, 0, hi},
                           {0, 0, 1, lo}, {0, 0, 1, hi}};
    double d[3] = {dx, dy, dz};

    for (size_t i = 0; i < 6; ++i)
    {
        long double nx0 = 0.0;
        long double nd = 0.0;
        for (size_t j = 0; j < 3; ++j)
        {
            nx0 += p[i][j] * v1[j];
            nd += p[i][j] * d[j];
        }
        long double t = (p[i][3] - nx0) / nd;
        if (!(t >= 0))
            continue;
        long double x = v1[0] + t * dx;
        long double y = v1[1] + t * dy;
        long double z = v1[2] + t * dz;
        if (x >= lo && x <= hi && y >= lo && y <= hi && z >= lo && z <= hi)
        {
            const V v2(x + 0.5, y + 0.5, z + 0.5);
            return line(v1, v2, filter, voxels);
        }
    }
}

//template<typename V>
//V ray(const V &v1, double dx, double dy, double dz, const typename V::Filter &filter, std::vector<V> *voxels)
//{
//    long l = std::numeric_limits<typename V::ElemType>::min();
//    long h = std::numeric_limits<typename V::ElemType>::max();
//    double tx = ((dx >= 0 ? h : l) - v1.x) / dx;
//    double ty = ((dy >= 0 ? h : l) - v1.y) / dy;
//    double tz = ((dz >= 0 ? h : l) - v1.z) / dz;
//    double t_min = std::min(tx, std::min(ty, tz));
//    V v2(round(v1.x + t_min * dx), round(v1.y + t_min * dy), round(v1.z + t_min * dz));
//    return line(v1, v2, filter, voxels);
//}


template class Voxel<char>;
template class Voxel<short>;
template class Voxel<int>;
template class Voxel<long>;

template class VoxelMap<Voxel<char>, char>;
template class VoxelMap<Voxel<int>, int>;
template class VoxelMap<Voxel<long>, long>;
template class VoxelMap<Voxel<int>, float>;
template class VoxelMap<Voxel<long>, double>;

template Voxel<short> line<Voxel<short> >(const Voxel<short> &v1, const Voxel<short> &v2,
                                          const Voxel<short>::Filter &filter,
                                          std::vector<Voxel<short> > *voxels);
template Voxel<int> line<Voxel<int> >(const Voxel<int> &v1, const Voxel<int> &v2,
                                      const Voxel<int>::Filter &filter,
                                      std::vector<Voxel<int> > *voxels);
template Voxel<long> line<Voxel<long> >(const Voxel<long> &v1, const Voxel<long> &v2,
                                        const Voxel<long>::Filter &filter,
                                        std::vector<Voxel<long> > *voxels);

template Voxel<short> ray<Voxel<short> >(const Voxel<short> &v1, double dx, double dy, double dz,
                                         const Voxel<short>::Filter &filter,
                                         std::vector<Voxel<short> > *voxels);
template Voxel<int> ray<Voxel<int> >(const Voxel<int> &v1, double dx, double dy, double dz,
                                     const Voxel<int>::Filter &filter,
                                     std::vector<Voxel<int> > *voxels);
template Voxel<long> ray<Voxel<long> >(const Voxel<long> &v1, double dx, double dy, double dz,
                                       const Voxel<long>::Filter &filter,
                                       std::vector<Voxel<long> > *voxels);

//template
//bool voxelInRange<Voxel<int>, double>(const VoxelMap<Voxel<int>, double> &map, const Voxel<int> &v,
//                                      const double &min, const double &max);

//template
//bool parentInRange<Voxel<int>, double>(const VoxelMap<Voxel<int>, double> &map, const Voxel<int> &v,
//                                       const double &min, const double &max);

} // namespace

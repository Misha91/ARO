#include <boost/bind.hpp>
#include <boost/chrono.hpp>
#include <cmath>
#include <gtest/gtest.h>
#include <typeinfo>
#include <voxel_map/strided.h>
#include <voxel_map/voxel_map.h>
#include <voxel_map/wrapper.h>

namespace
{

using namespace voxel_map;

class Timer
{
private:
    typedef boost::chrono::high_resolution_clock Clock;
    typedef Clock::time_point Time;
    typedef boost::chrono::duration<double> Duration;
    Time start;
public:
    Timer(): start(Clock::now()) {}
    Timer(const Time &s): start(s) {}
    Timer(const Timer &s): start(s.start) {}
    void reset() { start = Clock::now(); }
    double secondsElapsed() const
    {
        return boost::chrono::duration_cast<Duration>(Clock::now() - start).count();
    }
};

double randu() { return double(rand()) / RAND_MAX; }
int randi(int n) { return std::min(n - 1, int(std::floor(n * randu()))); }

template<typename V, typename T>
void fillBox(const V &v0, const V &v1, const T &val, VoxelMap<V, T> &map)
{
    ASSERT_LE(v0.x, v1.x);
    ASSERT_LE(v0.y, v1.y);
    ASSERT_LE(v0.z, v1.z);
    ASSERT_EQ(v0.level, v1.level);
    for (typename V::ElemType x = v0.x; x < v1.x; ++x)
    {
        for (typename V::ElemType y = v0.y; y < v1.y; ++y)
        {
            for (typename V::ElemType z = v0.z; z < v1.z; ++z)
            {
                map[V(x, y, z, v0.level)] = val;
            }
        }
    }
}

template<typename V, typename T>
void listVoxels(const VoxelMap<V, T> &map)
{
    for (typename VoxelMap<V, T>::ConstIter it = map.begin(); it != map.end(); ++it)
    {
        std::cout << it->first << ", value: " << it->second << ", ancestor: "
                  << (map.ancestorKnown(it->first.parent()) ? map.ancestor(it->first.parent()).toString() : "<none>")
                  << std::endl;
    }
}

TEST(RayTracing, Line)
{
    typedef Voxel<int> V;
    V v0(0, 0, 0);
    V v1(10, 0, 0);
    std::vector<V> voxels;
    V hit = voxel_map::line(v0, v1, &voxel_map::alwaysTrue<V>, &voxels);
    EXPECT_EQ((v1.x - v0.x + 1), voxels.size());
    EXPECT_EQ(v0, voxels.front());
    EXPECT_EQ(v1, voxels.back());
    EXPECT_EQ(v1, hit);
}

TEST(RayTracing, EndPointDirection)
{
    const double d[3] = {3, 7, 19};
    // char
    {
        typedef Voxel<char> V;
        V v1 = endPoint(V(0, 0, 0, 0), d);
        EXPECT_NEAR(double(v1[0]) / v1[1], d[0] / d[1], 1e-1);
        EXPECT_NEAR(double(v1[1]) / v1[2], d[1] / d[2], 1e-1);
        EXPECT_NEAR(double(v1[2]) / v1[0], d[2] / d[0], 1e-1);
    }
    // short
    {
        typedef Voxel<short> V;
        V v1 = endPoint(V(0, 0, 0, 0), d);
        EXPECT_NEAR(double(v1[0]) / v1[1], d[0] / d[1], 1e-3);
        EXPECT_NEAR(double(v1[1]) / v1[2], d[1] / d[2], 1e-3);
        EXPECT_NEAR(double(v1[2]) / v1[0], d[2] / d[0], 1e-3);
    }
    // int
    {
        typedef Voxel<int> V;
        V v1 = endPoint(V(0, 0, 0, 0), d);
        EXPECT_NEAR(double(v1[0]) / v1[1], d[0] / d[1], 1e-3);
        EXPECT_NEAR(double(v1[1]) / v1[2], d[1] / d[2], 1e-3);
        EXPECT_NEAR(double(v1[2]) / v1[0], d[2] / d[0], 1e-3);
    }
    // long
    {
        typedef Voxel<long> V;
        V v1 = endPoint(V(0, 0, 0, 0), d);
        EXPECT_NEAR(double(v1[0]) / v1[1], d[0] / d[1], 1e-3);
        EXPECT_NEAR(double(v1[1]) / v1[2], d[1] / d[2], 1e-3);
        EXPECT_NEAR(double(v1[2]) / v1[0], d[2] / d[0], 1e-3);
    }
}

TEST(RayTracing, HitPointDirection)
{
    const double d[3] = {3, 7, 19};
    double max_range = std::numeric_limits<double>::infinity();

    // char
    {
        typedef Voxel<char> V;
        V v1 = ray(V(0, 0, 0, 0), d);
        EXPECT_NEAR(double(v1[0]) / v1[1], d[0] / d[1], 1e-1);
        EXPECT_NEAR(double(v1[1]) / v1[2], d[1] / d[2], 1e-1);
        EXPECT_NEAR(double(v1[2]) / v1[0], d[2] / d[0], 1e-1);
    }
    // short
    {
        typedef Voxel<short> V;
        V v1 = ray(V(0, 0, 0, 0), d);
        EXPECT_NEAR(double(v1[0]) / v1[1], d[0] / d[1], 1e-3);
        EXPECT_NEAR(double(v1[1]) / v1[2], d[1] / d[2], 1e-3);
        EXPECT_NEAR(double(v1[2]) / v1[0], d[2] / d[0], 1e-3);
    }
    // int
    {
        typedef Voxel<int> V;
        typedef Voxel<short> VS;
        V v0(0, 0, 0, 0);
        V::Filter filter = boost::bind(voxelPositionInRange<V>,
                                       _1, VoxelTraits<VS>::lo(), VoxelTraits<VS>::hi());
//        Timer t;
        V v1 = ray(v0, d, filter);
//        std::cout << "int: ray from " << v0 << " to " << v1 << ": " << t.secondsElapsed() << " s." << std::endl;
        EXPECT_NEAR(double(v1[0]) / v1[1], d[0] / d[1], 1e-3);
        EXPECT_NEAR(double(v1[1]) / v1[2], d[1] / d[2], 1e-3);
        EXPECT_NEAR(double(v1[2]) / v1[0], d[2] / d[0], 1e-3);
    }
    // long
    {
        typedef Voxel<long> V;
        typedef Voxel<short> VS;
        V v0(0, 0, 0, 0);
        V::Filter filter = boost::bind(voxelPositionInRange<V>,
                                       _1, VoxelTraits<VS>::lo(), VoxelTraits<VS>::hi());
//        Timer t;
        V v1 = ray(v0, d, filter);
//        std::cout << "long: ray from " << v0 << " to " << v1 << ": " << t.secondsElapsed() << " s." << std::endl;
        EXPECT_NEAR(double(v1[0]) / v1[1], d[0] / d[1], 1e-3);
        EXPECT_NEAR(double(v1[1]) / v1[2], d[1] / d[2], 1e-3);
        EXPECT_NEAR(double(v1[2]) / v1[0], d[2] / d[0], 1e-3);
    }
}

TEST(RayTracing, LineFloat)
{
    // short
    {
        typedef Voxel<short> V;
        V v0(0, 0, 0, 0);
        V v1(VoxelTraits<V>::hi(), 1, VoxelTraits<V>::hi() - 1);
//        Timer t;
        V hit = line(v0, v1);
//        std::cout << "short: line from " << v0 << " to " << hit << ": " << t.secondsElapsed() << " s." << std::endl;
        EXPECT_EQ(hit, v1);
//        t.reset();
        V hitf = lineFloat(v0, v1);
//        std::cout << "short: lineFloat from " << v0 << " to " << hitf << ": " << t.secondsElapsed() << " s." << std::endl;
        EXPECT_EQ(hitf, v1);
    }
    // int
    {
        typedef Voxel<int> V;
        V v0(0, 0, 0, 0);
        V v1(1e7, 1, 1e7 - 1);
//        Timer t;
        V hit = line(v0, v1);
//        std::cout << "int: line from " << v0 << " to " << hit << ": " << t.secondsElapsed() << " s." << std::endl;
        EXPECT_EQ(hit, v1);
//        t.reset();
        V hitf = lineFloat(v0, v1);
//        std::cout << "int: lineFloat from " << v0 << " to " << hitf << ": " << t.secondsElapsed() << " s." << std::endl;
        EXPECT_EQ(hitf, v1);
    }
}

TEST(RayTracing, HitBoundary)
{
    typedef Voxel<long> V;
    const long hi = VoxelTraits<V>::hi();
    const size_t n = 1e1;
    for (size_t i = 0; i < n; ++i)
    {
        V from(hi - randi(1e3), hi - randi(1e3), hi - randi(1e3), 0);
        const double d[3] = {randu(), randu(), randu()};
        V hit = ray<V>(from, d);
        ASSERT_TRUE((hit[0] == hi) || (hit[1] == hi) || (hit[2] == hi));
    }
}

TEST(Voxel, DistanceSquared)
{
    typedef int T;
    ASSERT_EQ(Voxel<T>(0, 0, 0, 0).distanceSquared(Voxel<T>(1, 0, 0, 0)), 1);
    ASSERT_EQ(Voxel<T>(0, 0, 0, 1).distanceSquared(Voxel<T>(0, 1, 0, 1)), 4);
    ASSERT_EQ(Voxel<T>(0, 0, 0, 2).distanceSquared(Voxel<T>(0, 0, 1, 2)), 16);
}

TEST(Voxel, ChildIterator)
{
    typedef int T;
    Voxel<T> v(-1, 0, 1, 0);
    for (typename Voxel<T>::ChildIterator it = v.childBegin(); it != v.childEnd(); ++it)
    {
        ASSERT_EQ((*it).level, -1);
        ASSERT_TRUE((*it).x == -2 || (*it).x == -1);
        ASSERT_TRUE((*it).y == 0 || (*it).y == 1);
        ASSERT_TRUE((*it).z == 2 || (*it).z == 3);
    }
}

TEST(VoxelMap, CollapseOnce)
{
    typedef Voxel<int> V;
    typedef double T;
    V::ElemType max_level = 2;
    VoxelMap<V, T> map(max_level);
    V v1(16*16 + 1, 4, 4, 0);
    fillBox<V, T>(V(0, 0, 0, 0), v1, -1, map);
    ASSERT_EQ(v1[0]*v1[1]*v1[2], map.map.size());

    typename V::Filter should_collapse = boost::bind(voxelInRange<V, T>, boost::ref(map),
                                                     _1, -std::numeric_limits<T>::infinity(), 0.0, true);
//    std::cout << map.map.size() << " voxels level 0." << std::endl;
//    Timer t;
    collapseOnce<V, T>(map, should_collapse);
//    std::cout << "Collapsing into " << map.map.size() << " voxels (level 0 to 1): " << t.secondsElapsed() << " s." << std::endl;
    ASSERT_EQ((v1[0]/2)*(v1[1]/2)*(v1[2]/2) + 1*v1[1]*v1[2], map.map.size());

//    t.reset();
    collapseOnce<V, T>(map, should_collapse);
//    std::cout << "Collapsing into " << map.map.size() << " voxels (level 1 to 2): " << t.secondsElapsed() << " s." << std::endl;
    ASSERT_EQ((v1[0]/4)*(v1[1]/4)*(v1[2]/4) + 1*v1[1]*v1[2], map.map.size());
}

TEST(VoxelMap, ExpandOnce)
{
    typedef Voxel<int> V;
    typedef double T;
    V::ElemType max_level = 2;
    VoxelMap<V, T> map(max_level);
    map[V(0, 0, 0, 2)] = -1;
    map[V(3, 0, 0, 0)] = 1;
//    listVoxels(map);
    ASSERT_EQ(1+0+1, map.map.size());

    // Expand.
    typename V::Filter should_expand = boost::bind(voxelInRange<V, T>, boost::ref(map),
                                                   _1, 0.0, std::numeric_limits<T>::infinity(), true);
//    Timer t;
    expandOnce<V, T>(map, should_expand);
//    std::cout << "Expanding into " << map.map.size() << " voxels (level 2 to 1): " << t.secondsElapsed() << " s." << std::endl;
//    listVoxels(map);
    ASSERT_EQ(0+8+1, map.map.size());

//    t.reset();
    expandOnce<V, T>(map, should_expand);
//    std::cout << "Expanding into " << map.map.size() << " voxels (level 1 to 0): " << t.secondsElapsed() << " s." << std::endl;
    ASSERT_EQ(0+7+8, map.map.size());
}

TEST(VoxelMap, MergeOnce)
{
    typedef Voxel<int> V;
    typedef double T;
    V::ElemType max_level = 2;
    VoxelMap<V, T> map(max_level);
    map[V(0, 0, 0, 2)] = -1;
    map[V(1, 1, 0, 1)] = -1;
    map[V(2, 2, 2, 0)] = -1;
//    listVoxels(map);
    ASSERT_EQ(1+1+1, map.size());
    const boost::function<T (const T &, const T &)> reduce = std::plus<T>();
//    Timer t;
    mergeOnce<V, T>(map, reduce);
//    std::cout << "Merging into " << map.size() << " voxels (level 0 to 2): " << t.secondsElapsed() << " s." << std::endl;
//    listVoxels(map);
    EXPECT_EQ(1+0+0, map.size());
    EXPECT_EQ(-3, map[V(0, 0, 0, 2)]);
}

TEST(Wrapper, GetSetVoxels)
{
    typedef Voxel<int> V;
    typedef float T;
    V::ElemType max_level = 2;
    VoxelMap<V, T> map(max_level);

    V v0(0, 0, 0, 0);
    V v1(2, 3, 4, 0);
    T val = -1;
    fillBox(v0, v1, val, map);
    ASSERT_EQ(v1[0]*v1[1]*v1[2], map.size());

    std::vector<int> x, y, level, z;
    std::vector<float> v;
    getVoxels(map, std::back_inserter(x), std::back_inserter(y), std::back_inserter(z), std::back_inserter(level),
              std::back_inserter(v));
    EXPECT_EQ(map.size(), x.size());
    EXPECT_EQ(map.size(), y.size());
    EXPECT_EQ(map.size(), z.size());
    EXPECT_EQ(map.size(), level.size());
    EXPECT_EQ(map.size(), v.size());
    for (size_t i = 0; i < v.size(); ++i)
    {
        EXPECT_LE(v0.x, x[i]);
        EXPECT_LE(v0.y, y[i]);
        EXPECT_LE(v0.z, z[i]);
        EXPECT_LE(x[i], v1.x);
        EXPECT_LE(y[i], v1.y);
        EXPECT_LE(z[i], v1.z);
        EXPECT_EQ(0, level[i]);
        EXPECT_EQ(val, v[i]);
    }
    getVoxels(map, Void(), Void(), Void(), Void(), Void());

    T val2 = -2;
    std::vector<float> v2(v.size(), val2);
    setVoxels(map, x.begin(), x.end(), y.begin(), z.begin(), level.begin(), v2.begin());

    std::vector<float> v3;
    getVoxels(map, x.begin(), x.end(), y.begin(), z.begin(), level.begin(), std::back_inserter(v3));
    EXPECT_EQ(v.size(), v3.size());
    for (size_t i = 0; i < v3.size(); ++i)
    {
        EXPECT_EQ(val2, v3[i]);
    }
}

}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    int ret = RUN_ALL_TESTS();
    return ret;
}

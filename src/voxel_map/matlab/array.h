#ifndef VOXEL_MAP_ARRAY_H
#define VOXEL_MAP_ARRAY_H

#include <mex.h>
#include <voxel_map/strided.h>

namespace
{

template<typename T>
class ConstArray
{
public:
    typedef voxel_map::Strided<const T *> Iter;

    ConstArray(const mxArray *arr):
        carr_(arr)
    {
        assert(check());
    }
    const mxArray * array() const
    {
        assert(carr_ != NULL);
        return carr_;
    }
    size_t ndims() const { return mxGetNumberOfDimensions(array()); }
    size_t numel() const { return mxGetNumberOfElements(array()); }
    size_t size(size_t dim) const
    {
        assert(dim < ndims());
        return mxGetDimensions(array())[dim];
    }
    size_t stride(size_t dim) const
    {
        assert(dim < ndims());
        size_t s = 1;
        for (size_t i = 0; i < dim; ++i)
            s *= size(i);
        return s;
    }

    // Linear iterators
    const T * begin() const { return static_cast<const T *>(mxGetData(array())); }
    const T * end() const { return begin() + numel(); }

    // Strided (dimension) iterators
    Iter begin(const size_t dim, const size_t skip = 0) const
    {
        return Iter(begin() + skip, stride(dim));
    }
    Iter end(const size_t dim, const size_t skip = 0) const
    {
        return begin(dim, skip) + size(dim);
    }

    const T & value() const
    {
        assert(mxIsScalar(array()));
        return *begin();
    }
    const T & value(const size_t i) const
    {
        return begin()[i];
    }
    const T & operator[](const size_t i) const { return value(i); }
    const T & operator()(const size_t i) const { return value(i); }
    operator const T &() const { return value(); }
    operator const mxArray *() const { return array(); }
protected:
    const mxArray *carr_;
    ConstArray():
        carr_(NULL) {}
    void check() const;
};

template<>
void ConstArray<double>::check() const
{
    assert(mxIsDouble(array()));
}

template<typename T>
class Array: public ConstArray<T>
{
public:
    typedef voxel_map::Strided<T *> Iter;

    Array(mxArray *arr):
        ConstArray<T>(arr), arr_(arr)
    {
        assert(check());
    }
    Array(const size_t m, const size_t n)
    {
        ConstArray<T>::carr_ = arr_ = create(m, n);
        assert(check());
    }
    mxArray * array()
    {
        assert(arr != NULL);
        return arr_;
    }

    // Linear iterators
    T * begin() { return static_cast<T *>(mxGetData(array())); }
    T * end() { return begin() + ConstArray<T>::numel(); }

    // Strided (dimension) iterators
    Iter begin(const size_t dim, const size_t skip = 0)
    {
        return Iter(begin() + skip, ConstArray<T>::stride(dim));
    }
    Iter end(const size_t dim, const size_t skip = 0)
    {
        return begin(dim, skip) + ConstArray<T>::size(dim);
    }

    T & value()
    {
        assert(mxIsScalar(array()));
        return *begin();
    }
    T & value(const size_t i)
    {
        return begin()[i];
    }
    T & operator[](const size_t i) { return value(i); }
    T & operator()(const size_t i) { return value(i); }
    operator T &() { return value(); }
    operator mxArray *() { return array(); }
    operator ConstArray<T>() { return ConstArray<T>(array()); }
protected:
    mxArray *arr_;
    mxArray * create(const size_t m, const size_t n);
    void check() const;
};

template<>
mxArray * Array<double>::create(const size_t m, const size_t n)
{
    return mxCreateDoubleMatrix(m, n, mxREAL);
}

template<>
void Array<double>::check() const
{
    assert(ConstArray<double>::check());
    assert(mxIsDouble(array()));
}

}

#endif // VOXEL_MAP_ARRAY_H


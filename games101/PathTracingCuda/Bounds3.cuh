#pragma once

#ifndef __BOUNDS3_HPP__
#define __BOUNDS3_HPP__

#include "Ray.cuh"
#include "Vector.cuh"
#include <limits>
#include <array>
#include "global.cuh"

class Bounds3 : public CudaMemory
{
public:
    Vector3f pMin, pMax; // two points to specify the bounding box
    __device__ Bounds3()
    {
        double minNum = LOWEST_DOUBLE;
        double maxNum = MAX_DOUBLE;
        pMax = Vector3f(minNum, minNum, minNum);
        pMin = Vector3f(maxNum, maxNum, maxNum);
    }
    Bounds3(bool bHost)
    {
        double minNum = LOWEST_DOUBLE;
        double maxNum = MAX_DOUBLE;
        pMax = Vector3f(minNum, minNum, minNum, bHost);
        pMin = Vector3f(maxNum, maxNum, maxNum, bHost);
    }
    __device__ Bounds3(const Vector3f p) : pMin(p), pMax(p) {}
    __device__ Bounds3(const Vector3f p1, const Vector3f p2)
    {
        pMin = Vector3f(cuda::min(p1.x, p2.x), cuda::min(p1.y, p2.y), cuda::min(p1.z, p2.z));
        pMax = Vector3f(cuda::max(p1.x, p2.x), cuda::max(p1.y, p2.y), cuda::max(p1.z, p2.z));
    }
    
    Bounds3(const Vector3f p1, const Vector3f p2, bool bHost)
    {
        pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z), bHost);
        pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z), bHost);
    }

    __device__ Vector3f Diagonal() const { return pMax - pMin; }
    __device__ int maxExtent() const
    {
        Vector3f d = Diagonal();
        if (d.x > d.y && d.x > d.z)
            return 0;
        else if (d.y > d.z)
            return 1;
        else
            return 2;
    }

    __device__ double SurfaceArea() const
    {
        Vector3f d = Diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    __device__ Vector3f Centroid() { return pMin * 0.5 + pMax * 0.5; }
    __device__ Bounds3 Intersect(const Bounds3 &b) { return Bounds3(Vector3f(cuda::max(pMin.x, b.pMin.x), cuda::max(pMin.y, b.pMin.y), cuda::max(pMin.z, b.pMin.z)), Vector3f(cuda::min(pMax.x, b.pMax.x), cuda::min(pMax.y, b.pMax.y), cuda::min(pMax.z, b.pMax.z))); }

    __device__ Vector3f Offset(const Vector3f &p) const
    {
        Vector3f o = p - pMin;
        if (pMax.x > pMin.x)
            o.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y)
            o.y /= pMax.y - pMin.y;
        if (pMax.z > pMin.z)
            o.z /= pMax.z - pMin.z;
        return o;
    }

    __device__ bool Overlaps(const Bounds3 &b1, const Bounds3 &b2)
    {
        bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
        bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
        bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
        return (x && y && z);
    }

    __device__ bool Inside(const Vector3f &p, const Bounds3 &b)
    {
        return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
                p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
    }
    __device__ const Vector3f &operator[](int i) const
    {
        return (i == 0) ? pMin : pMax;
    }

    __device__ bool IntersectP(const Ray &ray, const Vector3f &invDir, const bool dirIsNeg[3]) const
    {
        // Home Work Begin
        Vector3f tMin = (pMin - ray.origin) * invDir;
        Vector3f tMax = (pMax - ray.origin) * invDir;
        if (!dirIsNeg[0])
        {
            cuda::swap(tMin.x, tMax.x);
        }
        if (!dirIsNeg[1])
        {
            cuda::swap(tMin.y, tMax.y);
        }
        if (!dirIsNeg[2])
        {
            cuda::swap(tMin.z, tMax.z);
        }
        float tEnter = cuda::max(tMin.x, cuda::max(tMin.y, tMin.z));
        float tExit = cuda::max(tMax.x, cuda::max(tMax.y, tMax.z));
        // printf("%f %f %f %f %f %f %d\n", pMin.x, pMin.y, pMin.z, pMax.x, pMax.y, pMax.z, tEnter <= tExit && tExit >= 0.0f);
        // return tEnter <= tExit && tExit >= 0.0f;
        return false;
        // Home Work End
    }
};

__device__ static Bounds3 Union(const Bounds3 &b1, const Bounds3 &b2)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
    return ret;
}

__device__ static Bounds3 Union(const Bounds3 &b, const Vector3f &p)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b.pMin, p);
    ret.pMax = Vector3f::Max(b.pMax, p);
    return ret;
}

static Bounds3 Union(const Bounds3 &b, const Vector3f &p, bool bHost)
{
    Bounds3 ret(bHost);
    ret.pMin = Vector3f::Min(b.pMin, p, bHost);
    ret.pMax = Vector3f::Max(b.pMax, p, bHost);
    return ret;
}

#endif

#pragma once

#ifndef __VECTOR_HPP__
#define __VECTOR_HPP__

#include "global.hpp"
#include "CudaMemory.hpp"

class Vector3f : public CudaMemory
{
public:
    float x, y, z;
    __device__ Vector3f() : x(0), y(0), z(0) {}
    __device__ Vector3f(float xx) : x(xx), y(xx), z(xx) {}
    __device__ Vector3f(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {}
    Vector3f(float xx, float yy, float zz, bool bHost) : x(xx), y(yy), z(zz) {}
    __device__ float norm() { return sqrt(x * x + y * y + z * z); }
    float norm(bool bHost) { return sqrt(x * x + y * y + z * z); }
    __device__ Vector3f normalized()
    {
        float n = std::sqrt(x * x + y * y + z * z);
        return Vector3f(x / n, y / n, z / n);
    }
    __device__ Vector3f operator*(const float &r) const { return Vector3f(x * r, y * r, z * r); }
    __device__ Vector3f operator/(const float &r) const { return Vector3f(x / r, y / r, z / r); }
    __device__ Vector3f operator*(const Vector3f &v) const { return Vector3f(x * v.x, y * v.y, z * v.z); }
    Vector3f mul(const Vector3f &v) const { return Vector3f(x * v.x, y * v.y, z * v.z, true); }
    __device__ Vector3f operator-(const Vector3f &v) const { return Vector3f(x - v.x, y - v.y, z - v.z); }
    Vector3f sub(const Vector3f &v) const { return Vector3f(x - v.x, y - v.y, z - v.z, true); }
    __device__ Vector3f operator+(const Vector3f &v) const { return Vector3f(x + v.x, y + v.y, z + v.z); }
    Vector3f add(const Vector3f &v) const { return Vector3f(x + v.x, y + v.y, z + v.z, true); }
    __device__ Vector3f operator-() const { return Vector3f(-x, -y, -z); }
    __device__ Vector3f &operator+=(const Vector3f &v)
    {
        x += v.x, y += v.y, z += v.z;
        return *this;
    }
    __device__ double operator[](int index) const
    {
        return (&x)[index];
    }
    __device__ static Vector3f Min(const Vector3f &p1, const Vector3f &p2) { return Vector3f(cuda::min(p1.x, p2.x), cuda::min(p1.y, p2.y), cuda::min(p1.z, p2.z)); }
    __device__ static Vector3f Max(const Vector3f &p1, const Vector3f &p2) { return Vector3f(cuda::max(p1.x, p2.x), cuda::max(p1.y, p2.y), cuda::max(p1.z, p2.z)); }
    static Vector3f Min(const Vector3f &p1, const Vector3f &p2, bool bHost) { return Vector3f(std::min(p1.x, p2.x), std::min(p1.y, p2.y), std::min(p1.z, p2.z), bHost); }
    static Vector3f Max(const Vector3f &p1, const Vector3f &p2, bool bHost) { return Vector3f(std::max(p1.x, p2.x), std::max(p1.y, p2.y), std::max(p1.z, p2.z), bHost); }
};

__device__ static Vector3f normalize(const Vector3f &v)
{
    float mag2 = v.x * v.x + v.y * v.y + v.z * v.z;
    if (mag2 > 0)
    {
        float invMag = 1 / sqrtf(mag2);
        return Vector3f(v.x * invMag, v.y * invMag, v.z * invMag);
    }
    return v;
}

static Vector3f normalize(const Vector3f &v, bool bHost)
{
    float mag2 = v.x * v.x + v.y * v.y + v.z * v.z;
    if (mag2 > 0)
    {
        float invMag = 1 / sqrtf(mag2);
        return Vector3f(v.x * invMag, v.y * invMag, v.z * invMag, bHost);
    }
    return v;
}

__device__ static Vector3f lerp(const Vector3f &a, const Vector3f &b, const float &t)
{
    return a * (1 - t) + b * t;
}
__device__ static float dotProduct(const Vector3f &a, const Vector3f &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
__device__ static Vector3f crossProduct(const Vector3f &a, const Vector3f &b) { return Vector3f(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }
static Vector3f crossProduct(const Vector3f &a, const Vector3f &b, bool bHost) { return Vector3f(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x, bHost); }

class Vector2f : public CudaMemory
{
public:
    Vector2f() : x(0), y(0) {}
    Vector2f(float xx) : x(xx), y(xx) {}
    Vector2f(float xx, float yy) : x(xx), y(yy) {}
    Vector2f operator*(const float &r) const { return Vector2f(x * r, y * r); }
    Vector2f operator+(const Vector2f &v) const { return Vector2f(x + v.x, y + v.y); }
    float x, y;
};

#endif
#pragma once

#ifndef __RAY_HPP__
#define __RAY_HPP__

#include "Vector.cuh"

struct Ray
{
    Vector3f origin;
    Vector3f direction, direction_inv;
    double t;
    double t_min, t_max;

    __device__ Ray(const Vector3f *ori, const Vector3f &dir, const double _t = 0.0) : origin(*ori), direction(dir), t(_t)
    {
        direction_inv = Vector3f(1.f / direction.x, 1.f / direction.y, 1.f / direction.z);
        t_min = 0.0;
        t_max = MAX_DOUBLE;
    }
    __device__ Vector3f operator()(double t) const { return origin + direction * t; }

    Ray(const Vector3f *ori, const Vector3f &dir, bool bHost, const double _t = 0.0) : origin(*ori), direction(dir), t(_t)
    {
        direction_inv = Vector3f(1.f / direction.x, 1.f / direction.y, 1.f / direction.z, bHost);
        t_min = 0.0;
        t_max = MAX_DOUBLE;
    }
};

#endif
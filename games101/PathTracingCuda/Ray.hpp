#pragma once

#ifndef __RAY_HPP__
#define __RAY_HPP__

#include "Vector.hpp"

struct Ray
{
    Vector3f origin;
    Vector3f direction, direction_inv;
    double t;
    double t_min, t_max;

    __device__ Ray(const Vector3f *ori, const Vector3f &dir, const double _t = 0.0) : origin(*ori), direction(dir), t(_t)
    {
        direction_inv = Vector3f(1. / direction.x, 1. / direction.y, 1. / direction.z);
        t_min = 0.0;
        t_max = MAX_FLOAT;
    }
    __device__ Vector3f operator()(double t) const { return origin + direction * t; }

    Ray(const Vector3f *ori, const Vector3f &dir, bool bHost, const double _t = 0.0) : origin(*ori), direction(dir), t(_t)
    {
        direction_inv = Vector3f(1. / direction.x, 1. / direction.y, 1. / direction.z, bHost);
        t_min = 0.0;
        t_max = MAX_FLOAT;
    }
};

#endif
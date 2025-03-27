#pragma once
#ifndef __GLOBAL_HPP__
#define __GLOBAL_HPP__

#include <cmath>
#include <curand_kernel.h>
#include <cuda.h>
#include <algorithm>
#include <cassert>

constexpr float EPSILON = 1e-6f;
constexpr float MAX_FLOAT = std::numeric_limits<float>::max();
constexpr float INFINITY_FLOAT = std::numeric_limits<float>::infinity();
constexpr double MAX_DOUBLE = std::numeric_limits<double>::max();
constexpr double LOWEST_DOUBLE = std::numeric_limits<double>::lowest();
constexpr uint32_t MAX_OBJECT = 16;
constexpr uint32_t MAX_LIGHT = 16;
constexpr uint32_t MAX_PRIMITIVES = 24;
constexpr uint32_t MAX_MESH_OBJECT = 64;

static bool SolveQuadratic(const float &a, const float &b, const float &c, float &x0, float &x1)
{
    float discr = b * b - 4 * a * c;
    if (discr < 0)
        return false;
    else if (discr == 0)
        x0 = x1 = -0.5f * b / a;
    else
    {
        float q = (b > 0.0f) ? -0.5f * (b + std::sqrt(discr)) : -0.5f * (b - std::sqrt(discr));
        x0 = q / a;
        x1 = c / q;
    }
    if (x0 > x1)
    {
        float temp = x0;
        x0 = x1;
        x1 = temp;
    }
    return true;
}

namespace cuda
{
    __device__ static void swap(float &a, float &b)
    {
        float t = a;
        a = b;
        b = t;
    }

    __device__ static float max(float a, float b) { return a > b ? a : b; }

    __device__ static float min(float a, float b) { return a < b ? a : b; }

    __device__ static int random_count = 0;
    __device__ static float get_random_float(int some_id)
    {
        curandState devStates;
        curand_init(some_id + random_count, some_id + random_count++, 0, &devStates);
        float rd = curand_uniform(&devStates);
        return rd;
    }

    __device__ static float clamp(const float &lo, const float &hi, const float &v) { return max(lo, min(hi, v)); }

    __device__ static bool solveQuadratic(const float &a, const float &b, const float &c, float &x0, float &x1)
    {
        float discr = b * b - 4 * a * c;
        if (discr < 0)
            return false;
        else if (discr == 0)
            x0 = x1 = -0.5f * b / a;
        else
        {
            float q = (b > 0.0f) ? -0.5f * (b + sqrt(discr)) : -0.5f * (b - sqrt(discr));
            x0 = q / a;
            x1 = c / q;
        }
        if (x0 > x1)
        {
            float temp = x0;
            x0 = x1;
            x1 = temp;
        }
        return true;
    }
}

#endif
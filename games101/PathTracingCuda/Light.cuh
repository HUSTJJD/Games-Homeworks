#pragma once

#ifndef __LIGHT_HPP__
#define __LIGHT_HPP__

#include "Vector.cuh"

class Light
{
public:
    Light(const Vector3f &p, const Vector3f &i) : position(p), intensity(i) {}
    virtual ~Light() = default;
    Vector3f position;
    Vector3f intensity;
};

#endif
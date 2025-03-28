#pragma once

#ifndef __INTERSECTION_HPP__
#define __INTERSECTION_HPP__

#include "Vector.hpp"
#include "Material.hpp"

class Object;

struct Intersection
{
    __device__ Intersection()
    {
        happened = false;
        coords = Vector3f();
        normal = Vector3f();
        distance = MAX_DOUBLE;
        obj = nullptr;
        m = nullptr;
    }
    bool happened;
    Vector3f coords;
    Vector3f tcoords;
    Vector3f normal;
    Vector3f emit;
    double distance;
    Object *obj;
    Material *m;
};

#endif

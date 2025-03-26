
#pragma once

#ifndef __BVH_HPP__
#define __BVH_HPP__

#include <vector>
#include <memory>
#include <ctime>
#include "CudaMemory.hpp"
#include "Ray.hpp"
#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Vector.hpp"

struct BVHBuildNode : public CudaMemory
{
    Bounds3 bounds;
    BVHBuildNode *left;
    BVHBuildNode *right;
    Object *object;
    float area;

public:
    int splitAxis = 0, firstPrimOffset = 0, nPrimitives = 0;
    // BVHBuildNode Public Methods
    BVHBuildNode()
    {
        bounds = Bounds3(true);
        left = nullptr;
        right = nullptr;
        object = nullptr;
    }
};

// BVHAccel Declarations
static int leafNodes, totalLeafNodes, totalPrimitives, interiorNodes;
class BVHAccel : public CudaMemory
{
public:
    // BVHAccel Public Types
    enum class SplitMethod
    {
        NAIVE,
        SAH
    };

    BVHBuildNode *root;
    const int maxPrimsInNode;
    const SplitMethod splitMethod;
    Object *primitives[MAX_PRIMITIVES];
    int primitives_num = 0;

    BVHAccel(Object *const objects[], const int objects_num, int maxPrimsInNode = 1, SplitMethod splitMethod = SplitMethod::NAIVE) : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod)
    {
        if (primitives_num + objects_num > MAX_PRIMITIVES)
            return;

        for (int i = 0; i < objects_num; i++)
        {
            primitives[primitives_num++] = objects[i];
        }
        std::vector<Object *> primitives_vector;
        for (int i = 0; i < primitives_num; i++)
        {
            primitives_vector.push_back(primitives[i]);
        }
        if (primitives_vector.empty())
            return;
        root = recursiveBuild(primitives_vector);
    }

    Bounds3 WorldBound() const;

    ~BVHAccel();

    BVHBuildNode *recursiveBuild(std::vector<Object *> objects);

    __device__ Intersection Intersect(const Ray &ray) const
    {
        Intersection isect;
        if (!root)
        {
            return isect;
        }
        isect = getIntersection(root, ray);
        return isect;
    }

    __device__ Intersection getIntersection(BVHBuildNode *node, const Ray &ray) const;

    __device__ void getSample(BVHBuildNode *node, float p, Intersection &pos, float &pdf, int thread_id);

    __device__ void Sample(Intersection &pos, float &pdf, int thread_id)
    {
        float p = std::sqrt(cuda::get_random_float(thread_id)) * root->area;
        getSample(root, p, pos, pdf, thread_id);
        pdf /= root->area;
    }
};

#endif
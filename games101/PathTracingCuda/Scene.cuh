#pragma once

#ifndef __SCENE_HPP__
#define __SCENE_HPP__

#include <vector>
#include "Vector.cuh"
#include "CudaMemory.cuh"
#include "Light.cuh"
#include "BVH.cuh"
#include "Ray.cuh"
#include "Triangle.cuh"

class Scene : public CudaMemory
{
public:
    // setting up options
    int width = 1280;
    int height = 960;
    double fov = 40;
    Vector3f backgroundColor = Vector3f(0.235294f, 0.67451f, 0.843137f);
    int maxDepth = 1;
    float RussianRoulette = 0.8f;
    BVHAccel *bvh;
    Object *objects[MAX_OBJECT];
    int objects_num = 0;
    Light *lights[MAX_LIGHT];
    int lights_num = 0;

    Scene(const int w, const int h) : width(w), height(h) {}

    void Add(Object *object)
    {
        if (objects_num > MAX_OBJECT)
        {
            return;
        }
        objects[objects_num++] = object;
    }

    void Add(Light *light)
    {
        if (lights_num > MAX_LIGHT)
        {
            return;
        }
        lights[lights_num++] = light;
    }

    void buildBVH()
    {
        printf(" - Generating BVH... objects_num: %d\n\n", objects_num);
        time_t start, stop;
        time(&start);
        this->bvh = new BVHAccel(objects, objects_num, 1, BVHAccel::SplitMethod::NAIVE);
        time(&stop);
        double diff = difftime(stop, start);
        int hrs = (int)diff / 3600;
        int mins = ((int)diff / 60) - (hrs * 60);
        int secs = (int)diff - (hrs * 3600) - (mins * 60);
        printf(" - BVH Generation complete: \n - Time Taken: %i hrs, %i mins, %i secs\n\n",
               hrs, mins, secs);
    }

    __device__ Intersection intersect(const Ray &ray) const { return this->bvh->Intersect(ray); }

    __device__ void sampleLight(Intersection &pos, float &pdf, int thread_id) const
    {
        float emit_area_sum = 0;
        for (int k = 0; k < objects_num; ++k)
        {
            if (((MeshTriangle *)objects[k])->hasEmit())
            {
                emit_area_sum += ((MeshTriangle *)objects[k])->getArea();
            }
        }
        float p = cuda::get_random_float(thread_id) * emit_area_sum;
        emit_area_sum = 0;
        for (int k = 0; k < objects_num; ++k)
        {
            if (((MeshTriangle *)objects[k])->hasEmit())
            {
                emit_area_sum += ((MeshTriangle *)objects[k])->getArea();
                if (p <= emit_area_sum)
                {
                    ((MeshTriangle *)objects[k])->Sample(pos, pdf, thread_id);
                    break;
                }
            }
        }
    }

    __device__ bool trace(const Ray &ray, const std::vector<Object *> &objects, float &tNear, uint32_t &index, Object **hitObject)
    {
        *hitObject = nullptr;
        for (int k = 0; k < objects_num; ++k)
        {
            float tNearK = MAX_FLOAT;
            uint32_t indexK;
            Vector2f uvK;
            if (((MeshTriangle *)objects[k])->intersect(ray, tNearK, indexK) && tNearK < tNear)
            {
                *hitObject = objects[k];
                tNear = tNearK;
                index = indexK;
            }
        }

        return (*hitObject != nullptr);
    }

    __device__ Vector3f castRay(const Ray &ray, int depth, int thread_id) const;
};

#endif
#pragma once

#ifndef __RENDERER_HPP__
#define __RENDERER_HPP__

#include "Scene.cuh"

struct hit_payload
{
    float tNear;
    uint32_t index;
    Vector2f uv;
    Object *hit_obj;
};

class Renderer
{
public:
    void Render(Scene *scene, int spp, Vector3f &in_eye_pos, Vector3f *in_framebuffer);
};

#endif

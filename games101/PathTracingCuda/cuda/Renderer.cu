//
// Created by goksu on 2/25/20.
//
#include "../Renderer.hpp"
#include "../Scene.hpp"
#include "../BVH.hpp"

__device__ void BVHAccel::getSample(BVHBuildNode *node, float oPos, Intersection &pos, float &pdf, int thread_id)
{
    if (node->left == nullptr || node->right == nullptr)
    {

        if (node->object->type == EObjectType::MeshTriangle)
        {
            ((MeshTriangle *)node->object)->Sample(pos, pdf, thread_id);
        }
        else if (node->object->type == EObjectType::Triangle)
        {
            ((Triangle *)node->object)->Sample(pos, pdf, thread_id);
        }
        pdf *= node->area;
        return;
    }
    if (oPos < node->left->area)
        getSample(node->left, oPos, pos, pdf, thread_id);
    else
        getSample(node->right, oPos - node->left->area, pos, pdf, thread_id);
}

__device__ Intersection BVHAccel::getIntersection(BVHBuildNode *node, const Ray &ray) const
{
    // Home Work Begin
    Intersection inter;
    // 判断光线方向是否为负
    // 光线方向
    float x = ray.direction.x;
    float y = ray.direction.y;
    float z = ray.direction.z;
    // 判断坐标是否为负
    bool dirsIsNeg[3] = {x > 0, y > 0, z > 0};
    // 判断结点的包围盒与光线是否相交
    if (node->bounds.IntersectP(ray, ray.direction_inv, dirsIsNeg) == false)
        return inter;
    if (node->left == nullptr && node->right == nullptr)
    {
        if (node->object->type == EObjectType::MeshTriangle)
        {
            inter = ((MeshTriangle *)node->object)->getIntersection(ray);
        }
        else if (node->object->type == EObjectType::Triangle)
        {
            inter = ((Triangle *)node->object)->getIntersection(ray);
        }
        return inter;
    }
    // 递归判断子节点是否存在与光线相交的情况
    auto hit1 = getIntersection(node->left, ray);
    auto hit2 = getIntersection(node->right, ray);
    if (hit1.distance < hit2.distance)
        return hit1;
    return hit2;
}

__device__ Vector3f Scene::castRay(const Ray &ray, int depth, int thread_id) const
{
    Intersection inter = intersect(ray);
    if (inter.happened)
    {
        // 如果射线第一次打到光源，直接返回
        if (inter.m->hasEmission())
        {
            if (depth == 0)
                return inter.m->getEmission();
            else
                return Vector3f(0, 0, 0);
        }
        Vector3f L_dir(0, 0, 0);
        Vector3f L_indir(0, 0, 0);
        // 随机 sample 灯光，用该 sample 的结果判断射线是否击中光源
        Intersection lightInter;
        float pdf_light = 0.0f;
        sampleLight(lightInter, pdf_light, thread_id);
        // 物体表面法线
        auto &N = inter.normal;
        // 灯光表面法线
        auto &NN = lightInter.normal;
        auto &objPos = inter.coords;
        auto &lightPos = lightInter.coords;
        auto diff = lightPos - objPos;
        auto lightDir = diff.normalized();
        float lightDistance = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;
        Ray light(&objPos, lightDir);
        Intersection light2obj = intersect(light);
        // 如果反射击中光源
        if (light2obj.happened && (light2obj.coords - lightPos).norm() < 1e-2)
        {
            Vector3f f_r = inter.m->eval(ray.direction, lightDir, N);
            L_dir = lightInter.emit * f_r * dotProduct(lightDir, N) * dotProduct(-lightDir, NN) / lightDistance / pdf_light;
        }
        if (cuda::get_random_float(thread_id) < RussianRoulette)
        {
            Vector3f nextDir = inter.m->sample(ray.direction, N, thread_id).normalized();
            Ray nextRay(&objPos, nextDir);
            Intersection nextInter = intersect(nextRay);
            if (nextInter.happened && !nextInter.m->hasEmission())
            {
                float pdf = inter.m->pdf(ray.direction, nextDir, N);
                Vector3f f_r = inter.m->eval(ray.direction, nextDir, N);
                L_indir = castRay(nextRay, depth + 1, thread_id) * f_r * dotProduct(nextDir, N) / pdf / RussianRoulette;
            }
        }
        return L_dir + L_indir;
    }
    return Vector3f(0, 0, 0);
}

__global__ void kernel(Scene *scene, int spp, Vector3f *eye_pos, float scale, float imageAspectRatio, Vector3f *framebuffer)
{
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    int offset = gridDim.x * blockDim.x;
    int total_count = scene->width * scene->height * spp;
    while (tid < total_count)
    {
        // 修正索引计算：按像素和采样分解
        int pixel_id = tid / spp; // 像素ID
        int i = pixel_id % scene->width;
        int j = pixel_id / scene->width;

        // 检查i和j是否越界
        if (i >= scene->width || j >= scene->height)
        {
            tid += offset;
            printf("out\n");
            continue;
        }

        float x = (2 * (i + 0.5) / (float)scene->width - 1) * imageAspectRatio * scale;
        float y = (1 - 2 * (j + 0.5) / (float)scene->height) * scale;
        Vector3f dir = normalize(Vector3f(-x, y, 1));

        // 累加前确保原子操作或同步
        Vector3f color = scene->castRay(Ray(eye_pos, dir), 0, tid) / spp;
        atomicAdd(&framebuffer[j * scene->width + i].x, color.x);
        atomicAdd(&framebuffer[j * scene->width + i].y, color.y);
        atomicAdd(&framebuffer[j * scene->width + i].z, color.z);

        tid += offset;
    }
}

void validGPUAvailable()
{
    int count;
    cudaGetDeviceCount(&count);
    if (count == 0)
    {
        printf("no cuda gpu detected\n");
        exit(-1);
    }
    else
    {
        printf("cuda gpu available\n");
    }
}

void setCudaLimit()
{
    size_t size_heap, size_stack;
    // 当出现内存错误时，适当调整此处参数
    cudaDeviceSetLimit(cudaLimitMallocHeapSize, 10240000 * 8);
    cudaDeviceSetLimit(cudaLimitStackSize, 128 * 1024);
    cudaDeviceGetLimit(&size_heap, cudaLimitMallocHeapSize);
    cudaDeviceGetLimit(&size_stack, cudaLimitStackSize);
    printf("Heap size found to be %d; Stack size found to be %d\n", (int)size_heap, (int)size_stack);
}

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(Scene *scene, int spp, Vector3f &in_eye_pos, Vector3f *in_framebuffer)
{
    validGPUAvailable();
    setCudaLimit();

    auto pixel_num = scene->height * scene->width;

    Vector3f *framebuffer;
    cudaMalloc((void **)&framebuffer, pixel_num * sizeof(Vector3f));
    cudaMemcpy(framebuffer, in_framebuffer, pixel_num * sizeof(Vector3f), cudaMemcpyHostToDevice);

    Vector3f *eye_pos;
    cudaMalloc((void **)&eye_pos, sizeof(Vector3f));
    cudaMemcpy(eye_pos, &in_eye_pos, sizeof(Vector3f), cudaMemcpyHostToDevice);

    float scale = std::tan(scene->fov * 0.5 / 180.f * M_PI);
    float imageAspectRatio = (float)scene->width / (float)scene->height;

    int threads_per_block = spp;
    int blocks_needed = (pixel_num * spp + threads_per_block - 1) / threads_per_block;
    kernel<<<blocks_needed, threads_per_block>>>(scene, spp, eye_pos, scale, imageAspectRatio, framebuffer);
    cudaDeviceSynchronize();

    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess)
    {
        printf("After CUDA Error: %s\n", cudaGetErrorString(err));
    }
    cudaMemcpy(in_framebuffer, framebuffer, pixel_num * sizeof(Vector3f), cudaMemcpyDeviceToHost);
    cudaFree(eye_pos);
    cudaFree(framebuffer);
    printf("Render_GPU done\n");
}

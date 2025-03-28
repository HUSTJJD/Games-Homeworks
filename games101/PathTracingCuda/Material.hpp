#pragma once

#ifndef __MATERIAL_HPP__
#define __MATERIAL_HPP__

#include "Vector.hpp"

enum MaterialType
{
    DIFFUSE,
    Microfacet
};

class Material : public CudaMemory
{
private:
    __device__ Vector3f reflect(const Vector3f &I, const Vector3f &N) const
    {
        return I - N * 2 * dotProduct(I, N);
    }

    __device__ Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) const
    {
        float cosi = cuda::clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0)
        {
            cosi = -cosi;
        }
        else
        {
            std::swap(etai, etat);
            n = -N;
        }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? Vector3f() : I * eta + n * (eta * cosi - sqrtf(k));
    }

    __device__ void fresnel(const Vector3f &I, const Vector3f &N, const float &ior, float &kr) const
    {
        float cosi = cuda::clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi > 0)
        {
            cuda::swap(etai, etat);
        }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(cuda::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1)
        {
            kr = 1;
        }
        else
        {
            float cost = sqrtf(cuda::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
    }

    __device__ Vector3f toWorld(const Vector3f &a, const Vector3f &N)
    {
        Vector3f B, C;
        if (std::fabs(N.x) > std::fabs(N.y))
        {
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            C = Vector3f(N.z * invLen, 0.0f, -N.x * invLen);
        }
        else
        {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            C = Vector3f(0.0f, N.z * invLen, -N.y * invLen);
        }
        B = crossProduct(C, N);
        return B * a.x + C * a.y + N * a.z;
    }

    __device__ float DistributionGGX(Vector3f N, Vector3f H, float roughness)
    {
        float a = roughness * roughness;
        float a2 = a * a;
        float NdotH = cuda::max(dotProduct(N, H), 0.0f);
        float NdotH2 = NdotH * NdotH;

        float nom = a2;
        float denom = (NdotH2 * (a2 - 1.0) + 1.0);
        denom = M_PI * denom * denom;

        return nom / cuda::max(denom, 0.0001f);
    }

    __device__ float GeometrySchlickGGX(float NdotV, float roughness)
    {
        float a = roughness;
        float k = (a * a) / 2.0f;

        float nom = NdotV;
        float denom = NdotV * (1.0f - k) + k;

        return nom / denom;
    }

    __device__ float GeometrySmith(float roughness, float NoV, float NoL)
    {
        float ggx2 = GeometrySchlickGGX(NoV, roughness);
        float ggx1 = GeometrySchlickGGX(NoL, roughness);

        return ggx1 * ggx2;
    }

public:
    MaterialType m_type;
    Vector3f m_emission;
    float ior;
    Vector3f Kd, Ks;
    float specularExponent;

    __device__ Material(MaterialType t = DIFFUSE, Vector3f e = Vector3f(0, 0, 0)) : m_type(t), m_emission(e) {}
    __device__ MaterialType getType() { return m_type; }
    __device__ Vector3f getColorAt(double u, double v) { return Vector3f(); }
    __device__ Vector3f getEmission() { return m_emission; }
    __device__ bool hasEmission()
    {
        if (m_emission.norm() > EPSILON)
            return true;
        else
            return false;
    }

    __device__ Vector3f sample(const Vector3f &wi, const Vector3f &N, int thread_id)
    {
        switch (m_type)
        {
        case DIFFUSE:
        {
            float x_1 = cuda::get_random_float(thread_id), x_2 = cuda::get_random_float(thread_id);
            float z = std::fabs(1.0f - 2.0f * x_1);
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r * std::cos(phi), r * std::sin(phi), z);
            return toWorld(localRay, N);
            break;
        }
        case Microfacet:
        {
            // uniform sample on the hemisphere在半球上均匀采样
            float x_1 = cuda::get_random_float(thread_id), x_2 = cuda::get_random_float(thread_id);
            // z∈[0,1]，是随机半球方向的z轴向量
            float z = std::fabs(1.0f - 2.0f * x_1);
            // r是半球半径随机向量以法线为旋转轴的半径
            // phi是r沿法线旋转轴的旋转角度
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;    // phi∈[0,2*pi]
            Vector3f localRay(r * std::cos(phi), r * std::sin(phi), z); // 半球面上随机的光线的弹射方向
            return toWorld(localRay, N);                                // 转换到世界坐标
            break;
        }
        }
        return Vector3f(0);
    }

    __device__ float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N)
    {
        switch (m_type)
        {
        case DIFFUSE:
        {
            if (dotProduct(wo, N) > 0.0f)
                return 0.5f / M_PI;
            else
                return 0.0f;
            break;
        }
        case Microfacet:
        {
            if (dotProduct(wo, N) > 0.0f)
                return 0.5f / M_PI;
            else
                return 0.0f;
            break;
        }
        }
        return 0.0f;
    }

    __device__ Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N)
    {
        switch (m_type)
        {
        case DIFFUSE:
        {
            float cosalpha = dotProduct(N, wo);
            if (cosalpha > 0.0f)
            {
                Vector3f diffuse = Kd / M_PI;
                return diffuse;
            }
            else
                return Vector3f();
            break;
        }
        case Microfacet:
        {
            // Disney PBR 方案
            float cosalpha = dotProduct(N, wo);
            if (cosalpha > 0.0f)
            {
                float roughness = 0.40;
                Vector3f V = -wi;
                Vector3f L = wo;
                Vector3f H = normalize(V + L);
                // 计算 distribution of normals: D
                float D = DistributionGGX(N, H, roughness);
                // 计算 shadowing masking term: G
                float G = GeometrySmith(roughness, dotProduct(N, V), dotProduct(N, L));
                // 计算 fresnel 系数: F
                float F;
                float etat = 1.85;
                fresnel(wi, N, etat, F);
                Vector3f nominator = D * G * F;
                float denominator = 4 * cuda::max(dotProduct(N, V), 0.0f) * cuda::max(dotProduct(N, L), 0.0f);
                Vector3f specular = nominator / cuda::max(denominator, 0.001f);
                // 能量守恒
                float ks_ = F;          // 反射比率
                float kd_ = 1.0f - ks_; // 折射比率
                Vector3f diffuse = 1.0f / M_PI;
                // 因为在 specular 项里已经考虑了反射部分的比例：F。所以反射部分不需要再乘以 ks_
                // Ks为镜面反射项，Kd为漫反射项。
                return specular * Ks + diffuse * kd_ * Kd;
            }
            else
                return Vector3f(0.0f);
            break;
        }
        }
        return Vector3f(0.0f);
    }
};

#endif
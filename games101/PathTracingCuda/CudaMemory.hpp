#pragma once

#ifndef __OBJECT_HPP__
#define __OBJECT_HPP__

#include <cuda_runtime.h>

class CudaMemory
{
public:
    void *operator new(size_t len)
    {
        void *ptr;
        cudaMallocManaged(&ptr, len);
        cudaDeviceSynchronize();
        return ptr;
    }
    void operator delete(void *ptr)
    {
        cudaDeviceSynchronize();
        cudaFree(ptr);
    }
};

#endif
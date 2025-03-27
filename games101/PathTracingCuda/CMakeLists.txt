cmake_minimum_required(VERSION 3.10)
project(PathTracingCuda)

set(CMAKE_CXX_STANDARD 17)
set(CUDA_SEPARABLE_COMPILATION ON)
add_definitions(-D_USE_MATH_DEFINES)

find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})


cuda_add_executable(
    PathTracingCuda
    Bounds3.cuh BVH.cuh BVH.cu Vector.cuh 
    global.cuh cuda/Renderer.cu
    Intersection.cuh Light.cuh main.cu Material.cuh 
    OBJ_Loader.cuh Object.cuh Ray.cuh Renderer.cuh
    Scene.cuh Triangle.cuh 
)

target_link_libraries(${PROJECT_NAME})
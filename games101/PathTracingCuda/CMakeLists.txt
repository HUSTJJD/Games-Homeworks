cmake_minimum_required(VERSION 3.10)
project(PathTracingCuda)

set(CMAKE_CXX_STANDARD 17)
set(CUDA_SEPARABLE_COMPILATION ON)
add_definitions(-D_USE_MATH_DEFINES)

find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})


cuda_add_executable(
    PathTracingCuda
    Bounds3.hpp BVH.hpp BVH.cpp Vector.hpp 
    global.hpp cuda/Renderer.cu
    Intersection.hpp Light.hpp main.cpp Material.hpp 
    OBJ_Loader.hpp Object.hpp Ray.hpp Renderer.hpp
    Scene.hpp Triangle.hpp 
)

target_link_libraries(${PROJECT_NAME})
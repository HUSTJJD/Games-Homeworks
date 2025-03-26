cmake_minimum_required(VERSION 3.10)
project(BezierCurve)
set(CMAKE_CXX_STANDARD 14)

set(BUILD_SHARED_LIBS OFF)
add_compile_options("/MT")
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

add_executable(BezierCurve main.cpp)

target_link_libraries(BezierCurve ${OpenCV_LIBRARIES})

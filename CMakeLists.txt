cmake_minimum_required(VERSION 3.10)
project(OctreeProject)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Add source files
add_executable(OctreeProject
    src/main.cpp
    src/enviroment/enviroment.cpp
    src/enviroment/point/point.cpp
    src/enviroment/octree/Octree.cpp
    src/3d_model/PointCloud.cpp
)

# Include directories
target_include_directories(OctreeProject PRIVATE src/)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
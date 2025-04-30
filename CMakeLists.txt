cmake_minimum_required(VERSION 3.10)
project(DroneScanner)

find_package(PCL COMPONENTS common registration io kdtree features surface REQUIRED)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE Debug)  # Ensure Debug mode is used
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -Wextra -pedantic -fsanitize=address,undefined")

# Add source files
add_executable(DroneScanner
    src/main.cpp

    src/enviroment/enviroment.cpp

    src/enviroment/point/point.cpp
    src/enviroment/EnviromentBlock/octree/Octree.cpp
    src/enviroment/EnviromentBlock/enviroment_block.cpp
    src/enviroment/map_updating/map_update.cpp

    src/enviroment/pathfinding/a_star.cpp

    src/3d_model/alpha_shapes.cpp
    src/3d_model/greedy_projection_triangulation.cpp
    src/3d_model/poisson_reconstruction.cpp

    src/drone_communication/Communicator.cpp

)

# Include directories
target_include_directories(DroneScanner PRIVATE src/)
target_link_libraries(DroneScanner PRIVATE ${PCL_LIBRARIES})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
#include "../enviroment.h"
#include "../point/point.h"
// #include "EnviromentBlock/enviroment_block.h"
#include "../globals.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/common.h>


//TODO: Add point to multiple voxels if it is on the boundary
Point Enviroment::update_enviroment(Point origin, Point points[]) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr new_scan(new pcl::PointCloud<pcl::PointXYZ>());
    for (int i = 0; i < BATCH_SIZE; ++i) {
        new_scan->points.emplace_back(points[i].x, points[i].y, points[i].z);
    }

    // Compute bounding box for new scan plus buffer zone
    float min_x = std::numeric_limits<float>::max(), max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max(), max_y = std::numeric_limits<float>::lowest();
    float min_z = std::numeric_limits<float>::max(), max_z = std::numeric_limits<float>::lowest();

    const float buffer_distance = 50.0f;

    // Find the bounding box of the new scan
    for (const auto& point : new_scan->points) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
        min_z = std::min(min_z, point.z);
        max_z = std::max(max_z, point.z);
    }

    // Expand bounding box by the buffer zone
    min_x -= buffer_distance;
    max_x += buffer_distance;
    min_y -= buffer_distance;
    max_y += buffer_distance;
    min_z -= buffer_distance;
    max_z += buffer_distance;

    // Collect all relevant points from the environment
    pcl::PointCloud<pcl::PointXYZ>::Ptr world_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < depth; ++y) {
            for (int z = 0; z < height; ++z) {
                EnviromentBlock* block = get_block(x, y, z);
                if (block) {
                    // Check if this block intersects the expanded bounding box
                    Point block_center = block->get_center();
                    if (block_center.x + DRONE_WIDTH / 2 >= min_x &&
                        block_center.x - DRONE_WIDTH / 2 <= max_x &&
                        block_center.y + DRONE_HEIGHT / 2 >= min_y &&
                        block_center.y - DRONE_HEIGHT / 2 <= max_y &&
                        block_center.z + DRONE_DEPTH / 2 >= min_z &&
                        block_center.z - DRONE_DEPTH / 2 <= max_z) {
                        // Add all points from this block into the world cloud
                        for (const auto& point : block->get_points()) {
                            world_cloud->points.emplace_back(point.x, point.y, point.z);
                        }
                    }
                }
            }
        }
    }

    Point new_origin = origin;
    pcl::PointCloud<pcl::PointXYZ> aligned_scan;
    if (!world_cloud->empty()) {
        // Perform ICP alignment
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(new_scan);
        icp.setInputTarget(world_cloud);

        icp.align(aligned_scan);

        if (!icp.hasConverged()) {
            std::cerr << "ICP alignment failed!" << std::endl;
            return origin;
        }

        Eigen::Matrix4f transformation = icp.getFinalTransformation();
        new_origin = Point(
            origin.x + transformation(0, 3),
            origin.y + transformation(1, 3),
            origin.z + transformation(2, 3)
        );

    } else {
        aligned_scan = *new_scan;
    }

    int origin_x = origin.x / DRONE_WIDTH;
    int origin_y = origin.y / DRONE_DEPTH;
    int origin_z = origin.z / DRONE_HEIGHT;

    // Integrate the aligned scan into the environment's voxel grid
    for (const auto& point : aligned_scan.points) {

        int x = point.x / DRONE_WIDTH;
        int y = point.y / DRONE_DEPTH;
        int z = point.z / DRONE_HEIGHT;

        
        EnviromentBlock* block = get_block(x, y, z);
        if (block) {

            // Add air confidence to all voxels along the path
            std::vector<std::array<int, 3>> path = bresenham_3d({origin_x, origin_y, origin_z}, {x, y, z});
            for (std::array<int, 3> p : path) {
                int x_ = p[0];
                int y_ = p[1];
                int z_ = p[2];
                EnviromentBlock* air_block = get_block(x_, y_, z_);
                air_block->increment_air_count();
            }
            
            block->add_point(Point(point.x, point.y, point.z));
        }
    }

    return new_origin;

};
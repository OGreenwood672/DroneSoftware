#include "../enviroment/enviroment.h"
#include "../enviroment/EnviromentBlock/enviroment_block.h"

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/obj_io.h>

void Enviroment::greedy_projection_triangulation(const std::string& filename) const {
    // Create point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : get_points()) {
        cloud->points.emplace_back(point.x, point.y, point.z);
    }

    if (cloud->points.empty()) {
        std::cerr << "No points in environment! Cannot generate .obj file." << std::endl;
        return;
    }

    // Update point cloud width and height
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    // Estimate normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    
    normal_estimation.setInputCloud(cloud);
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setRadiusSearch(0.03);  // Reduced radius for better local feature estimation
    normal_estimation.compute(*normals);

    // Combine points and normals
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    
    // Manual concatenation of points and normals
    cloud_with_normals->points.resize(cloud->points.size());
    cloud_with_normals->width = cloud->width;
    cloud_with_normals->height = cloud->height;
    cloud_with_normals->is_dense = cloud->is_dense;

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud_with_normals->points[i].x = cloud->points[i].x;
        cloud_with_normals->points[i].y = cloud->points[i].y;
        cloud_with_normals->points[i].z = cloud->points[i].z;
        cloud_with_normals->points[i].normal_x = normals->points[i].normal_x;
        cloud_with_normals->points[i].normal_y = normals->points[i].normal_y;
        cloud_with_normals->points[i].normal_z = normals->points[i].normal_z;
        cloud_with_normals->points[i].curvature = normals->points[i].curvature;
    }

    // Create search tree for combined cloud
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    // Initialize triangulation object
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh mesh;

    // Set triangulation parameters
    gp3.setSearchRadius(0.025);              // Set smaller search radius
    gp3.setMu(2.5);                          // Set typical value for mu
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI/4);      // 45 degrees
    gp3.setMinimumAngle(M_PI/18);            // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3);           // 120 degrees
    gp3.setNormalConsistency(true);          // Enforce normal consistency

    // Set input for triangulation
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);

    try {
        // Perform reconstruction
        gp3.reconstruct(mesh);

        // Save the mesh
        if (pcl::io::saveOBJFile(filename, mesh) == 0) {
            std::cout << "Successfully exported environment to " << filename << std::endl;
        } else {
            std::cerr << "Failed to save mesh to " << filename << std::endl;
        }
    } catch (const pcl::PCLException& e) {
        std::cerr << "Error during mesh reconstruction: " << e.what() << std::endl;
    }
}
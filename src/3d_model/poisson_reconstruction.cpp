#include "../enviroment/enviroment.h"
#include "../enviroment/EnviromentBlock/enviroment_block.h"
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/obj_io.h>

void Enviroment::poisson_reconstruction(const std::string& filename) const {
    // Create point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // Reserve space for efficiency
    auto points = get_points();
    cloud->points.reserve(points.size());
    
    for (const auto& point : points) {
        cloud->points.emplace_back(point.x, point.y, point.z);
    }

    if (cloud->points.empty()) {
        std::cerr << "No points in environment! Cannot generate .obj file." << std::endl;
        return;
    }

    // Set basic cloud parameters
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    // Compute normals with optimized parameters
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    
    normal_estimation.setInputCloud(cloud);
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setKSearch(15);  // Balanced value for normal estimation
    normal_estimation.compute(*normals);

    // Combine points and normals
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    cloud_with_normals->points.resize(cloud->points.size());
    cloud_with_normals->width = cloud->width;
    cloud_with_normals->height = cloud->height;
    cloud_with_normals->is_dense = true;

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud_with_normals->points[i].x = cloud->points[i].x;
        cloud_with_normals->points[i].y = cloud->points[i].y;
        cloud_with_normals->points[i].z = cloud->points[i].z;
        cloud_with_normals->points[i].normal_x = normals->points[i].normal_x;
        cloud_with_normals->points[i].normal_y = normals->points[i].normal_y;
        cloud_with_normals->points[i].normal_z = normals->points[i].normal_z;
    }

    try {
        // Create Poisson surface reconstruction object with conservative parameters
        pcl::Poisson<pcl::PointNormal> poisson;
        
        // Set conservative parameters
        poisson.setDepth(7);           // Moderate depth for stability
        poisson.setSolverDivide(7);    // Matched with depth
        poisson.setIsoDivide(7);       // Matched with depth
        poisson.setPointWeight(2.0);    // Reduced weight
        poisson.setScale(1.1);         // Slight scale adjustment
        poisson.setSamplesPerNode(1.5); // Moderate sampling
        
        // Set input
        poisson.setInputCloud(cloud_with_normals);
        
        // Perform reconstruction
        pcl::PolygonMesh mesh;
        poisson.reconstruct(mesh);

        // Save the mesh
        if (pcl::io::saveOBJFile(filename, mesh) == 0) {
            std::cout << "Successfully exported environment to " << filename << std::endl;
            std::cout << "Number of vertices: " << mesh.cloud.width * mesh.cloud.height << std::endl;
            std::cout << "Number of faces: " << mesh.polygons.size() << std::endl;
        } else {
            std::cerr << "Failed to save mesh to " << filename << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error during mesh reconstruction: " << e.what() << std::endl;
    }
}
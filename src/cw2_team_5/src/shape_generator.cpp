// File: shape_generator.cpp
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>


#include "shape_generator.h"
// Generate cross cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr generateCrossShapePointCloud(const float cell_size) {
    // Instanciate cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Define params
    const int grid_size = 5;  // 5x5 grid
    const float height = 0.04;  // obj height
    const float point_density = 0.002;  // Matches the Voxel grid
    const int num_points_per_cell = cell_size / point_density; // num_points_per_cell * num_points_per_cell points in each grid cell

    // Generate pointcloud
    for (int i = 0; i < grid_size; ++i) {
        for (int j = 0; j < grid_size; ++j) {
            // find grids of cross shape
            if (i == 2 || j == 2) {
                float x_start = i * cell_size;
                float y_start = j * cell_size;

                // generate points
                for (int xi = 0; xi < num_points_per_cell; ++xi) {
                    for (int yi = 0; yi < num_points_per_cell; ++yi) {
                        for (int zi = 0; zi < 5; ++zi) { // height direction point??
                            float x = x_start + (xi / static_cast<float>(num_points_per_cell)) * cell_size;
                            float y = y_start + (yi / static_cast<float>(num_points_per_cell)) * cell_size;
                            float z = (zi / 5.0f) * height;
                            cloud->points.emplace_back(x, y, z);
                        }
                    }
                }
            }
        }
    }

    // set cloud params
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1; 
    cloud->is_dense = true;

    return cloud;
}




pcl::PointCloud<pcl::PointXYZ>::Ptr generateOughtShapePointCloud(const float cell_size) {
    // Instanciate cloud pointer
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Define parameters
    const int grid_size = 5;  // 5x5 grid
    const float height = 0.04;  // object height
    const float point_density = 0.002;  // Matches the Voxel grid
    const int num_points_per_cell = cell_size / point_density; // num_points_per_cell * num_points_per_cell points in each grid cell

    // 生成点云
    for (int i = 0; i < grid_size; ++i) {
        for (int j = 0; j < grid_size; ++j) {
            // If grid cell satisfies nought shape
            if (i == 0 || j == 0 || i == 4 || j == 4) {
                float x_start = i * cell_size;
                float y_start = j * cell_size;

                // Generate points
                for (int xi = 0; xi < num_points_per_cell; ++xi) {
                    for (int yi = 0; yi < num_points_per_cell; ++yi) {
                        for (int zi = 0; zi < 5; ++zi) { // verticle points?
                            float x = x_start + (xi / static_cast<float>(num_points_per_cell)) * cell_size;
                            float y = y_start + (yi / static_cast<float>(num_points_per_cell)) * cell_size;
                            float z = (zi / 5.0f) * height;
                            cloud->points.emplace_back(x, y, z);
                        }
                    }
                }
            }
        }
    }

    // change cloud parameters
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1; 
    cloud->is_dense = true;

    return cloud;
}


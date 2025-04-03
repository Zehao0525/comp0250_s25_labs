// File: shape_generator.cpp
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>


#include "shape_generator.h"
// 生成十字形点云的函数
pcl::PointCloud<pcl::PointXYZ>::Ptr generateCrossShapePointCloud(const float cell_size) {
    // 定义点云指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 参数定义
    const int grid_size = 5;  // 5x5 格子
    // const float cell_size = 40.0f; // 每个格子40mm
    const float height = 0.04;  // 高度40mm
    const int num_points_per_cell = 10; // 每个格子的点密度（越大越密集）

    // 生成点云
    for (int i = 0; i < grid_size; ++i) {
        for (int j = 0; j < grid_size; ++j) {
            // 满足十字架形状（第3行或第3列）
            if (i == 2 || j == 2) {
                float x_start = i * cell_size;
                float y_start = j * cell_size;

                // 生成该格子内的点
                for (int xi = 0; xi < num_points_per_cell; ++xi) {
                    for (int yi = 0; yi < num_points_per_cell; ++yi) {
                        for (int zi = 0; zi < 5; ++zi) { // 高度方向的点
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

    // 设置点云参数
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;  // 无序点云
    cloud->is_dense = true;

    return cloud;
}




pcl::PointCloud<pcl::PointXYZ>::Ptr generateOughtShapePointCloud(const float cell_size) {
    // 定义点云指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 参数定义
    const int grid_size = 5;  // 5x5 格子
    // const float cell_size = 40.0f; // 每个格子40mm
    const float height = 0.04;  // 高度40mm
    const int num_points_per_cell = 10; // 每个格子的点密度（越大越密集）

    // 生成点云
    for (int i = 0; i < grid_size; ++i) {
        for (int j = 0; j < grid_size; ++j) {
            // 满足十字架形状（第3行或第3列）
            if (i == 0 || j == 0 || i == 4 || j == 4) {
                float x_start = i * cell_size;
                float y_start = j * cell_size;

                // 生成该格子内的点
                for (int xi = 0; xi < num_points_per_cell; ++xi) {
                    for (int yi = 0; yi < num_points_per_cell; ++yi) {
                        for (int zi = 0; zi < 5; ++zi) { // 高度方向的点
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

    // 设置点云参数
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;  // 无序点云
    cloud->is_dense = true;

    return cloud;
}


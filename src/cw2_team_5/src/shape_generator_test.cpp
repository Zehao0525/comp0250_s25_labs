#include <pcl/io/pcd_io.h>
#include <iostream>
#include "shape_generator.h"

int main() {
    // 生成十字架点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cross_cloud = generateCrossShapePointCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generateOughtShapePointCloud(); 

    // 保存到 PCD 文件
    pcl::io::savePCDFileASCII("cross_shape.pcd", *cross_cloud);
    pcl::io::savePCDFileASCII("ought_shape.pcd", *cloud);
    std::cout << "Saved " << cloud->points.size() << " points to cross_shape.pcd" << std::endl;


    return 0;
}
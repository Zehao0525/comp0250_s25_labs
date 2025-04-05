#include <pcl/io/pcd_io.h>
#include <iostream>
#include "shape_generator.h"

int main() {
    // gen cross
    pcl::PointCloud<pcl::PointXYZ>::Ptr cross_cloud = generateCrossShapePointCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generateOughtShapePointCloud(); 

    // save to pcd
    pcl::io::savePCDFileASCII("cross_shape.pcd", *cross_cloud);
    pcl::io::savePCDFileASCII("ought_shape.pcd", *cloud);
    std::cout << "Saved " << cloud->points.size() << " points to cross_shape.pcd" << std::endl;


    return 0;
}
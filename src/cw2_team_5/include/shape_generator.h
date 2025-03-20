#ifndef SHAPE_GENERATOR_H_
#define SHAPE_GENERATOR_H_

pcl::PointCloud<pcl::PointXYZ>::Ptr generateCrossShapePointCloud(const float cell_size = 0.04);
pcl::PointCloud<pcl::PointXYZ>::Ptr generateOughtShapePointCloud(const float cell_size = 0.04);


#endif // end of include guard for SHAPE_GENERATOR_H_
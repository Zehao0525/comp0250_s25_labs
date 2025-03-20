#ifndef DETECT_OBJECT_H_
#define DETECT_OBJECT_H_


#include "cw2_class.h"

void detect_objects(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& in_cloud_ptr, DetectedObject detected_objects, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& obj_cloud_ptr) ;
void detect_objects(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& in_cloud_ptr, std::vector<DetectedObject>& detected_objects);
void detect_objects(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& in_cloud_ptr, DetectedObject detected_object);

void subtractPointCloud(
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_main,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_to_remove,
    double distance_threshold);


static bool enforce_color_similarity(const pcl::PointXYZRGBA& a, const pcl::PointXYZRGBA& b, float /*squared_dist*/) ;


#endif // end of include guard for cw2_CLASS_H_
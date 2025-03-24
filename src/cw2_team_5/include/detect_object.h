#ifndef DETECT_OBJECT_H_
#define DETECT_OBJECT_H_


#include "cw2_class.h"
#include "data_structure.h"

// void detect_objects(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& in_cloud_ptr, DetectedObject detected_objects, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& obj_cloud_ptr) ;
void detect_objects(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& in_cloud_ptr, std::vector<ShapeDetectionResult>& detected_objects);
// void detect_objects(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& in_cloud_ptr, DetectedObject detected_object);

void subtractPointCloud(
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_main,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_to_remove,
    double distance_threshold);


static bool enforce_color_similarity(const pcl::PointXYZRGBA& a, const pcl::PointXYZRGBA& b, float /*squared_dist*/) ;

ShapeDetectionResult detectShapeRotation(
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& obj_cloud_ptr, 
    float angle_step = 0.5) ;

float calculateOverlap(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);
void translatePointCloudToOrigin(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void translatePointCloudToOrigin(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

bool checkBasket(
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster,
    const Eigen::Vector3f& dims,
    const geometry_msgs::Point& object_point,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr working_cloud,
    std::vector<ShapeDetectionResult>& detected_objects);


////////////////////
#include <thread>
#include <mutex>
// multi-processes
void processShapeCombination(
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& obj_cloud_ptr,
    const std::string& shape_type,
    float size,
    float angle_step,
    ShapeDetectionResult& global_result);

ShapeDetectionResult detectShapeRotation_multi(
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& obj_cloud_ptr,
    float angle_step = 0.5);
#endif // end of include guard for cw2_CLASS_H_
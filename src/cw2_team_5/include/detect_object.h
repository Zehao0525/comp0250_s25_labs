#ifndef DETECT_OBJECT_H_
#define DETECT_OBJECT_H_


#include "cw2_class.h"
#include "data_structure.h"

void detectObjects(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& in_cloud_ptr, std::vector<ShapeDetectionResult>& detected_objects, 
    std::vector<Obstacle>& obstacles);


void subtractPointCloud(
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_main,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_to_remove,
    double distance_threshold);


static bool enforceColorSimilarity(const pcl::PointXYZRGBA& a, const pcl::PointXYZRGBA& b, float /*squared_dist*/) ;

float calculateOverlap(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);
float calculateOverlap_backup(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);

void translatePointCloudToOrigin(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void translatePointCloudToOrigin(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

bool checkBasket(
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster,
    const Eigen::Vector3f& dims,
    const geometry_msgs::Point& object_point,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr working_cloud,
    std::vector<ShapeDetectionResult>& detected_objects);

    void clusterPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud, 
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>& valid_clusters,
        std::vector<Eigen::Vector4f>& cluster_centroids);
        bool isValidObjectCluster(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cluster, 
            const Eigen::Vector3f& dims, 
            std::vector<Obstacle>& obstacles);

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

void processShapeCombination_backup(
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& obj_cloud_ptr,
    const std::string& shape_type,
    float size,
    float angle_step,
    ShapeDetectionResult& global_result);


ShapeDetectionResult detectShapeRotationMulti(
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& obj_cloud_ptr,
    float angle_step = 2);
#endif // end of include guard for cw2_CLASS_H_
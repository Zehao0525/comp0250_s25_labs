#ifndef DATA_STRUCTURE_H_
#define DATA_STRUCTURE_H_

#include <geometry_msgs/Pose.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
#include <shape_msgs/SolidPrimitive.h>

class Init_Pose : public geometry_msgs::Pose {
public:
    Init_Pose() {

    orientation.x = 0.9239;
    orientation.y = -0.3827;
    orientation.z = 0.0;
    orientation.w = 0.0;

    position.x = 0.0;
    position.y = 0.0;
    position.z = 0.0;

  }

  // void operator=(const geometry_msgs::Pose& pose) {
  //   orientation = pose.orientation;
  //   position = pose.position;
  // }
  
  Init_Pose& operator=(const geometry_msgs::Pose& pose) {
    this->position = pose.position;
    this->orientation = pose.orientation;
    return *this;
  }
};


struct DetectedObject
{
  std::string type, color, name;
  geometry_msgs::Point position;
  float w,l,h,r,g,b;

  DetectedObject() {
    position.x = 0.0;
    position.y = 0.0;
    position.z = 0.0;
  }
};


struct ShapeDetectionResult {
  float rotation_angle;
  std::string shape_type;
  float size;
  float overlap_score;
  Eigen::Matrix<float, 4, 1> centroid;
  uint8_t r, g, b;

  ShapeDetectionResult() {
    rotation_angle = -1.0;
    shape_type = "unknown";
    size = 0.0;
    overlap_score = 0.0;
    centroid << 0.0, 0.0, 0.0, 0.0;
    r, g, b = 0;
  }


};


struct Obstacle {
  shape_msgs::SolidPrimitive obstacle;
  geometry_msgs::Pose obstacle_pose;

};

#endif // end of include guard for DATA_STRUCTURE_H_
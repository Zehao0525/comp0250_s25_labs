#ifndef DATA_STRUCTURE_H_
#define DATA_STRUCTURE_H_

#include <geometry_msgs/Pose.h>

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

};


struct DetectedObject
{
  std::string type, color, name;
  geometry_msgs::Point position;
  float w,l,h,r,g,b;
};

#endif // end of include guard for DATA_STRUCTURE_H_
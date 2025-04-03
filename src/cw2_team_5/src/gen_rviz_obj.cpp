#include "cw2_class.h"

void cw2::genRect(const std::string& name, const float x, const float y, const float dim_x, const float dim_y, const float theta)
{
  // Define basket location
  geometry_msgs::Vector3 rect_dim;
  rect_dim.x = dim_x;
  rect_dim.y = dim_y;
  rect_dim.z = 0.04;

  geometry_msgs::Point rect_pos;
  rect_pos.x = x;
  rect_pos.y = y;
  rect_pos.z = 0.04;

  geometry_msgs::Quaternion rect_ori;
  rect_ori.x = 0.0;
  rect_ori.y = 0.0;
  rect_ori.z = sin(theta / 2.0);
  rect_ori.w = cos(theta / 2.0);

  addCollisionObject(name, rect_pos, rect_dim, rect_ori);
}

std::vector<std::string> cw2::genNoughtObj(const float cell_size, const float x, const float y, const float theta){
    
    std::vector<std::string> obj_names;
    float displacement = cell_size * 2;
    float dim1 = cell_size * 5;
    float dim2 = cell_size;
    float theta_rad = theta / 180 * M_PI;

    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO_STREAM("Cell size: " << cell_size);
    ROS_INFO_STREAM("x,y: " << x << y);
    ROS_INFO_STREAM("theta: " << theta);

    // left Side
    std::string id1 = "obj_nought_1";
    obj_names.push_back(id1);
    genRect(id1, x + displacement*cos(theta_rad), y + displacement*sin(theta_rad), dim2, dim1, theta_rad);

    // Up Side
    std::string id2 = "obj_nought_2";
    obj_names.push_back(id2);
    genRect(id2, x + displacement*sin(theta_rad), y - displacement*cos(theta_rad), dim1, dim2, theta_rad);

    // Right Side
    std::string id3 = "obj_nought_3";
    obj_names.push_back(id3);
    genRect(id3, x - displacement*cos(theta_rad), y - displacement*sin(theta_rad), dim2, dim1, theta_rad);

    // Down Side
    std::string id4 = "obj_nought_4";
    obj_names.push_back(id4);
    genRect(id4, x - displacement*sin(theta_rad), y + displacement*cos(theta_rad), dim1, dim2, theta_rad);

    return obj_names;
}


std::vector<std::string> cw2::genCrossObj(const float cell_size, const float x, const float y, const float theta){
    
    std::vector<std::string> obj_names;
    float displacement = cell_size * 2;
    float dim1 = cell_size * 5;
    float dim2 = cell_size;
    float theta_rad = theta / 180 * M_PI;

    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO("Cell size:");
    ROS_INFO_STREAM("Cell size: " << cell_size);
    ROS_INFO_STREAM("x,y: " << x << y);
    ROS_INFO_STREAM("theta: " << theta);

    // left Side
    std::string id1 = "obj_cross_1";
    obj_names.push_back(id1);
    genRect(id1, x, y, dim1, dim2, theta_rad);

    // Up Side
    std::string id2 = "obj_cross_2";
    obj_names.push_back(id2);
    genRect(id2, x, y, dim2, dim1, theta_rad);

    return obj_names;
}

void cw2::removeCollisionObjects(std::vector<std::string> obj_names)
{
    for (const std::string& name : obj_names) {
        removeCollisionObject(name);
    }
}


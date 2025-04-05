#include "cw2_class.h"

///////////////////////////////////////////////////////////////////////////////

void cw2::addPlane(const std::string& object_name)
{
  // 0.02 thick, 0.1 on both sides for a tile
  // x center from 0.25 to 0.65 (0.2-0.7)
  // y center from -0.4 to 0.4 (-0.45-0.45)
  // z centres at 0.01
  geometry_msgs::Point plane_pos;
  plane_pos.x = 0.05;
  plane_pos.y = 0;
  plane_pos.z = 0.01;

  geometry_msgs::Vector3 plane_dim;
  plane_dim.x = 1.32;
  plane_dim.y = 0.92;
  plane_dim.z = 0.02;

  geometry_msgs::Quaternion plate_ori;
  plate_ori.w = 1.0;
  plate_ori.x = 0.0;
  plate_ori.y = 0.0;
  plate_ori.z = 0.0;

  addCollisionObject(object_name, plane_pos, plane_dim, plate_ori);
}


void cw2::addBasket(const std::string& name, const geometry_msgs::Point& basket_pos)
{
  // Define basket location
  geometry_msgs::Vector3 basket_dim;
  basket_dim.x = 0.37;
  basket_dim.y = 0.37;
  basket_dim.z = 0.05;

  geometry_msgs::Point basket_pos_1 = basket_pos;
  basket_pos_1.z = basket_pos.z;

  geometry_msgs::Quaternion basket_ori;
  basket_ori.w = 1.0;
  basket_ori.x = 0.0;
  basket_ori.y = 0.0;
  basket_ori.z = 0.0;

  addCollisionObject(name, basket_pos_1, basket_dim, basket_ori);
}


void cw2::removeAllCollisions()
{
  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;

  collision_object.operation = collision_object.REMOVE;
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);
}

void cw2::addCollisionObject(const std::string& object_name,
                            const geometry_msgs::Point& centre, 
                            const geometry_msgs::Vector3& dimensions,
                            const geometry_msgs::Quaternion& orientation)
{
  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;
  
  // input header information
  collision_object.id = object_name;
  collision_object.header.frame_id = base_frame_;

  // define the primitive and its dimensions
  collision_object.primitives.resize(1);
  collision_object.primitives[0].type = collision_object.primitives[0].BOX;
  collision_object.primitives[0].dimensions.resize(3);
  collision_object.primitives[0].dimensions[0] = dimensions.x;
  collision_object.primitives[0].dimensions[1] = dimensions.y;
  collision_object.primitives[0].dimensions[2] = dimensions.z;

  // define the pose of the collision object
  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0].position.x = centre.x;
  collision_object.primitive_poses[0].position.y = centre.y;
  collision_object.primitive_poses[0].position.z = centre.z;
  collision_object.primitive_poses[0].orientation = orientation;

  // define that we will be adding this collision object 
  collision_object.operation = collision_object.ADD;

  // add the collision object to the vector, then apply to planning scene
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);
}

void cw2::removeCollisionObject(const std::string& object_name)
{
  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;
  
  // input header information
  collision_object.id = object_name;
  collision_object.header.frame_id = base_frame_;

  // define that we will be removing this collision object 
  collision_object.operation = collision_object.REMOVE;

  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);
}

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

std::vector<std::string> cw2::genNoughtObj(const float cell_size, const float x, const float y, const float theta, const std::string prefix){
    
    std::vector<std::string> obj_names;
    float displacement = cell_size * 2;
    float dim1 = cell_size * 5;
    float dim2 = cell_size;
    float theta_rad = theta / 180 * M_PI;


    // left Side
    std::string id1 = prefix+"obj_nought_1";
    obj_names.push_back(id1);
    genRect(id1, x + displacement*cos(theta_rad), y + displacement*sin(theta_rad), dim2, dim1, theta_rad);

    // Up Side
    std::string id2 = prefix+"obj_nought_2";
    obj_names.push_back(id2);
    genRect(id2, x + displacement*sin(theta_rad), y - displacement*cos(theta_rad), dim1, dim2, theta_rad);

    // Right Side
    std::string id3 = prefix+"obj_nought_3";
    obj_names.push_back(id3);
    genRect(id3, x - displacement*cos(theta_rad), y - displacement*sin(theta_rad), dim2, dim1, theta_rad);

    // Down Side
    std::string id4 = prefix+"obj_nought_4";
    obj_names.push_back(id4);
    genRect(id4, x - displacement*sin(theta_rad), y + displacement*cos(theta_rad), dim1, dim2, theta_rad);

    return obj_names;
}


std::vector<std::string> cw2::genCrossObj(const float cell_size, const float x, const float y, const float theta, const std::string prefix){
    
    std::vector<std::string> obj_names;
    float displacement = cell_size * 2;
    float dim1 = cell_size * 5;
    float dim2 = cell_size;
    float theta_rad = theta / 180 * M_PI;


    // left Side
    std::string id1 = prefix+"obj_cross_1";
    obj_names.push_back(id1);
    genRect(id1, x, y, dim1, dim2, theta_rad);

    // Up Side
    std::string id2 = prefix+"obj_cross_2";
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


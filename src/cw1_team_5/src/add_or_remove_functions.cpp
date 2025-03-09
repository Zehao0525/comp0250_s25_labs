#include "cw1_class.h"


// #include "cw1_tools.cpp"

///////////////////////////////////////////////////////////////////////////////

void cw1::add_plane(const std::string& object_name)
{
  // 0.02 thick, 0.1 on both sides for a tile
  // x center from 0.25 to 0.65 (0.2-0.7)
  // y center from -0.4 to 0.4 (-0.45-0.45)
  // z centres at 0.01
  geometry_msgs::Point plane_pos;
  plane_pos.x = 0.45;
  plane_pos.y = 0;
  plane_pos.z = 0.01;

  geometry_msgs::Vector3 plane_dim;
  plane_dim.x = 0.52;
  plane_dim.y = 0.92;
  plane_dim.z = 0.04;

  geometry_msgs::Quaternion plate_ori;
  plate_ori.w = 1.0;
  plate_ori.x = 0.0;
  plate_ori.y = 0.0;
  plate_ori.z = 0.0;

  add_collision_object(object_name, plane_pos, plane_dim, plate_ori);
}

void cw1::add_clarance(const std::string& object_name)
{
  geometry_msgs::Point plane_pos;
  plane_pos.x = 0.45;
  plane_pos.y = 0.0;
  plane_pos.z = 0.06;

  geometry_msgs::Vector3 plane_dim;
  plane_dim.x = 0.52;
  plane_dim.y = 0.92;
  plane_dim.z = 0.12;

  geometry_msgs::Quaternion plate_ori;
  plate_ori.w = 1.0;
  plate_ori.x = 0.0;
  plate_ori.y = 0.0;
  plate_ori.z = 0.0;

  add_collision_object(object_name, plane_pos, plane_dim, plate_ori);
}

void cw1::add_basket(const std::string& name, const geometry_msgs::Point& basket_pos)
{
  // Define basket location
  geometry_msgs::Vector3 basket_dim;
  basket_dim.x = 0.12;
  basket_dim.y = 0.12;
  basket_dim.z = 0.12;

  geometry_msgs::Quaternion basket_ori;
  basket_ori.w = 1.0;
  basket_ori.x = 0.0;
  basket_ori.y = 0.0;
  basket_ori.z = 0.0;

  add_collision_object(name, basket_pos, basket_dim, basket_ori);
}

void cw1::add_baskets(const std::string& name_prefix, 
                    const std::vector<geometry_msgs::Point>& basket_locs)
{
  // If nothing to add, return
  if (basket_locs.empty()) {
    return;
  }

  // Define basket location
  geometry_msgs::Vector3 basket_dim;
  basket_dim.x = 0.12;
  basket_dim.y = 0.12;
  basket_dim.z = 0.12;

  geometry_msgs::Quaternion basket_ori;
  basket_ori.w = 1.0;
  basket_ori.x = 0.0;
  basket_ori.y = 0.0;
  basket_ori.z = 0.0;

  for (size_t i = 0; i < basket_locs.size(); i++)
  {
    std::string basket_name = name_prefix + std::to_string(i + 1);
    add_collision_object(basket_name, basket_locs[i], basket_dim, basket_ori);
  }
}

void cw1::add_baskets(const std::string& name_prefix,
                    const std::vector<geometry_msgs::PointStamped>& basket_locs)
{
  // If nothing to add, return
  if (basket_locs.empty()) {
    return;
  }

  std::vector<geometry_msgs::Point> basket_locs_pts;
  basket_locs_pts.reserve(basket_locs.size());

  for (size_t i = 0; i < basket_locs.size(); i++)
  {
    basket_locs_pts.push_back(basket_locs[i].point);
  }
  add_baskets(name_prefix, basket_locs_pts);
}

void cw1::add_cube(const std::string& name, const geometry_msgs::Point& cube_loc)
{
  geometry_msgs::Vector3 cube_dimen;
  geometry_msgs::Quaternion cube_orien;

  cube_dimen.x = 0.06;
  cube_dimen.y = 0.06;
  cube_dimen.z = 0.06;

  cube_orien.w = 1.0;
  cube_orien.x = 0.0;
  cube_orien.y = 0.0;
  cube_orien.z = 0.0;

  add_collision_object(name, cube_loc, cube_dimen, cube_orien);
}

void cw1::add_cubes(const std::string& name_prefix, 
                  const std::vector<geometry_msgs::Point>& cube_locs)
{
  // If nothing to add, return
  if (cube_locs.empty()) {
    return;
  }

  geometry_msgs::Vector3 cube_dimen;
  geometry_msgs::Quaternion cube_orien;

  cube_dimen.x = 0.06;
  cube_dimen.y = 0.06;
  cube_dimen.z = 0.06;

  cube_orien.w = 1.0;
  cube_orien.x = 0.0;
  cube_orien.y = 0.0;
  cube_orien.z = 0.0;

  for (size_t i = 0; i < cube_locs.size(); i++)
  {
    std::string cube_name = name_prefix + std::to_string(i + 1);
    add_collision_object(cube_name, cube_locs[i], cube_dimen, cube_orien);
  }
}

void cw1::add_cubes(const std::string& name_prefix,
                  const std::vector<geometry_msgs::PointStamped>& cube_locs)
{
  // If nothing to add, return
  if (cube_locs.empty()) {
    return;
  }

  // Convert Point stamped vector to point vectors
  std::vector<geometry_msgs::Point> cube_locs_pts;
  cube_locs_pts.reserve(cube_locs.size());
  
  for (size_t i = 0; i < cube_locs.size(); i++)
  {
    cube_locs_pts.push_back(cube_locs[i].point);
  }
  add_cubes(name_prefix, cube_locs_pts);
}

void cw1::remove_all_collisions()
{
  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;

  collision_object.operation = collision_object.REMOVE;
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);
}

void cw1::add_collision_object(const std::string& object_name,
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

void cw1::remove_collision_object(const std::string& object_name)
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
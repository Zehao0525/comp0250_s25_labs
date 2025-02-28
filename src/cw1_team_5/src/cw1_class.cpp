/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

#include <cw1_class.h>

///////////////////////////////////////////////////////////////////////////////

cw1::cw1(ros::NodeHandle nh):
  g_cloud_ptr (new PointC), // input point cloud
  g_cloud_filtered (new PointC), // filtered point cloud
  g_cloud_filtered2 (new PointC), // filtered point cloud
  g_tree_ptr (new pcl::search::KdTree<PointT> ()), // KdTree
  debug_ (false)
{  
  /* class constructor */

  nh_ = nh;

  // advertise solutions for coursework tasks
  t1_service_  = nh_.advertiseService("/task1_start", 
    &cw1::t1_callback, this);
  t2_service_  = nh_.advertiseService("/task2_start", 
    &cw1::t2_callback, this);
  t3_service_  = nh_.advertiseService("/task3_start",
    &cw1::t3_callback, this);

  // Define public variables
  g_vg_leaf_sz = 0.01; // VoxelGrid leaf size: Better in a config file
  g_k_nn = 50; // Normals nn size: Better in a config file

  ROS_INFO("cw1 class initialised");
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t1_callback(cw1_world_spawner::Task1Service::Request &request,
  cw1_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */

  ROS_INFO("The coursework solving callback for task 1 has been triggered");
  // TODO: use the addCollisionObject function
  // Set the orientation and adjust translation

  // To provide leeway, we will inflate all objects by 0.01 in each diection (that matters)

  // create cup object
  // add the collision in the RViz
  // We examine from model.sdf.xmacro that the tiles span from:
  // 0.2 to 0.7 in the x direction, and -0.45 to 0.45 in the y direction
  // Thickness in the z direction is 0.01
  geometry_msgs::Point plane_pos;
  plane_pos.x = 0.45;
  plane_pos.y = 0;
  plane_pos.z = 0.005;

  geometry_msgs::Vector3 plane_dim;
  plane_dim.x = 5;
  plane_dim.y = 9;
  plane_dim.z = 0.03;

  geometry_msgs::Quaternion plate_ori;
  plate_ori.w = 1;
  plate_ori.x = 0;
  plate_ori.y = 0;
  plate_ori.z = 0;

  addCollisionObject(plane_name,plane_pos,plane_dim,plate_ori);
  

  geometry_msgs::Vector3 cube_dim;
  cube_dim.x = 0.06;
  cube_dim.y = 0.06;
  cube_dim.z = 0.06;

  geometry_msgs::Quaternion cube_ori;
  cube_ori.w = 1;
  cube_ori.x = 0;
  cube_ori.y = 0;
  cube_ori.z = 0;

  // add the cube
  addCollisionObject(cube_name, request.object_loc.pose.position,cube_dim,cube_ori);

  // Define basket location
  geometry_msgs::Vector3 basket_dim;
  basket_dim.x = 0.12;
  basket_dim.y = 0.12;
  basket_dim.z = 0.12;

  geometry_msgs::Quaternion basket_ori;
  basket_ori.w = 1;
  basket_ori.x = 0;
  basket_ori.y = 0;
  basket_ori.z = 0;

  // add the basket
  addCollisionObject(basket_name, request.goal_loc.point, basket_dim, basket_ori);

  set_constraint ();

  // We specify waypoints for the arm trajectory
  geometry_msgs::Pose init_pos, target_pos, lift_pos, goal_pos;

  // We specify the location the arm need to reach to pick up the object
  target_pos.orientation.x = 0.9239;
  target_pos.orientation.y = -0.3827;
  target_pos.orientation.z = 0.0;
  target_pos.orientation.w = 0.0;
  target_pos.position.x = request.object_loc.pose.position.x;
  target_pos.position.y = request.object_loc.pose.position.y;
  target_pos.position.z = request.object_loc.pose.position.z + 0.125; //0.125 is the tested distance for the end effector to grab the cube firmly

  // We specify an intermediate state for the planner because it i stupid and could't find a clear path to goal
  lift_pos = target_pos;
  lift_pos.position.z = 0.18;

  // We define the pose above the goal for he arm to move to
  goal_pos = target_pos;
  goal_pos.position.x = request.goal_loc.point.x;
  goal_pos.position.y = request.goal_loc.point.y;
  goal_pos.position.z = 0.3;  // 0.1(cup height) + 0.125(height from edn effector to bottom of cube) + 0.075(leeway)

  // Open gripper
  bool opgrip_success = moveGripper(1);
  // Move into object position
  bool mvarm_success = moveArm(target_pos);
  // Close gripper
  removeCollisionObject(cube_name);
  bool clgrip_success = moveGripper(0);
  
  // Move to lifted position
  bool mvarm_success2 = moveArm(lift_pos);
  // Move to goal position
  bool mvarm_success3 = moveArm(goal_pos);
  // Open gripper
  bool opgrip_success2 = moveGripper(1);
  // Determine success
  //response.success = opgrip_success && mvarm_success && clgrip_success && mvarm_success2 && opgrip_success2;

  removeCollisionObject(basket_name);
  removeCollisionObject(plane_name);
  
  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t2_callback(cw1_world_spawner::Task2Service::Request &request,
  cw1_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */

  ROS_INFO("The coursework solving callback for task 2 has been triggered");

  addPlane(plane_name);
  addBaskets(basket_name, request.basket_locs);

  // OK so we first try to go over the whole thing, and see if things work out fine. We also display the data from /c200 to see what's going on
  int numb_targets = request.basket_locs.size();
  geometry_msgs::Pose target, target_obs_pose;
  target.orientation.x = 0.9239;
  target.orientation.y = -0.3827;
  target.orientation.z = 0;
  target.orientation.w = 0;

  // initialize the output
  std::vector<std::string> output;
  output.resize(numb_targets);

  bool mvarm_success;
  // loop the location of the baskets
  for (int i=0; i<numb_targets; i++){
    target.position = request.basket_locs[i].point;
    target_obs_pose = target;
    target_obs_pose.position = target.position;
    target_obs_pose.position.z = 0.2;
    mvarm_success = moveArm(target);
    break;
  }

  response.basket_colours = output;
  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t3_callback(cw1_world_spawner::Task3Service::Request &request,
  cw1_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");

  return true;
}



///////////////////////////////////////////////////////////////////////////////

void cw1::addPlane(std::string object_name)
{
  geometry_msgs::Point plane_pos;
  plane_pos.x = 0.45;
  plane_pos.y = 0;
  plane_pos.z = 0.005;

  geometry_msgs::Vector3 plane_dim;
  plane_dim.x = 5;
  plane_dim.y = 9;
  plane_dim.z = 0.03;

  geometry_msgs::Quaternion plate_ori;
  plate_ori.w = 1;
  plate_ori.x = 0;
  plate_ori.y = 0;
  plate_ori.z = 0;

  addCollisionObject(object_name,plane_pos,plane_dim,plate_ori);
}


void cw1::addBaskets(std::string name_prefix, std::vector<geometry_msgs::Point> basket_locs)
{
  // If nothing to add, return
  if(basket_locs.size()<=0) return;

  int num_baskets = basket_locs.size();
  // Define basket location
  geometry_msgs::Vector3 basket_dim;
  basket_dim.x = 0.12;
  basket_dim.y = 0.12;
  basket_dim.z = 0.12;

  geometry_msgs::Quaternion basket_ori;
  basket_ori.w = 1;
  basket_ori.x = 0;
  basket_ori.y = 0;
  basket_ori.z = 0;

  for (int i=0; i<basket_locs.size(); i++)
  {
    basket_name = name_prefix + std::to_string(i + 1);
    geometry_msgs::Point basket_loc = basket_locs[i];
    addCollisionObject(basket_name,basket_loc,basket_dim,basket_ori);
  }
}

void cw1::addBaskets(std::string name_prefix, std::vector<geometry_msgs::PointStamped> basket_locs)
{
  // If nothing to add, return
  if(basket_locs.size()<=0) return;

  std::vector<geometry_msgs::Point> basket_locs_pts;

  for (int i=0; i<basket_locs.size(); i++)
  {
    basket_locs_pts.push_back(basket_locs[i].point);
  }
  addBaskets(name_prefix,basket_locs_pts);
}

void cw1::addCubes(std::string name_prefix, std::vector<geometry_msgs::Point> cube_locs)
{
  // If nothing to add, return
  if(cube_locs.size()<=0) return;

  geometry_msgs::Vector3 cube_dimen;
  geometry_msgs::Quaternion cube_orien;
  geometry_msgs::Point cube_pos;

  cube_dimen.x = 0.06;
  cube_dimen.y = 0.06;
  cube_dimen.z = 0.06;

  cube_orien.w = 1;
  cube_orien.x = 0;
  cube_orien.y = 0;
  cube_orien.z = 0;

  for (int i=0; i<cube_locs.size(); i++)
  {
    cube_name = name_prefix + std::to_string(i + 1);
    cube_pos = cube_locs[i];
    addCollisionObject(cube_name,cube_pos,cube_dimen,cube_orien);
  }
}

void cw1::addCubes(std::string name_prefix, std::vector<geometry_msgs::PointStamped> cube_locs)
{
  // If nothing to add, return
  if(cube_locs.size()<=0) return;

  // Convert Point stamped vector to point vectors
  std::vector<geometry_msgs::Point> cube_locs_pts;
  for (int i=0; i<cube_locs.size(); i++)
  {
    cube_locs_pts.push_back(cube_locs[i].point);
  }
  addCubes(name_prefix,cube_locs_pts);
}





// =====================================================
//            Helper functions
// =====================================================

bool 
cw1::moveArm(geometry_msgs::Pose target_pose)
{
  // setup the target pose
  ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  // create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // google 'c++ conditional operator' to understand this line
  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // execute the planned path
  arm_group_.move();

  return success;
}

////////////////////////////////////////////////////////////////////////////////

bool 
cw1::moveGripper(float width)
{
  // safety checks in case width exceeds safe values
  if (width > gripper_open_) 
    width = gripper_open_;
  if (width < gripper_closed_) 
    width = gripper_closed_;

  // calculate the joint targets as half each of the requested distance
  double eachJoint = width / 2.0;

  // create a vector to hold the joint target for each joint
  std::vector<double> gripperJointTargets(2);
  gripperJointTargets[0] = eachJoint;
  gripperJointTargets[1] = eachJoint;

  // apply the joint target
  hand_group_.setJointValueTarget(gripperJointTargets);

  // move the robot hand
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (hand_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // move the gripper joints
  hand_group_.move();

  return success;
}

////////////////////////////////////////////////////////////////////////////////

void
cw1::addCollisionObject(std::string object_name,
  geometry_msgs::Point centre, geometry_msgs::Vector3 dimensions,
  geometry_msgs::Quaternion orientation)
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

  return;
}

////////////////////////////////////////////////////////////////////////////////

void
cw1::removeCollisionObject(std::string object_name)
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

  return;
}


////////////////////////////////////////////////////////////////////////////////
void
cw1::applyVX (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr)
{
  g_vx.setInputCloud (in_cloud_ptr);
  g_vx.setLeafSize (g_vg_leaf_sz, g_vg_leaf_sz, g_vg_leaf_sz);
  g_vx.filter (*out_cloud_ptr);
  
  return;
}


////////////////////////////////////////////////////////////////////////////////
void
cw1::cloudCallBack
  (const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  // Extract inout point cloud info
  g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;
    
  // Convert to PCL data type
  pcl_conversions::toPCL (*cloud_input_msg, g_pcl_pc);
  pcl::fromPCLPointCloud2 (g_pcl_pc, *g_cloud_ptr);

  // Perform the filtering
  applyVX (g_cloud_ptr, g_cloud_filtered);

  return;
}






////////////////////////////////////////////////////////////////////////////////

void
cw1::set_constraint ()
{
  joint_1.joint_name = "panda_joint1";
  joint_1.position = 0.0;
  joint_1.tolerance_above = M_PI * 0.45;
  joint_1.tolerance_below = M_PI * 0.45;
  joint_1.weight = 1.0;

  joint_2.joint_name = "panda_joint2";
  joint_2.position = 0.4;
  joint_2.tolerance_above = M_PI * 0.8;
  joint_2.tolerance_below = M_PI * 0.8;
  joint_2.weight = 1.0;

  joint_3.joint_name = "panda_joint3";
  joint_3.position = 0.0;
  joint_3.tolerance_above = M_PI * 0.25;
  joint_3.tolerance_below = M_PI * 0.25;
  joint_3.weight = 1.0;

  joint_4.joint_name = "panda_joint4";
  joint_4.position = -2.13;
  joint_4.tolerance_above = M_PI * 0.5;
  joint_4.tolerance_below = M_PI * 0.5;
  joint_4.weight = 1.0;

  joint_5.joint_name = "panda_joint5";
  joint_5.position = 0.0;
  joint_5.tolerance_above = M_PI * 0.3;
  joint_5.tolerance_below = M_PI * 0.3;
  joint_5.weight = 1.0;

  joint_6.joint_name = "panda_joint6";
  joint_6.position = 2.33;
  joint_6.tolerance_above = M_PI * 0.25;
  joint_6.tolerance_below = M_PI * 0.8;
  joint_6.weight = 1.0;

  joint_7.joint_name = "panda_joint7";
  joint_7.position = 0.785;
  joint_7.tolerance_above = M_PI * 0.25;
  joint_7.tolerance_below = M_PI * 0.25;
  joint_7.weight = 1.0;

  // constraints.joint_constraints.push_back(joint_1);
  constraints.joint_constraints.push_back(joint_2);
  constraints.joint_constraints.push_back(joint_3);
  constraints.joint_constraints.push_back(joint_4);
  constraints.joint_constraints.push_back(joint_5);
  // constraints.joint_constraints.push_back(joint_6);
  // constraints.joint_constraints.push_back(joint_7);
  arm_group_.setPathConstraints(constraints);


  return;
}



void
cw1::pick_place_cube(std::string obj_name, geometry_msgs::Point obj_loc, geometry_msgs::Point goal_loc){
  ROS_INFO("picking up and placing: %s", obj_name.c_str());

  // Set constraint
  set_constraint ();

  // We specify waypoints for the arm trajectory
  geometry_msgs::Pose init_pos, target_pos, lift_pos, goal_pos;

  // We specify the location the arm need to reach to pick up the object
  target_pos.orientation.x = 0.9239;
  target_pos.orientation.y = -0.3827;
  target_pos.orientation.z = 0.0;
  target_pos.orientation.w = 0.0;
  target_pos.position.x = obj_loc.x;
  target_pos.position.y = obj_loc.y;
  target_pos.position.z = obj_loc.z + 0.125; //0.125 is the tested distance for the end effector to grab the cube firmly

  // We specify an intermediate state for the planner because it i stupid and could't find a clear path to goal
  lift_pos = target_pos;
  lift_pos.position.z = 0.18;

  // We define the pose above the goal for he arm to move to
  goal_pos = target_pos;
  goal_pos.position.x = goal_loc.x;
  goal_pos.position.y = goal_loc.y;
  goal_pos.position.z = 0.3;  // 0.1(cup height) + 0.125(height from edn effector to bottom of cube) + 0.075(leeway)

  // Open gripper
  bool opgrip_success = moveGripper(1);
  // Move into object position
  bool mvarm_success = moveArm(target_pos);
  // Close gripper
  removeCollisionObject(obj_name);
  bool clgrip_success = moveGripper(0);
  
  // Move to lifted position
  bool mvarm_success2 = moveArm(lift_pos);
  // Move to goal position
  bool mvarm_success3 = moveArm(goal_pos);
  // Open gripper
  bool opgrip_success2 = moveGripper(1);
}

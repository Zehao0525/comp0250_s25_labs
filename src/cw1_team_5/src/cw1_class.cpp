/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

#include <cw1_class.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

///////////////////////////////////////////////////////////////////////////////

cw1::cw1(ros::NodeHandle nh):
  g_cloud_ptr (new PointC), // input point cloud
  g_cloud_filtered (new PointC), // filtered point cloud
  g_cloud_filtered2 (new PointC), // filtered point cloud
  g_cloud_world (new PointC), 
  g_cloud_world_tmp (new PointC), 
  g_tree_ptr (new pcl::search::KdTree<PointT> ()), // KdTree
  debug_ (false),
  g_cloud_dirty (false) // Initialize dirty flag
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

void
cw1::reset_task(){
  g_cloud_ptr.reset(new PointC);             // Reset input point cloud
  g_cloud_filtered.reset(new PointC);        // Reset filtered cloud
  g_cloud_filtered2.reset(new PointC);       // Reset second filtered cloud
  g_cloud_world.reset(new PointC);           // Reset world cloud
  g_cloud_world_tmp.reset(new PointC);       // Reset temporary cloud
  g_tree_ptr.reset(new pcl::search::KdTree<PointT>()); // Reset KdTree
  remove_all_collisions();
  g_cloud_dirty = false;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t1_callback(cw1_world_spawner::Task1Service::Request &request,
  cw1_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */

  ROS_INFO("The coursework solving callback for task 1 has been triggered");
  reset_task();
  // TODO: use the addCollisionObject function
  // Set the orientation and adjust translation

  // To provide leeway, we will inflate all objects by 0.01 in each diection (that matters)

  // create cup object
  // add the collision in the RViz
  // We examine from model.sdf.xmacro that the tiles span from:
  // 0.2 to 0.7 in the x direction, and -0.45 to 0.45 in the y direction
  // Thickness in the z direction is 0.01
  addPlane(plane_name);  
  addCube(cube_name, request.object_loc.pose.position);
  addBasket(basket_name, request.goal_loc.point);

  // geometry_msgs::Vector3 cube_dim;
  // cube_dim.x = 0.06;
  // cube_dim.y = 0.06;
  // cube_dim.z = 0.06;

  // geometry_msgs::Quaternion cube_ori;
  // cube_ori.w = 1;
  // cube_ori.x = 0;
  // cube_ori.y = 0;
  // cube_ori.z = 0;

  // // add the cube
  // addCollisionObject(cube_name, request.object_loc.pose.position,cube_dim,cube_ori);

  // // Define basket location
  // geometry_msgs::Vector3 basket_dim;
  // basket_dim.x = 0.12;
  // basket_dim.y = 0.12;
  // basket_dim.z = 0.12;

  // geometry_msgs::Quaternion basket_ori;
  // basket_ori.w = 1;
  // basket_ori.x = 0;
  // basket_ori.y = 0;
  // basket_ori.z = 0;

  // // add the basket
  // addCollisionObject(basket_name, request.goal_loc.point, basket_dim, basket_ori);

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

  pick_place_cube(cube_name, target_pos.position, goal_pos.position);
  return true;

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
  reset_task();

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
    ROS_INFO("Position (x, y, z): (%f, %f, %f)",
         target.position.x,
         target.position.y,
         target.position.z);
    target_obs_pose = target;
    target_obs_pose.position = target.position;
    target_obs_pose.position.z = 0.5;
    ROS_INFO("Moving arm");
    mvarm_success = moveArm(target_obs_pose);

    // Here we insert a list of 
    std::vector<DetectedObject> detected_objects;
    ROS_INFO("Detecting Object");


    // The whole object detction section
    waitForNewPointCloud();
    convert_ptcld_to_world(g_cloud_filtered, g_cloud_filtered2);
    detect_objects(g_cloud_filtered2, detected_objects);

    // Find closest color
    double min_dist_sq = std::numeric_limits<double>::max();
    std::string closest_color = "";
    
    std::cout << "At location " << i << std::endl;
    // Loop over all detected objects
    for (const auto &obj : detected_objects)
    {
        std::cout << "Object " << std::endl;
        std::cout << "w:" << obj.w << " l:" << obj.l << "h:" << obj.h << std::endl;
        std::cout << "r:" << obj.r << " g:" << obj.g << "b:" << obj.b << std::endl;

        // Skip if color == "other"
        if (obj.color == "other")
            continue;

        // Compute squared distance to target.position
        double dx = target.position.x - obj.position.x;
        double dy = target.position.y - obj.position.y;
        double dist_sq = dx * dx + dy * dy;

        // Keep track of the closest so far
        // Also we only accept distance that is less than 0.05 apart from were we think the basket should be
        if (dist_sq < min_dist_sq && dist_sq < 0.05 * 0.05)
        {
            min_dist_sq = dist_sq;
            closest_color = obj.color;
        }
    }

    // After the loop, check if we found any valid object
    if (closest_color.empty()) {
        // No valid "non-other" objects were found
        output[i] = "none";
    } else {
        // We have the color of the closest object
        output[i] = closest_color;
    }
  }

  response.basket_colours = output;
  for (const auto &s : output) {
    std::cout << s << std::endl ;
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t3_callback(cw1_world_spawner::Task3Service::Request &request,
  cw1_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");
  reset_task();
  addPlane(plane_name);
  addClarance("Clearance");
  set_constraint ();


  // Step 1: mapping the scene
  // We will take visualizations every 0.2 m
  geometry_msgs::Pose target;
  target.orientation.x = 0.9239;
  target.orientation.y = -0.3827;
  target.orientation.z = 0;
  target.orientation.w = 0;

  // Scanning
  for(float y = -0.35; y<=0.41; y+=0.3){
    for(float x = 0.325; x<=0.71; x+=0.25){
      target.position.x = x;
      target.position.y = y;
      target.position.z = 0.6;
      ROS_INFO("Scanning at position: (%f, %f, %f)",
           target.position.x,
           target.position.y,
           target.position.z);
      ROS_INFO("Moving arm");
      moveArm(target);
  
      // Scanning
      waitForNewPointCloud();
      convert_ptcld_to_world(g_cloud_filtered, g_cloud_filtered2);
      // Scan again for another point cloud
      waitForNewPointCloud();
      convert_ptcld_to_world(g_cloud_filtered, g_cloud_filtered2);
    }
  }
  // Scaning phase complete, removing clearance
  removeCollisionObject("Clearance");

  // Step 2: detect all the objects in the scene
  std::vector<DetectedObject> detected_objects;
  ROS_INFO("Detecting Object");
  detect_objects(g_cloud_world, detected_objects);

  // Step 3: find out which baskets we have, and pace cubes with the right color into the right basket

  // Create a map from color to basket positions for quick lookup
  std::unordered_map<std::string, geometry_msgs::Point> basket_map;

  // Populate basket map
  int i=0;
  for (auto& obj : detected_objects) {
    ROS_INFO("Object at position: (%s, %s, %f, %f, %f)",obj.type.c_str(), obj.color.c_str(),
    obj.position.x,
    obj.position.y,
    obj.position.z);
    if (obj.type == "cube" && obj.color != "other") {
      obj.name = "cube" + std::to_string(i);
      addCube(obj.name, obj.position);
    }
    if (obj.type == "basket" && obj.color != "other") {
      obj.name = "basket" + std::to_string(i);
      addBasket(obj.name, obj.position);
      basket_map[obj.color] = obj.position;
    }
    i++;
  }

  // Main logic: place cubes into matching color baskets
  for (auto& cube : detected_objects) {
    if (cube.type == "cube" && cube.color != "other") {
      auto basket_it = basket_map.find(cube.color);
      if (basket_it != basket_map.end()) {
        pick_place_cube(cube.name, cube.position, basket_it->second);
      }
    }
  }

  return true;
}



///////////////////////////////////////////////////////////////////////////////

void cw1::addPlane(std::string object_name)
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
  plate_ori.w = 1;
  plate_ori.x = 0;
  plate_ori.y = 0;
  plate_ori.z = 0;

  addCollisionObject(object_name,plane_pos,plane_dim,plate_ori);
}

void cw1::addClarance(std::string object_name)
{
  geometry_msgs::Point plane_pos;
  plane_pos.x = 0.45;
  plane_pos.y = 0;
  plane_pos.z = 0.06;

  geometry_msgs::Vector3 plane_dim;
  plane_dim.x = 0.52;
  plane_dim.y = 0.92;
  plane_dim.z = 0.12;

  geometry_msgs::Quaternion plate_ori;
  plate_ori.w = 1;
  plate_ori.x = 0;
  plate_ori.y = 0;
  plate_ori.z = 0;

  addCollisionObject(object_name,plane_pos,plane_dim,plate_ori);
}

void cw1::addBasket(std::string name, geometry_msgs::Point basket_pos)
{
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

  addCollisionObject(name,basket_pos,basket_dim,basket_ori);

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

void cw1::addCube(std::string name, geometry_msgs::Point cube_loc)
{

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

  addCollisionObject(name,cube_loc,cube_dimen,cube_orien);
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


void
cw1::remove_all_collisions ()
{
  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;

  collision_object.operation = collision_object.REMOVE;
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);

  return;
}



// =====================================================
//            Helper functions
// =====================================================

bool cw1::moveArm(geometry_msgs::Pose& target_pose)
{
  ROS_INFO("Setting pose target.");
  arm_group_.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool plan_success = false;
  bool exec_success = false;

  int attempts = 0;

  while ((!plan_success || !exec_success) && attempts < 100)
  {
    ROS_INFO("Planning attempt %d...", attempts + 1);
    plan_success = (arm_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    attempts++;
    if (!plan_success){continue;}
    // arm_group_.move()
    auto exec_result = arm_group_.execute(my_plan);
    exec_success = exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    // We wil take time_outs, cause otherwise it will never fcking reach the goals
    exec_success = exec_success || (exec_result == moveit::planning_interface::MoveItErrorCode::TIMED_OUT);
    if(exec_success){break;}
  }
  if (!plan_success || !exec_success){
    ROS_WARN("Plan failed");
  }
  return true;
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
cw1::applyVX (PointCPtr& in_cloud_ptr,
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
  (const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg)
{
  // Extract inout point cloud info
  g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;
  g_input_pc_time_ = pcl_conversions::toPCL(cloud_input_msg->header.stamp);
    
  // Convert to PCL data type
  pcl_conversions::toPCL (*cloud_input_msg, g_pcl_pc);
  pcl::fromPCLPointCloud2 (g_pcl_pc, *g_cloud_ptr);

  // Perform the filtering
  applyVX (g_cloud_ptr, g_cloud_filtered);
  g_cloud_filtered->header.frame_id = g_input_pc_frame_id_;
  g_cloud_filtered->header.stamp = g_input_pc_time_;

  g_cloud_dirty = true;

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


////////////////////////////////////////////////////////////////////////////////

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
  //target_pos.position.z = obj_loc.z + 0.125; //0.125 is the tested distance for the end effector to grab the cube firmly
  target_pos.position.z = 0.03 + 0.125; //0.03 is the hard coded position of the cubes

  // We specify an intermediate state for the planner because it i stupid and could't find a clear path to goal
  lift_pos = target_pos;
  lift_pos.position.z = 0.3;

  // We define the pose above the goal for he arm to move to
  goal_pos = target_pos;
  goal_pos.position.x = goal_loc.x;
  goal_pos.position.y = goal_loc.y;
  goal_pos.position.z = 0.3;  // 0.1(cup height) + 0.125(height from edn effector to bottom of cube) + 0.075(leeway)

  // Move into object position
  // Move to lifted position
  bool mvarm_success = moveArm(lift_pos);

  // Open gripper
  bool opgrip_success = moveGripper(1);

  mvarm_success = mvarm_success && moveArm(target_pos);
  // Close gripper
  removeCollisionObject(obj_name);
  bool clgrip_success = moveGripper(0);
  
  // Move to lifted position
  mvarm_success = mvarm_success && moveArm(lift_pos);
  // Move to goal position
  mvarm_success = mvarm_success && moveArm(goal_pos);
  // Open gripper
  bool opgrip_success2 = moveGripper(0.8);
}


// Condition function 
bool 
enforceColorSimilarity(const PointT& a, const PointT& b, float /*squared_dist*/)
{
  float bScale = std::max({b.r, b.g, b.b});
  float aScale = std::max({a.r, a.g, a.b});
  float tol = 0.05f * bScale * aScale;
  // This below logic will ignore pure black clusters. (Or be undefined, but I think we wouldn't get 
  // floating point error on *0, so it is ignore)
  // This is the price we pay for saving the compute on handeling scale == 0 cases
  return (std::fabs(a.r * bScale - b.r * aScale) < tol) && (std::fabs(a.g *bScale- b.g * aScale) < tol) && (std::fabs(a.b * bScale - b.b * aScale) < tol);
}

void 
cw1::waitForNewPointCloud()
{
  ros::Rate rate(10.0);  // 10 Hz loop rate (checks 10 times per second)

  while (ros::ok() && !g_cloud_dirty)
  {
    ros::spinOnce();  // Process incoming messages
    rate.sleep();     // Wait until next loop iteration (0.1 seconds)
  }

  if (g_cloud_dirty)
  {
    g_cloud_dirty = false;  // Reset flag after detection if appropriate
  }
}


void cw1::convert_ptcld_to_world(PointCPtr& input_cloud, PointCPtr& transformed_cloud)
{

  PointCPtr tmp_cloud(new PointC);
  // Wait for transform availability
  if (!g_listener_.waitForTransform(base_frame_,
        input_cloud->header.frame_id,
        input_cloud_msg->header.stamp, ros::Duration(3.0)))
  {
    ROS_WARN("Transform unavailable from %s to %s",
      input_cloud->header.frame_id.c_str(), base_frame_.c_str());
    return;
  }
  ROS_INFO("Transforming from%s to %s",input_cloud->header.frame_id.c_str(), base_frame_.c_str());

  // Perform the transformation
  try
  {
    pcl_ros::transformPointCloud(base_frame_, *input_cloud, *tmp_cloud, g_listener_);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("Transformation error: %s", ex.what());
    return;
  }

  // Do a z filter to remove the ground plane
  // Reduce memory consumption
  pass_filter.setInputCloud(tmp_cloud);
  pass_filter.setFilterFieldName("z");
  pass_filter.setFilterLimits(0.015, std::numeric_limits<float>::max()); // keeps points with z ≥ 0.015
  pass_filter.filter(*transformed_cloud);

  // Update header
  transformed_cloud->header.frame_id  = base_frame_;
  transformed_cloud->header.stamp = input_cloud->header.stamp;

  // Merge point clouds
  *g_cloud_world += *transformed_cloud;
  g_cloud_world->header.frame_id = base_frame_;
  g_cloud_world->header.stamp = pcl_conversions::toPCL(ros::Time::now());
} 

////////////////////////////////////////////////////////////////////////////////
void cw1::detect_objects(PointCPtr& in_cloud_ptr, std::vector<DetectedObject>& detected_objects){

  // --- 1. Cluster Extraction ---
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(in_cloud_ptr);
  // For each cluster, 
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::ConditionalEuclideanClustering<PointT> cec(true);   // 'true' to allow cluster size filtering
  cec.setInputCloud(in_cloud_ptr);
  cec.setConditionFunction(&enforceColorSimilarity);
  cec.setClusterTolerance(0.02f);                        // 5mm spatial cluster radius
  cec.setMinClusterSize(25);  
  cec.setSearchMethod(tree); 

  // Cluster pointcloud
  cec.segment(cluster_indices);

  // Counters for cubes and baskets
  int cube_count = 0, basket_count = 0;


  geometry_msgs::Point obj_position_tmp;
  // --- 2. Process each cluster ---
  for (const auto &indices : cluster_indices)
  {
    // Create a new cloud for the current cluster
    PointCPtr cluster(new PointC);
    for (const auto &idx : indices.indices)
      cluster->points.push_back(in_cloud_ptr->points[idx]);
    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;

    // Compute the axis-aligned bounding box (assuming objects are well-aligned)
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);

    Eigen::Vector3f dims(1.0f, 2.0f, 3.0f);
    dims[0] = std::fabs(max_pt.x - min_pt.x);
    dims[1] = std::fabs(max_pt.y - min_pt.y);
    dims[2] = std::fabs(max_pt.z - min_pt.z);

    // --- 3. Classification based on dimensions ---
    // Here we assume:
    //   • A cube will have roughly equal sides of ~0.04 (with some tolerance)
    //   • A basket (rectangular cuboid outline) will have outer dimensions ~0.1
    // We find the coordinates of the object

    // The x and y coordinates are the x and y, but z is the minimum z
    Eigen::Vector4f cluster_centroid;
    pcl::compute3DCentroid(*cluster, cluster_centroid);

    g_cyl_pt_msg.header.frame_id = g_input_pc_frame_id_;
    g_cyl_pt_msg.header.stamp = ros::Time (0);
    g_cyl_pt_msg.point.x = cluster_centroid[0];
    g_cyl_pt_msg.point.y = cluster_centroid[1];
    g_cyl_pt_msg.point.z = cluster_centroid[2];

    float tol = 0.02f; // tolerance in meters

    bool isCube = (std::fabs(dims[0] - 0.04f) < tol &&
                    std::fabs(dims[1] - 0.04f) < tol && 
                    std::fabs(max_pt.z - 0.05f) < tol);

    bool isBasket = (std::fabs(dims[0] - 0.1f) < tol &&
                     std::fabs(dims[1] - 0.1f) < tol && 
                     std::fabs(max_pt.z - 0.11) < tol);


    // obtain rgba data as the avergae rgb values
    int r = 0;
    int g = 0;
    int b = 0;
    // retrieve the RGB data
    for (int j = 0; j < cluster->points.size(); j++)
    {
      rgba = cluster->points[j].rgba;
      uint8_t uint8_r = (rgba >> 16) & 0x0000ff;
      uint8_t uint8_g = (rgba >> 8) & 0x0000ff;
      uint8_t uint8_b = (rgba)&0x0000ff;
      uint8_t uint8_a = (rgba >> 24) & 0x000000ff;

      r = r + uint8_r;
      g = g + uint8_g;
      b = b + uint8_b;
    }

    // take the average number of rgb of the image area
    r = r / cluster->points.size();
    g = g / cluster->points.size();
    b = b / cluster->points.size();

    
    // --- 4. Data analysis
    // We find the color of the object
    // Use better conditions
    DetectedObject cur_obj;
    float colorScale = std::max({r, g, b}) / 0.8; // This will scale the Color to the GT RGB colors

    // Get meta information
    cur_obj.w = dims[0];
    cur_obj.l = dims[1];
    cur_obj.h = dims[2];
    cur_obj.r = r;
    cur_obj.g = g;
    cur_obj.b = b;

    // Assign position
    cur_obj.position = g_cyl_pt_msg.point;

    if(colorScale == 0){cur_obj.color = "other";}
    else if (std::fabs(r / colorScale - red[0])<0.1 && std::fabs(g / colorScale - red[1])<0.1 && std::fabs(b / colorScale - red[2])<0.1)
    {
      cur_obj.color = "red";
    }
    else if (std::fabs(r / colorScale - blue[0])<0.1 && std::fabs(g / colorScale - blue[1])<0.1 && std::fabs(b / colorScale - blue[2])<0.1)
    {
      cur_obj.color = "blue";
    }
    else if (std::fabs(r / colorScale - purple[0])<0.1 && std::fabs(g / colorScale - purple[1])<0.1 && std::fabs(b / colorScale - purple[2])<0.1)
    {
      cur_obj.color = "purple";
    }
    else
    {
      cur_obj.color = "other";
    }

    // We find the type of the object
    std::cout << "obj z: " << obj_position_tmp.z << std::endl;
    if (isCube)
    {
      cur_obj.type = "cube";
      std::cout << "Detected cube. Dimensions (m): " << dims << std::endl;
    }
    else if (isBasket)
    {
      cur_obj.type = "basket";
      std::cout << "Detected basket. Dimensions (m): " << dims << std::endl;
    }
    else
    {
      cur_obj.type = "other";
      std::cout << "Unknown object. Dimensions (m): " << dims << std::endl;
    }

    detected_objects.push_back(cur_obj);
  }



  return;
}

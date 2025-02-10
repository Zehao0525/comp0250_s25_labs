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
  g_cloud_ptr_task_3 (new PointC), // input point cloud
  g_cloud_filtered (new PointC), // filtered point cloud
  g_cloud_filtered2 (new PointC), // filtered point cloud
  g_cloud_filtered_task_3 (new PointC), // filtered point cloud
  g_cloud_filtered2_task_3 (new PointC), // filtered point cloud
  g_cloud_plane (new PointC), // plane point cloud
  g_cloud_cylinder (new PointC), // cylinder point cloud
  // g_tree_ptr (new pcl::search::KdTree<PointT> ()), // KdTree
  g_tree_ptr (new pcl::search::KdTree<PointT>), // KdTree

  singal_cluster_ptr (new pcl::PointIndices), //cluster
  singal_cluster(new pcl::PointCloud<PointT>), //singal cluster

  g_cloud_normals (new pcl::PointCloud<pcl::Normal>), // segmentation
  g_cloud_normals2 (new pcl::PointCloud<pcl::Normal>), // segmentation
  g_inliers_plane (new pcl::PointIndices), // plane seg
  g_inliers_cylinder (new pcl::PointIndices), // cylidenr seg
  g_coeff_plane (new pcl::ModelCoefficients), // plane coeff
  g_coeff_cylinder (new pcl::ModelCoefficients), // cylinder coeff
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

  ros::Subscriber sub_target_pose = 
                  nh_.subscribe ("/target_pos",
                  10,
                  &cw1::target_pose_callback,
                  this);

  // define the publisher
  g_pub_task_2 = nh_.advertise<sensor_msgs::PointCloud2> ("filtered_task_2", 1, true);
  g_pub_task_3 = nh_.advertise<sensor_msgs::PointCloud2> ("filtered_task_3", 1, true);
  g_pub_pose = nh_.advertise<geometry_msgs::PointStamped> ("target_pos", 1, true);

  // publisher for cluster testing
  pub_cluster = nh_.advertise<sensor_msgs::PointCloud2> ("cluster_testing", 1, true);

  // define the constants for PCL
  // g_pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud", 1, true);
  g_vg_leaf_sz = 0.0005; // VoxelGrid leaf size: Better in a config file
  g_pt_thrs_min_x = -0.05; // PassThrough min thres: Better in a config file
  g_pt_thrs_max_x = 0.05; // PassThrough max thres: Better in a config file
  g_pt_thrs_min_y = -0.01; // PassThrough min thres: Better in a config file
  g_pt_thrs_max_y = 0.09; // PassThrough max thres: Better in a config file
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

  // add the collision in the RViz
  geometry_msgs::Point plate_position;
  plate_position.x = 0;
  plate_position.x = 0;
  plate_position.x = 0;

  geometry_msgs::Vector3 plate_dimension;
  plate_dimension.x = 5;
  plate_dimension.y = 5;
  plate_dimension.z = 0.03;

  geometry_msgs::Quaternion plate_orientation;
  plate_orientation.w = 1;
  plate_orientation.x = 0;
  plate_orientation.y = 0;
  plate_orientation.z = 0;

  addCollisionObject(plate_name,plate_position,plate_dimension,plate_orientation);
  

  geometry_msgs::Vector3 cube_dimension;
  cube_dimension.x = 0.07;
  cube_dimension.y = 0.07;
  cube_dimension.z = 0.08;

  geometry_msgs::Quaternion cube_orientation;
  cube_orientation.w = 1;
  cube_orientation.x = 0;
  cube_orientation.y = 0;
  cube_orientation.z = 0;

  // add the cube
  addCollisionObject(cube_name, request.object_loc.pose.position,cube_dimension,cube_orientation);

  geometry_msgs::Vector3 basket_dimension;
  basket_dimension.x = 0.12;
  basket_dimension.y = 0.12;
  basket_dimension.z = 0.16;

  geometry_msgs::Quaternion basket_orientation;
  basket_orientation.w = 1;
  basket_orientation.x = 0;
  basket_orientation.y = 0;
  basket_orientation.z = 0;

  // add the basket
  addCollisionObject(basket_name, request.goal_loc.point, basket_dimension, basket_orientation);

  // define the constraint to joints
  set_constraint ();

  // move the arm to achieve pick and place
  geometry_msgs::Pose target_cube, target_cube_ahead, target_goal, intermedia;

  // define the position and orientation of the cube
  target_cube.position = request.object_loc.pose.position;
  target_cube.orientation.x = 0.9239;
  target_cube.orientation.y = -0.3827;
  target_cube.orientation.z = 0;
  target_cube.orientation.w = 0;
  target_cube.position.z = target_cube.position.z + 0.12;

  // define the target_cube_ahead
  target_cube_ahead = target_cube;
  target_cube_ahead.position.z = 0.18;
  target_cube_ahead.orientation.x = 0.9239;
  target_cube_ahead.orientation.y = -0.3827;
  target_cube_ahead.orientation.z = 0;
  target_cube_ahead.orientation.w = 0;

  // define the position and orientation of the goal
  target_goal.position = request.goal_loc.point;
  target_goal.orientation = target_cube.orientation;
  target_goal.orientation.x = 1;
  target_goal.orientation.y = 0;
  target_goal.position.z = 0.35;

  // define the position and orientation of the intermedia point
  intermedia = target_cube_ahead;
  intermedia.position.z = target_cube_ahead.position.z;

  moveGripper(1);
  moveArm(target_cube_ahead);
  moveArm(target_cube);
  removeCollisionObject(cube_name);
  moveGripper(0.01);
  moveArm(target_goal);
  moveGripper(1);
  removeCollisionObject(basket_name);
  removeCollisionObject(plate_name);
  

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t2_callback(cw1_world_spawner::Task2Service::Request &request,
  cw1_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */

  ROS_INFO("The coursework solving callback for task 2 has been triggered");

  // get the number of target
  int numb_targets;
  numb_targets = request.basket_locs.size();

  // moving the arm to the targets
  // define the pose of manipulator
  geometry_msgs::Pose target;
  target.orientation.x = 0.9239;
  target.orientation.y = -0.3827;
  target.orientation.z = 0;
  target.orientation.w = 0;

  // define the constraint to joints
  moveit_msgs::Constraints constraints;
  moveit_msgs::JointConstraint joint_3, joint_5;
  joint_5.joint_name = "panda_joint5";
  joint_5.position = 0.0;
  joint_5.tolerance_above = 0.5;
  joint_5.tolerance_below = 0.5;
  joint_5.weight = 1.0;

  joint_3.joint_name = "panda_joint3";
  joint_3.position = 0.0;
  joint_3.tolerance_above = 0.5;
  joint_3.tolerance_below = 0.5;
  joint_3.weight = 1.0;

  constraints.joint_constraints.push_back(joint_3);
  constraints.joint_constraints.push_back(joint_5);
  arm_group_.setPathConstraints(constraints);

  // visualise the point cloud
  // pcl::visualization::CloudViewer viewer ("Cluster viewer");
  // viewer.showCloud(g_cloud_ptr);

  // initialize the output
  std::vector<std::string> output;
  output.resize(numb_targets);

  // loop the location of the baskets
  for (int i=0; i<numb_targets; i++){
    target.position = request.basket_locs[i].point;
    // adjust the camera ahead of the basket
    target.position.z = 0.5;
    moveArm(target);

    // initialize the r, g, b data
    int r = 0;
    int g = 0;
    int b = 0;
    // retrieve the RGB data
    for (int j=0; j<(*g_cloud_filtered2).points.size(); j++){
      rgba = (*g_cloud_filtered2).points[j].rgba;
      uint8_t uint8_r = (rgba >> 16) & 0x0000ff;
      uint8_t uint8_g = (rgba >> 8)  & 0x0000ff;
      uint8_t uint8_b = (rgba)       & 0x0000ff;
      uint8_t uint8_a = (rgba >> 24) & 0x000000ff;

      r = r + uint8_r;
      g = g + uint8_g;
      b = b + uint8_b;
    }
    // take the average number of rgb of the image area
    r = r / (*g_cloud_filtered2).points.size();
    g = g / (*g_cloud_filtered2).points.size();
    b = b / (*g_cloud_filtered2).points.size();

    if (r/255.0 <= red[0] + 0.1 && g/255.0 <= red[1] + 0.1 && b/255.0 <= red[2] + 0.1){
      output[i] = "red";
      // std::cout << "red" << std::endl;
      // std::cout << "_____________________________" << std::endl;
    }
    else if (r/255.0 <= blue[0] + 0.1 && g/255.0 <= blue[1] + 0.1 && b/255.0 <= blue[2] + 0.1){
      output[i] = "blue";
      // std::cout << "blue" << std::endl;
      // std::cout << "_____________________________" << std::endl;

    }
    else if (r/255.0 <= purple[0] + 0.1 && g/255.0 <= purple[1] + 0.1 && b/255.0 <= purple[2] + 0.1){
      output[i] = "purple";
      // std::cout << "purple" << std::endl;
      // std::cout << "_____________________________" << std::endl;
    }
    else {
      output[i] = "empty";
      // std::cout << "empty" << std::endl;
      // std:cout << "_____________________________" << std::endl;
    }
  }

  // assign the output to the response
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

  remove_all_collisions();

  // initialise the flags
  cluster_or_not = false;
  task_3_triger = false;
  cluster_done = false;

  // clear the vectors
  red_cube.clear();
  blue_cube.clear();
  purple_cube.clear();
  red_basket.clear();
  blue_basket.clear();
  purple_basket.clear();

  // define the constraint to joints
  set_constraint ();

  // move the arm to the target pose
  geometry_msgs::Pose target_pose_1, target_pose_2;
  target_pose_1.position.x = 0.41;
  target_pose_1.position.y = 0.3;
  target_pose_1.position.z = 0.7;

  target_pose_1.orientation.x = 0.9239;
  target_pose_1.orientation.y = -0.3827;
  target_pose_1.orientation.z = 0;
  target_pose_1.orientation.w = 0;

  target_pose_2 = target_pose_1;
  target_pose_2.position.y = -target_pose_1.position.y;

  // move to the first detection position
  moveArm(target_pose_1);

  cluster_or_not = true;
  task_3_triger = true;

  // hand on until the first clustering is finished
  while(!cluster_done)
  {}

  // move to the second detection position
  cluster_or_not = false;
  task_3_triger = false;
  cluster_done = false;
  moveArm(target_pose_2);

  cluster_or_not = true;
  task_3_triger = true;

  // hand on until the second clustering is finished
  while(!cluster_done)
  {}
  cluster_or_not = false;
  task_3_triger = false;

  // discrad the repeated position
  red_cube = cw1::discard_repeat_pos(red_cube);
  blue_cube = cw1::discard_repeat_pos(blue_cube);
  purple_cube = cw1::discard_repeat_pos(purple_cube);
  red_basket = cw1::discard_repeat_pos(red_basket);
  blue_basket = cw1::discard_repeat_pos(blue_basket);
  purple_basket = cw1::discard_repeat_pos(purple_basket);
  ROS_INFO("Discard the repeated cube and basket finished");


  /////////////////////////////////////////
  // print_cube_basket();
  /////////////////////////////////////////

  // add the collision for path planning
  add_collision();

  ROS_INFO("Pick and place starts");
  pick_and_place(red_cube, red_basket, "red_cube");
  pick_and_place(blue_cube, blue_basket, "blue_cube");
  pick_and_place(purple_cube, purple_basket, "purple_cube");



  return true;
}

////////////////////////////////////////////////////////////////////////////////

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

  moveArm_done = true;

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
cw1::cloudCallBackOne
  (const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  // // Extract inout point cloud info
  g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;
    
  // Convert to PCL data type
  pcl_conversions::toPCL (*cloud_input_msg, g_pcl_pc);
  pcl::fromPCLPointCloud2 (g_pcl_pc, *g_cloud_ptr);

  // apply the filter to filter specific area
  applyPT_x (g_cloud_ptr, g_cloud_filtered);
  applyPT_y (g_cloud_filtered, g_cloud_filtered2);

  // std::cout << (*g_cloud_filtered2) << std::endl;

  pubFilteredPCMsg(g_pub_task_2, *g_cloud_filtered2);

  return;
}

////////////////////////////////////////////////////////////////////////////////

void
cw1::cloudCallBackTwo
  (const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg_task_3)
{
  while (task_3_triger)
  {
    // Extract inout point cloud info
    g_input_pc_frame_id_ = cloud_input_msg_task_3->header.frame_id;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_input_msg_task_3, g_pcl_pc_task_3);
    pcl::fromPCLPointCloud2(g_pcl_pc_task_3, *g_cloud_ptr_task_3);

    // Perform the filtering
    applyVX(g_cloud_ptr_task_3, g_cloud_filtered_task_3);

    // Segment plane and cylinder
    findNormals(g_cloud_filtered_task_3);
    segPlane(g_cloud_filtered_task_3);
    segCylind(g_cloud_filtered_task_3);

    // clustering the cubes and baskets
    if (cluster_or_not)
    {
      cluster_or_not = false;
      cluster(g_cloud_cylinder);
    }

    // Publish the data
    pcl::toROSMsg(*g_cloud_cylinder, g_cloud_filtered_msg);
    // pcl::toROSMsg(*g_cloud_plane, g_cloud_filtered_msg);
    g_pub_task_3.publish(g_cloud_filtered_msg);
  }

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
cw1::applyPT_x (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr)
{
  g_pt.setInputCloud (in_cloud_ptr);
  g_pt.setFilterFieldName ("x");
  g_pt.setFilterLimits (g_pt_thrs_min_x, g_pt_thrs_max_x);
  g_pt.filter (*out_cloud_ptr);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

void
cw1::applyPT_y (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr)
{
  g_pt.setInputCloud (in_cloud_ptr);
  g_pt.setFilterFieldName ("y");
  g_pt.setFilterLimits (g_pt_thrs_min_y, g_pt_thrs_max_y);
  g_pt.filter (*out_cloud_ptr);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

void
cw1::pubFilteredPCMsg (ros::Publisher &pc_pub,
                               PointC &pc)
{
  // Publish the data
  pcl::toROSMsg(pc, g_cloud_filtered_msg);
  pc_pub.publish (g_cloud_filtered_msg);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

void
cw1::findNormals (PointCPtr &in_cloud_ptr)
{
  // Estimate point normals
  g_ne.setInputCloud (in_cloud_ptr);
  g_ne.setSearchMethod (g_tree_ptr);
  g_ne.setKSearch (g_k_nn);
  g_ne.compute (*g_cloud_normals);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

void
cw1::segPlane (PointCPtr &in_cloud_ptr)
{
  // Create the segmentation object for the planar model
  // and set all the params
  g_seg.setOptimizeCoefficients (true);
  g_seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  // g_seg.setNormalDistanceWeight (0.1); //bad style

  g_seg.setNormalDistanceWeight (0.0001); //bad style

  g_seg.setMethodType (pcl::SAC_RANSAC);
  g_seg.setMaxIterations (1000); //bad style
  g_seg.setDistanceThreshold (0.03); //bad style original
  // g_seg.setDistanceThreshold (0.01); //bad style


  g_seg.setInputCloud (in_cloud_ptr);
  g_seg.setInputNormals (g_cloud_normals);
  // Obtain the plane inliers and coefficients
  g_seg.segment (*g_inliers_plane, *g_coeff_plane);
  
  // Extract the planar inliers from the input cloud
  g_extract_pc.setInputCloud (in_cloud_ptr);
  g_extract_pc.setIndices (g_inliers_plane);
  g_extract_pc.setNegative (false);
  
  // Write the planar inliers to disk
  g_extract_pc.filter (*g_cloud_plane);
  
  // Remove the planar inliers, extract the rest
  g_extract_pc.setNegative (true);
  g_extract_pc.filter (*g_cloud_filtered2_task_3);
  g_extract_normals.setNegative (true);
  g_extract_normals.setInputCloud (g_cloud_normals);
  g_extract_normals.setIndices (g_inliers_plane);
  g_extract_normals.filter (*g_cloud_normals2);

  //ROS_INFO_STREAM ("Plane coefficients: " << *g_coeff_plane);
  ROS_INFO_STREAM ("PointCloud representing the planar component: "
                   << g_cloud_plane->size()
                   << " data points.");
}

////////////////////////////////////////////////////////////////////////////////

void
cw1::segCylind (PointCPtr &in_cloud_ptr)
{
  // Create the segmentation object for cylinder segmentation
  // and set all the parameters
  g_seg.setOptimizeCoefficients (true);
  g_seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
  g_seg.setMethodType (pcl::SAC_RANSAC);

  // g_seg.setNormalDistanceWeight (0.1); //bad style
  g_seg.setNormalDistanceWeight (0.0001); //bad style

  g_seg.setMaxIterations (10000); //bad style
  // g_seg.setDistanceThreshold (0.05); //bad style
  g_seg.setDistanceThreshold (0.1); //bad style


  // g_seg.setRadiusLimits (0, 0.1); //bad style
  g_seg.setInputCloud (g_cloud_filtered2_task_3);
  g_seg.setInputNormals (g_cloud_normals2);

  // Obtain the cylinder inliers and coefficients
  g_seg.segment (*g_inliers_cylinder, *g_coeff_cylinder);
  
  // Write the cylinder inliers to disk
  g_extract_pc.setInputCloud (g_cloud_filtered2_task_3);
  g_extract_pc.setIndices (g_inliers_cylinder);
  g_extract_pc.setNegative (false);
  g_extract_pc.filter (*g_cloud_cylinder);
  
  ROS_INFO_STREAM ("PointCloud representing the cylinder component: "
                   << g_cloud_cylinder->size ()
                   << " data points.");

  segCylind_done = true;
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

void
cw1::findCylPose (PointCPtr &in_cloud_ptr)
{
  task_3_position.x = 0;
  task_3_position.y = 0;
  task_3_position.z = 0;

  Eigen::Vector4f centroid_in;
  pcl::compute3DCentroid(*in_cloud_ptr, centroid_in);
  
  g_cyl_pt_msg.header.frame_id = g_input_pc_frame_id_;
  g_cyl_pt_msg.header.stamp = ros::Time (0);
  g_cyl_pt_msg.point.x = centroid_in[0];
  g_cyl_pt_msg.point.y = centroid_in[1];
  g_cyl_pt_msg.point.z = centroid_in[2];
  
  // Transform the point to new frame
  geometry_msgs::PointStamped g_cyl_pt_msg_out;
  try
  {
    g_listener_.transformPoint ("panda_link0",  // bad styling
                                g_cyl_pt_msg,
                                g_cyl_pt_msg_out);
    //ROS_INFO ("trying transform...");
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR ("Received a trasnformation exception: %s", ex.what());
  }
  
  // publishPose (g_cyl_pt_msg_out);

  task_3_position = g_cyl_pt_msg_out.point;
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

void
cw1::cluster (PointCPtr &in_cloud_ptr)
{

  // ros::AsyncSpinner spinner(0.25);
  // spinner.start();

  ROS_INFO("start clustering");

  // use k-means to cluster the filtered point cloud, and get the indices
  g_tree_ptr->setInputCloud(in_cloud_ptr);
  // std::vector<pcl::PointIndices> cluster_indices;
  g_ec.setClusterTolerance(0.02);
  g_ec.setMinClusterSize(100);
  g_ec.setMaxClusterSize(25000);
  // ROS_INFO("parameters setup done");

  g_ec.setSearchMethod(g_tree_ptr);
  // ROS_INFO("method setup done");

  g_ec.setInputCloud(in_cloud_ptr);
  // ROS_INFO("input setup done");

  cluster_indices.clear();
  g_ec.extract(cluster_indices);
  // ROS_INFO("extract done");

  // std::cout << "////////////////////////////////////////////" << std::endl;
  // std::cout << cluster_indices.size() << std::endl;
  // std::cout << "////////////////////////////////////////////" << std::endl;


  // extract the data from each cluster
  for (int i = 0; i < cluster_indices.size(); i++)
  {
    *singal_cluster_ptr = cluster_indices[i];
    g_extract_pc.setInputCloud(in_cloud_ptr);
    g_extract_pc.setIndices(singal_cluster_ptr);
    g_extract_pc.filter(*singal_cluster);

    // initialize the r, g, b data
    int r = 0;
    int g = 0;
    int b = 0;
    // retrieve the RGB data
    for (int j = 0; j < (*singal_cluster).points.size(); j++)
    {
      rgba = (*singal_cluster).points[j].rgba;
      uint8_t uint8_r = (rgba >> 16) & 0x0000ff;
      uint8_t uint8_g = (rgba >> 8) & 0x0000ff;
      uint8_t uint8_b = (rgba)&0x0000ff;
      uint8_t uint8_a = (rgba >> 24) & 0x000000ff;

      r = r + uint8_r;
      g = g + uint8_g;
      b = b + uint8_b;
    }
    // take the average number of rgb of the image area
    r = r / (*singal_cluster).points.size();
    g = g / (*singal_cluster).points.size();
    b = b / (*singal_cluster).points.size();

    // std::cout << "////////////////////////////////////////////" << std::endl;
    // std::cout << "cluster " << i << std::endl;
    // std::cout << "number of points = " << (*singal_cluster).points.size() << std::endl;
    // std::cout << "r = " << r / 255.0 << std::endl;
    // std::cout << "g = " << g / 255.0 << std::endl;
    // std::cout << "b = " << b / 255.0 << std::endl;

    // std::cout << "////////////////////////////////////////////" << std::endl;

    if (r / 255.0 <= red[0]+0.05 && g / 255.0 <= red[1]+0.05 && b / 255.0 <= red[2]+0.05)
    {
      // output[i] = "red";
      // std::cout << "red" << std::endl;
      findCylPose(singal_cluster);
      // std::cout << task_3_position << std::endl;
      // std::cout << "_____________________________" << std::endl;
      // cube or basket?
      if((*singal_cluster).points.size()>4500)
      {
        red_basket.push_back(task_3_position);
      }
      else
      {
        red_cube.push_back(task_3_position);
      }
    }
    else if (r / 255.0 <= blue[0]+0.05 && g / 255.0 <= blue[1]+0.05 && b / 255.0 <= blue[2]+0.05)
    {
      // output[i] = "blue";
      // std::cout << "blue" << std::endl;
      findCylPose(singal_cluster);
      // std::cout << task_3_position << std::endl;
      // std::cout << "_____________________________" << std::endl;
      // cube or basket?
      if((*singal_cluster).points.size()>4500)
      {
        blue_basket.push_back(task_3_position);
      }
      else
      {
        blue_cube.push_back(task_3_position);
      }
    }
    else if (r / 255.0 <= purple[0]+0.05 && g / 255.0 <= purple[1]+0.05 && b / 255.0 <= purple[2]+0.05)
    {
      // output[i] = "purple";
      // std::cout << "purple" << std::endl;
      findCylPose(singal_cluster);
      // std::cout << task_3_position << std::endl;
      // std::cout << "_____________________________" << std::endl;
      // cube or basket?
      if((*singal_cluster).points.size()>4500)
      {
        purple_basket.push_back(task_3_position);
      }
      else
      {
        purple_cube.push_back(task_3_position);
      }
    }
    else
    {
      // output[i] = "empty";
      // std::cout << "none of them" << std::endl;
      // std:cout << "_____________________________" << std::endl;
    }


    // // publish the cluster
    // pcl::toROSMsg(*singal_cluster, clustered);
    // pub_cluster.publish (clustered);

    moveArm_done = false;

    // std::cout << "////////////////////////////////////////////" << std::endl;
    // std::cout << cluster_indices.size() << std::endl;
    // std::cout << "////////////////////////////////////////////" << std::endl;
  }

  ROS_INFO("Clustering finished");

  cluster_done = true;

  return;
}

////////////////////////////////////////////////////////////////////////////////

void
cw1::publishPose (geometry_msgs::PointStamped &cyl_pt_msg)
{
  // Create and publish the cylinder pose (ignore orientation)

  g_pub_pose.publish (cyl_pt_msg);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

void
cw1::target_pose_callback (const geometry_msgs::PointStamped &cyl_pt_msg)
{
  // subscribe the cylinder pose (ignore orientation)

  // std::cout << cyl_pt_msg << std::endl;

  cyl_pt_msg_header = cyl_pt_msg.header;
  cyl_pt_msg_point = cyl_pt_msg.point;
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<geometry_msgs::Point>
cw1::discard_repeat_pos (std::vector<geometry_msgs::Point> in_vector)
{
  std::vector<geometry_msgs::Point> temp_vec;
  geometry_msgs::Point temp_pos;
  bool same;

  if (in_vector.size() > 1)
  {
    for (int i = 0; i < in_vector.size()-1; i++)
    {
      same = false;
      temp_pos = in_vector[i];
      for (int j = i+1; j < in_vector.size(); j++)
      {
        if (abs(temp_pos.x-in_vector[j].x)<0.01 && abs(temp_pos.y-in_vector[j].y)<0.01)
        {
          same = true;
        }
      }
      if(!same)
      {
        temp_vec.push_back(temp_pos);
      }
    }
    temp_vec.push_back(in_vector[in_vector.size()-1]);

    // update the order 
    int n=temp_vec.size();
    bool swap;
    do{
      swap = false;
      for (int k=1; k<n; ++k)
      {
        if (temp_vec[k-1].x<temp_vec[k].x)
        {
          std::swap(temp_vec[k-1], temp_vec[k]);
          swap = true;
        }
      }
      --n;
    }while(swap);
  }
  else
  {
    temp_vec = in_vector;
  }

  return temp_vec;
}

////////////////////////////////////////////////////////////////////////////////

void 
cw1::print_cube_basket()
{
  std::cout << "after" << std::endl;

  std::cout << "red cube" << std::endl;
  for (int i = 0; i < red_cube.size(); i++)
  {
    std::cout << red_cube[i] << std::endl;
    std::cout << "____________________" << std::endl;
  }
  std::cout << "////////////////////////////////////////////" << std::endl;

  std::cout << "blue cube" << std::endl;
  for (int i = 0; i < blue_cube.size(); i++)
  {
    std::cout << blue_cube[i] << std::endl;
    std::cout << "____________________" << std::endl;
  }
  std::cout << "////////////////////////////////////////////" << std::endl;

  std::cout << "purple cube" << std::endl;
  for (int i = 0; i < purple_cube.size(); i++)
  {
    std::cout << purple_cube[i] << std::endl;
    std::cout << "____________________" << std::endl;
  }
  std::cout << "////////////////////////////////////////////" << std::endl;

  std::cout << "red basket" << std::endl;
  for (int i = 0; i < red_basket.size(); i++)
  {
    std::cout << red_basket[i] << std::endl;
    std::cout << "____________________" << std::endl;
  }
  std::cout << "////////////////////////////////////////////" << std::endl;

  std::cout << "blue basket" << std::endl;
  for (int i = 0; i < blue_basket.size(); i++)
  {
    std::cout << blue_basket[i] << std::endl;
    std::cout << "____________________" << std::endl;
  }
  std::cout << "////////////////////////////////////////////" << std::endl;

  std::cout << "purple basket" << std::endl;
  for (int i = 0; i < purple_basket.size(); i++)
  {
    std::cout << purple_basket[i] << std::endl;
    std::cout << "____________________" << std::endl;
  }
  std::cout << "////////////////////////////////////////////" << std::endl;

  return;
}

////////////////////////////////////////////////////////////////////////////////

void
cw1::add_collision()
{
  // add the plate 
  add_plate();

  // add the cubes
  add_cube(red_cube, "red_cube");
  add_cube(blue_cube, "blue_cube");
  add_cube(purple_cube, "purple_cube");

  // add the baskets
  add_basket(red_basket, "red_basket");
  add_basket(blue_basket, "blue_basket");
  add_basket(purple_basket, "purple_basket");

  ROS_INFO("Collisions added successfully");


  return;
}

////////////////////////////////////////////////////////////////////////////////

void
cw1::add_plate()
{
  // add the collision in the RViz
  geometry_msgs::Point plate_position;
  plate_position.x = 0;
  plate_position.x = 0;
  plate_position.x = 0;

  geometry_msgs::Vector3 plate_dimension;
  plate_dimension.x = 5;
  plate_dimension.y = 5;
  plate_dimension.z = 0.08;

  geometry_msgs::Quaternion plate_orientation;
  plate_orientation.w = 1;
  plate_orientation.x = 0;
  plate_orientation.y = 0;
  plate_orientation.z = 0;

  addCollisionObject(plate_name,plate_position,plate_dimension,plate_orientation);

  // add additional basket
  std::string add_bas_name = "add_bas";
  geometry_msgs::Pose add_basket;
  geometry_msgs::Vector3 add_basket_dimen;
  add_basket.position.x = 0.6;
  add_basket.position.y = 0.6;
  add_basket.position.z = 0.07;
  add_basket.orientation.w = 1;
  add_basket.orientation.x = 0;
  add_basket.orientation.y = 0;
  add_basket.orientation.z = 0;
  add_basket_dimen.x = 5;
  add_basket_dimen.y = 0.01;
  add_basket_dimen.z = 5;
  // addCollisionObject(add_bas_name,add_basket.position,add_basket_dimen,add_basket.orientation);

  add_bas_name = "add_bas_";
  add_basket.position.y = -0.6;
  // addCollisionObject(add_bas_name,add_basket.position,add_basket_dimen,add_basket.orientation);

  return;
}

////////////////////////////////////////////////////////////////////////////////

void
cw1::add_cube(std::vector<geometry_msgs::Point> in_vec, std::string name)
{
  if (in_vec.size()>0)
  {
    geometry_msgs::Vector3 cube_dimen;
    geometry_msgs::Quaternion cube_orien;
    geometry_msgs::Point cube_pos;

    cube_dimen.x = 0.045;
    cube_dimen.y = 0.045;
    cube_dimen.z = 0.05;

    cube_orien.w = 1;
    cube_orien.x = 0;
    cube_orien.y = 0;
    cube_orien.z = 0;

    for (int i=0; i<in_vec.size(); i++)
    {
      std::string cube_name = name;
      cube_name = cube_name + std::to_string(i + 1);
      cube_pos = in_vec[i];
      addCollisionObject(cube_name,cube_pos,cube_dimen,cube_orien);
    }
  }

  return;
}

////////////////////////////////////////////////////////////////////////////////

void
cw1::add_basket(std::vector<geometry_msgs::Point> in_vec, std::string name)
{
  if (in_vec.size()>0)
  {
    geometry_msgs::Vector3 basket_dimen;
    geometry_msgs::Quaternion basket_orien;
    geometry_msgs::Point basket_pos;

    basket_dimen.x = 0.12;
    basket_dimen.y = 0.12;
    basket_dimen.z = 0.16;

    basket_orien.w = 1;
    basket_orien.x = 0;
    basket_orien.y = 0;
    basket_orien.z = 0;

    for (int i=0; i<in_vec.size(); i++)
    {
      std::string basket_name = name;
      basket_name = basket_name + std::to_string(i + 1);
      basket_pos = in_vec[i];
      addCollisionObject(basket_name,basket_pos,basket_dimen,basket_orien);
    }
  }

  return;
}

////////////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////////

void
cw1::pick_and_place (std::vector<geometry_msgs::Point> cube, 
                  std::vector<geometry_msgs::Point> basket,
                  std::string cube_name)
{
  if (cube.size()>0 && basket.size()>0)
  {
    geometry_msgs::Pose tar_cube, tar_basket;

    tar_basket.position = basket[0];
    tar_basket.position.z =  0.35;
    tar_basket.orientation.x = 0.9239;
    tar_basket.orientation.y = -0.3827;
    tar_basket.orientation.z = 0;
    tar_basket.orientation.w = 0;

    for (int i=0; i<cube.size(); i++)
    {
      std::string name = cube_name + std::to_string(i+1);
      tar_cube.position = cube[i];
      // tar_cube.position.z = 0.155;

      // tar_cube.orientation = tar_basket.orientation;
      tar_cube.orientation.x = 0.9239;
      tar_cube.orientation.y = -0.3827;
      tar_cube.orientation.z = 0;
      tar_cube.orientation.w = 0;

      moveGripper(1);
      tar_cube.position.z = 0.175;
      moveArm(tar_cube);
      tar_cube.position.z = 0.155;
      moveArm(tar_cube);
      removeCollisionObject(name);
      moveGripper(0.01);
      tar_cube.position.z = 0.2;
      // moveArm(tar_cube);
      moveArm(tar_basket);
      moveGripper(1);

      ROS_INFO("Pick and place finished");

    }

  }

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

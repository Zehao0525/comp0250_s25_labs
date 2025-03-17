/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw2_team_<your_team_number> package */

#include "cw2_class.h" // change to your team name here!

///////////////////////////////////////////////////////////////////////////////

cw2::cw2(ros::NodeHandle nh)
{
  /* class constructor */

  nh_ = nh;
  publish_cloud_ = false;

  // advertise solutions for coursework tasks
  t1_service_  = nh_.advertiseService("/task1_start", 
    &cw2::t1_callback, this);
  t2_service_  = nh_.advertiseService("/task2_start", 
    &cw2::t2_callback, this);
  t3_service_  = nh_.advertiseService("/task3_start",
    &cw2::t3_callback, this);


  point_cloud_sub_ = nh_.subscribe("/r200/camera/depth_registered/points", 1, &cw2::point_cloud_callback, this);
  // point_cloud_sub_ = nh_.subscribe("/camera/depth/points", 1, &cw2::point_cloud_callback, this);

  point_cloud_pub_test_ = nh_.advertise<sensor_msgs::PointCloud2>("transformed_cloud", 1);

  ROS_INFO("cw2 class initialised");
}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::t1_callback(cw2_world_spawner::Task1Service::Request &request,
  cw2_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */

  ROS_INFO("The coursework solving callback for task 1 has been triggered");

  geometry_msgs::PointStamped object_point = request.object_point;
  geometry_msgs::PointStamped goal_point = request.goal_point;
  std::string shape_type = request.shape_type;

  ROS_INFO("Object Point: [x: %f, y: %f, z: %f]", object_point.point.x, object_point.point.y, object_point.point.z);
  ROS_INFO("Goal Point: [x: %f, y: %f, z: %f]", goal_point.point.x, goal_point.point.y, goal_point.point.z);
  ROS_INFO("Shape Type: %s", shape_type.c_str());


  Init_Pose target_pose;
  target_pose.position = object_point.point;
  target_pose.position.z = 0.2;

  if (shape_type == "cross"){ target_pose.position.x += 0.05; }

  else if (shape_type == "nought") { target_pose.position.y += 0.08; }

  set_constraint();
  bool mvstate = move_arm(target_pose);
  ROS_INFO("Move state: %d", mvstate);

  // pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_cloud = convertToPCL(latest_cloud);

  // publishPointCloud(PCL_cloud);
  
  Init_Pose goal_pose;
  goal_pose.position = goal_point.point;
  goal_pose.position.z = 0.2;
  pick_and_place(shape_type, target_pose.position,  goal_pose.position);


  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::t2_callback(cw2_world_spawner::Task2Service::Request &request,
  cw2_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */
  set_constraint();

  // decoder
  std::vector<geometry_msgs::Point> ref_object_points;
  ROS_INFO("The coursework solving callback for task 2 has been triggered");

  for (const auto& point : request.ref_object_points) {
    ROS_INFO("Reference Object Point: [x: %f, y: %f, z: %f]", point.point.x, point.point.y, point.point.z);
    ref_object_points.push_back(point.point);
  }

  std::vector<DetectedObject> detected_objects;
  for (const auto& point : request.ref_object_points) {
    geometry_msgs::PointStamped reference_point = point;

    Init_Pose target_pose;
    target_pose.position = reference_point.point;
    target_pose.position.z = 0.5;
    bool mvstate = move_arm(target_pose);
    ROS_INFO("Move state: %d", mvstate);
  
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PCL_cloud = convertToPCL(latest_cloud, tf_listener_);
  
    ///save the point cloud to a file
    // pcl::io::savePCDFileASCII("mystery_point.pcd", *PCL_cloud);
  
    DetectedObject detected_object;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr obj_cloud_ptr;
    detect_objects(PCL_cloud, detected_object, obj_cloud_ptr);
    detected_objects.push_back(detected_object);
  }




  
  geometry_msgs::PointStamped mystery_point = request.mystery_object_point;
  // end decoder


  Init_Pose target_pose;
  target_pose.position = mystery_point.point;
  target_pose.position.z = 0.5;
  bool mvstate = move_arm(target_pose);
  ROS_INFO("Move state: %d", mvstate);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PCL_cloud = convertToPCL(latest_cloud, tf_listener_);

  ///save the point cloud to a file
  // pcl::io::savePCDFileASCII("mystery_point.pcd", *PCL_cloud);

  DetectedObject detected_object;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr obj_cloud_ptr;
  detect_objects(PCL_cloud, detected_object, obj_cloud_ptr);

  size_t match_idx = 0;
  // Match the mystery object to the reference object
  for (size_t idx = 0; idx < detected_objects.size(); ++idx) {
    if (detected_object.type == detected_objects[idx].type) {
        ROS_INFO("Matched object at index: %zu", idx);
        match_idx = idx;
        break;
    }
}

  response.mystery_object_num = match_idx;

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::t3_callback(cw2_world_spawner::Task3Service::Request &request,
  cw2_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");
  // reset_arm();
  set_constraint();


  combined_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

  float TOL = 0.2;
  float PLATFORM_X = 1.3 + TOL;
  float PLATFORM_Y = 1.1;


  for (float i = 0.1; i < PLATFORM_X; i += 0.3) {
    for (float j = 0.1; j < PLATFORM_Y; j += 0.3) {
      
      geometry_msgs::Point goal_point;
      goal_point.x = -0.5 * PLATFORM_X + i;
      goal_point.y = -0.5 * PLATFORM_Y + j; // half of the platform size
      goal_point.z = 0.5;

      if((goal_point.x < 0.2 && goal_point.x > -0.2)&&(goal_point.y < 0.25 && goal_point.y > -0.25)){
        continue;
      }

      Init_Pose target_pose;
      target_pose.position = goal_point;
      // target_pose.position.z = 0.5;
      bool mvstate = move_arm(target_pose);
      ROS_INFO("Move to : %f, %f", goal_point.x, goal_point.y);

      
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PCL_cloud = convertToPCL(latest_cloud, tf_listener_, "world");

      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::fromROSMsg(*latest_cloud, *pcl_cloud);
      pcl::io::savePCDFileASCII("test", *pcl_cloud);

      // pcl::io::savePCDFileASCII("test", *latest_cloud);

      
      if (combined_cloud == nullptr) {
          combined_cloud = PCL_cloud;
      } else {
          *combined_cloud += *PCL_cloud;
      }

      ROS_INFO("Cloud size: %zu", combined_cloud->points.size());
    }
  }

  ROS_INFO("Total cloud size: %zu", combined_cloud->points.size());
  std::string pcd_filename1 = "task3_scan_result1.pcd";
  // pcl::io::savePCDFile(pcd_filename1, *combined_cloud);
  pcl::io::savePCDFileASCII(pcd_filename1, *combined_cloud);
  ROS_INFO("Point cloud saved to %s", pcd_filename1.c_str());
  


  // 1. 点云下采样 - 使用VoxelGrid滤波器减少点数量
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
  vg.setInputCloud(combined_cloud);
  vg.setLeafSize(0.002f, 0.002f, 0.002f); // 5mm体素大小
  vg.filter(*cloud_filtered);
  ROS_INFO("Cloud size after voxel grid filtering: %zu", cloud_filtered->points.size());
  
  // // 2. 去除离群点 - 使用Statistical Outlier Removal
  // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered_2(new pcl::PointCloud<pcl::PointXYZRGBA>);
  // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
  // sor.setInputCloud(cloud_filtered);
  // sor.setMeanK(50);       // 平均计算的邻居数量
  // sor.setStddevMulThresh(1.0); // 标准差的乘数阈值
  // sor.filter(*cloud_filtered_2);
  // ROS_INFO("Cloud size after outlier removal: %zu", cloud_filtered_2->points.size());
  
  // // 3. 应用PassThrough滤波器，只保留z轴在一定范围内的点（例如，去除桌面和过高的点）
  // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered_3(new pcl::PointCloud<pcl::PointXYZRGBA>);
  // pcl::PassThrough<pcl::PointXYZRGBA> pass;
  // pass.setInputCloud(cloud_filtered_2);
  // pass.setFilterFieldName("z");
  // pass.setFilterLimits(0.02, 0.45); // 保留z轴在1cm到30cm之间的点
  // pass.filter(*cloud_filtered_3);
  // ROS_INFO("Cloud size after pass through filtering: %zu", cloud_filtered_3->points.size());
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered_3 = cloud_filtered;

  // 4. 保存最终的点云为PCD文件
  std::string pcd_filename = "task3_scan_result.pcd";
  pcl::io::savePCDFileASCII(pcd_filename, *cloud_filtered_3);
  ROS_INFO("Point cloud saved to %s", pcd_filename.c_str());
  
  std::vector<DetectedObject> detected_objects;
  detect_objects(cloud_filtered_3, detected_objects);
  ROS_INFO("====================================  Detected Objects  ====================================");
  for (const auto& obj : detected_objects) {
    ROS_INFO("Detected object: %s", obj.type.c_str());
    ROS_INFO("Position: [x: %f, y: %f, z: %f]", obj.position.x, obj.position.y, obj.position.z);
    ROS_INFO("Color: [r: %f, g: %f, b: %f]", obj.r, obj.g, obj.b);
  }







  ///response
  response.total_num_shapes = detected_objects.size() - 1; // exclude the basket
  DetectedObject picked_object, candidate_cross, candidate_nought, candidate_basket;
  int cross_count = 0;
  int nought_count = 0;
  for (const auto& obj : detected_objects) {
    if (obj.type == "cross") {
      cross_count = cross_count + 1;
      candidate_cross = obj;
    } else if (obj.type == "nought") {
      nought_count = nought_count + 1;
      candidate_nought = obj;
    }
    else if (obj.type == "basket") {
      candidate_basket = obj;
    }
  }

  if (cross_count > nought_count) {
    response.num_most_common_shape = cross_count;
    ROS_INFO("Most common shape: cross");
    ROS_INFO("Cross count: %d", cross_count);
    picked_object = candidate_cross;
  } else if (cross_count == nought_count) {
    response.num_most_common_shape = 0;
    ROS_INFO("Most common shape: same");
    ROS_INFO("Cross count: %d", cross_count);
    picked_object = candidate_cross;
  }
  else {
    response.num_most_common_shape = nought_count;
    ROS_INFO("Most common shape: nought");
    ROS_INFO("Nought count: %d", nought_count);
    picked_object = candidate_nought;
  }
  //// excute the pick and place

  if (picked_object.type == "cross") { picked_object.position.x += 0.05; }
  else if (picked_object.type == "nought") { picked_object.position.y += 0.08; }


  pick_and_place(picked_object.type, picked_object.position, candidate_basket.position); 


  ROS_INFO("Task 3 complete");

  return true;
}



bool cw2::move_arm(geometry_msgs::Pose& target_pose) {
  ROS_INFO("Setting pose target.");
  arm_group_.setPoseTarget(target_pose);
  // arm_group_.setPlannerId("RRTstar");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool plan_success = false;
  bool exec_success = false;

  int attempts = 0;
  const int max_attempts = 10;

  while ((!plan_success || !exec_success) && attempts < max_attempts) {
    ROS_INFO("Planning attempt %d...", attempts + 1);
    plan_success = (arm_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    attempts++;
    if (!plan_success) {
      ROS_INFO("Execution attempt %d failed", attempts);
      geometry_msgs::Pose temporary_pose = arm_group_.getCurrentPose().pose;
      temporary_pose.position.z -= 0.05;
      move_arm(temporary_pose);
      continue;
    }
    
    auto exec_result = arm_group_.execute(my_plan);
    exec_success = exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    // We wil take time_outs, cause otherwise it will never fcking reach the goals
    exec_success = exec_success || (exec_result == moveit::planning_interface::MoveItErrorCode::TIMED_OUT);
    
    if(exec_success) {
      break;
    }


  }
  
  if (!plan_success || !exec_success) {
    ROS_WARN("Plan failed");
  }
  
  return true;
}



void cw2::pick_and_place(const std::string& obj_name, const geometry_msgs::Point& obj_loc, const geometry_msgs::Point& goal_loc) {
ROS_INFO("picking up and placing: %s", obj_name.c_str());

// We specify waypoints for the arm trajectory
// geometry_msgs::Pose lift_pos, goal_pos;

// Init_Pose target_pose;
// // We specify the location the arm need to reach to pick up the object
// target_pose.position.x = obj_loc.x;
// target_pose.position.y = obj_loc.y;
// //target_pos.position.z = obj_loc.z + 0.125; //0.125 is the tested distance for the end effector to grab the cube firmly
// target_pose.position.z = 0.04 + 0.1; //0.03 is the hard coded position of the cubes


// // We define the pose above the goal for he arm to move to
// goal_pos = target_pose;
// goal_pos.position.x = goal_loc.x;
// goal_pos.position.y = goal_loc.y;
// goal_pos.position.z = 0.05 + 0.1;  // 0.1(cup height) + 0.125(height from edn effector to bottom of cube) + 0.075(leeway)


Init_Pose target_pos_down;
Init_Pose target_pos_up;
Init_Pose target_pos_down2;
Init_Pose target_pos_up2;

///////////////////////////////////////////
// initial position
///////////////////////////////////////////
bool opgrip_success = move_gripper(1.0);

target_pos_down.position = obj_loc;
target_pos_down.position.z = 0.04 + 0.1;
bool move_down_success = move_arm(target_pos_down);


bool close_gripper_success = move_gripper(0.0);


target_pos_up.position = obj_loc;
target_pos_up.position.z = 0.5;
bool move_up_success = move_arm(target_pos_up);

///////////////////////////////////////////
// Move to goal position
///////////////////////////////////////////

target_pos_down2.position = goal_loc;
target_pos_down2.position.z = 0.35;
bool move_goal_success = move_arm(target_pos_down2);


///////////////////////////////////////////
// Place the object
///////////////////////////////////////////


target_pos_down2.position = goal_loc;
target_pos_down2.position.z = 0.2;
bool move_down_success2 = move_arm(target_pos_down2);

bool open_gripper_success = move_gripper(1.0);

target_pos_up2.position = goal_loc;
target_pos_up2.position.z = 0.5;
bool move_up_success2 = move_arm(target_pos_up2);

}


void cw2::set_constraint() {
  joint_constraints_[0].joint_name = "panda_joint1";
  joint_constraints_[0].position = 0.0;
  joint_constraints_[0].tolerance_above = M_PI * 0.45;
  joint_constraints_[0].tolerance_below = M_PI * 0.45;
  joint_constraints_[0].weight = 1.0;

  joint_constraints_[1].joint_name = "panda_joint2";
  joint_constraints_[1].position = 0.4;
  joint_constraints_[1].tolerance_above = M_PI * 0.8;
  joint_constraints_[1].tolerance_below = M_PI * 0.8;
  joint_constraints_[1].weight = 1.0;

  joint_constraints_[2].joint_name = "panda_joint3";
  joint_constraints_[2].position = 0.0;
  joint_constraints_[2].tolerance_above = M_PI * 0.25;
  joint_constraints_[2].tolerance_below = M_PI * 0.25;
  joint_constraints_[2].weight = 1.0;

  joint_constraints_[3].joint_name = "panda_joint4";
  joint_constraints_[3].position = -2.13;
  joint_constraints_[3].tolerance_above = M_PI * 0.5;
  joint_constraints_[3].tolerance_below = M_PI * 0.5;
  joint_constraints_[3].weight = 1.0;

  joint_constraints_[4].joint_name = "panda_joint5";
  joint_constraints_[4].position = 0.0;
  joint_constraints_[4].tolerance_above = M_PI * 0.3;
  joint_constraints_[4].tolerance_below = M_PI * 0.3;
  joint_constraints_[4].weight = 1.0;

  joint_constraints_[5].joint_name = "panda_joint6";
  joint_constraints_[5].position = 2.33;
  joint_constraints_[5].tolerance_above = M_PI * 0.25;
  joint_constraints_[5].tolerance_below = M_PI * 0.8;
  joint_constraints_[5].weight = 1.0;

  joint_constraints_[6].joint_name = "panda_joint7";
  joint_constraints_[6].position = 0.785;
  joint_constraints_[6].tolerance_above = M_PI * 0.25;
  joint_constraints_[6].tolerance_below = M_PI * 0.25;
  joint_constraints_[6].weight = 1.0;

  // constraints.joint_constraints.push_back(joint_constraints_[0]);
  constraints_.joint_constraints.push_back(joint_constraints_[1]);
  constraints_.joint_constraints.push_back(joint_constraints_[2]);
  constraints_.joint_constraints.push_back(joint_constraints_[3]);
  constraints_.joint_constraints.push_back(joint_constraints_[4]);
  // constraints.joint_constraints.push_back(joint_constraints_[5]);
  // constraints.joint_constraints.push_back(joint_constraints_[6]);
  arm_group_.setPathConstraints(constraints_);
}

bool cw2::move_gripper(float width) {
  // safety checks in case width exceeds safe values
  if (width > gripper_open_) {
    width = gripper_open_;
  }
  if (width < gripper_closed_) {
    width = gripper_closed_;
  }

  // calculate the joint targets as half each of the requested distance
  double each_joint = width / 2.0;

  // create a vector to hold the joint target for each joint
  std::vector<double> gripper_joint_targets(2);
  gripper_joint_targets[0] = each_joint;
  gripper_joint_targets[1] = each_joint;

  // apply the joint target
  hand_group_.setJointValueTarget(gripper_joint_targets);

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

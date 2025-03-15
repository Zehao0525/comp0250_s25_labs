/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw2_team_<your_team_number> package */

#include <cw2_class.h> // change to your team name here!

///////////////////////////////////////////////////////////////////////////////

cw2::cw2(ros::NodeHandle nh)
{
  /* class constructor */

  nh_ = nh;

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

  ROS_INFO("The coursework solving callback for task 2 has been triggered");

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::t3_callback(cw2_world_spawner::Task3Service::Request &request,
  cw2_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");

  return true;
}

void cw2::point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg) {
  latest_cloud = cloud_input_msg;
  
}

bool cw2::move_arm(geometry_msgs::Pose& target_pose) {
  ROS_INFO("Setting pose target.");
  arm_group_.setPoseTarget(target_pose);
  // arm_group_.setPlannerId("RRTstar");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool plan_success = false;
  bool exec_success = false;

  int attempts = 0;
  const int max_attempts = 100;

  while ((!plan_success || !exec_success) && attempts < max_attempts) {
    ROS_INFO("Planning attempt %d...", attempts + 1);
    plan_success = (arm_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    attempts++;
    if (!plan_success) {
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

pcl::PointCloud<pcl::PointXYZ>::Ptr cw2::convertToPCL(sensor_msgs::PointCloud2ConstPtr cloud_msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

  // Transform the point cloud to the desired frame
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  try {
    tf_listener_.waitForTransform("world", cloud_msg->header.frame_id, ros::Time(0), ros::Duration(3.0));
    pcl_ros::transformPointCloud("world", *pcl_cloud, *transformed_cloud, tf_listener_);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("Transform error: %s", ex.what());
    return pcl_cloud; // Return the original cloud if transform fails
  }

  return transformed_cloud;
}

void cw2::publishPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud, output);
  output.header.frame_id = "world";
  point_cloud_pub_test_.publish(output);
  ROS_INFO("Published point cloud");
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

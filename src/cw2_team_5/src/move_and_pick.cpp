// File: move_and_pick.cpp
#include "cw2_class.h"
#include "external.h"
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>



bool cw2::move_arm(geometry_msgs::Pose& target_pose, bool use_cartesian, int recursion_depth) {
  ROS_INFO("Setting pose target.");
  arm_group_.setPoseTarget(target_pose);

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



// bool cw2::move_arm(geometry_msgs::Pose& target_pose, bool use_cartesian, int recursion_depth) {
//   // 递归深度限制，防止无限递归
//   const int max_recursion_depth = 3;
//   if (recursion_depth > max_recursion_depth) {
//     ROS_ERROR("Maximum recursion depth reached in move_arm. Aborting.");
//     return false;
//   }

//   ROS_INFO("Setting pose target (recursion depth: %d).", recursion_depth);
//   arm_group_.setPoseTarget(target_pose);
//   arm_group_.setPlanningTime(20.0);

//   moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//   bool plan_success = false;
//   bool exec_success = false;

//   int attempts = 0;
//   const int max_attempts = 4;

//   while ((!plan_success || !exec_success) && attempts < max_attempts) {
//     ROS_INFO("Planning attempt %d...", attempts + 1);
//     plan_success = (arm_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
//     attempts++;
//     if (!plan_success) {
//       ROS_WARN("Planning attempt %d failed.", attempts);
//       continue;
//     }

//     // 对轨迹进行时间参数化
//     // robot_trajectory::RobotTrajectory rt(arm_group_.getCurrentState()->getRobotModel(), "panda_arm");
//     // rt.setRobotTrajectoryMsg(*arm_group_.getCurrentState(), my_plan.trajectory_);
//     // trajectory_processing::IterativeParabolicTimeParameterization iptp;
//     // bool success = iptp.computeTimeStamps(rt);
//     // ROS_INFO("Computed time stamp %s", success ? "SUCCEEDED" : "FAILED");
//     // rt.getRobotTrajectoryMsg(my_plan.trajectory_);
//     // ///////////////////////////////////////////

//     // saveTrajectoryToCSV(rt, "trajectory.csv");



//     ////////////////////////////////////////////
//     ROS_INFO("Executing plan...");
//     auto exec_result = arm_group_.execute(my_plan);
//     exec_success = exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS;
//     // 考虑超时也算作一种成功，因为可能已经接近目标位置
//     if (exec_result == moveit::planning_interface::MoveItErrorCode::TIMED_OUT) {
//       ROS_WARN("Execution timed out, but continuing as this may be close enough.");
//       exec_success = true;
//     }
    
//     if(exec_success) {
//       ROS_INFO("Execution successful.");
//       break;
//     } else {
//       ROS_WARN("Execution failed with code: %d", exec_result);
//     }
//   }
  
//   if (!plan_success || !exec_success) {
//     ROS_WARN("All attempts failed. Trying intermediate waypoint approach.");
    
//     // 获取当前位置
//     Init_Pose current_pose;
//     current_pose = arm_group_.getCurrentPose().pose;
//     Init_Pose temp_pose;
    
//     // 计算中点位置
//     temp_pose.position.x = (current_pose.position.x + target_pose.position.x) / 2.0;
//     temp_pose.position.y = (current_pose.position.y + target_pose.position.y) / 2.0;
//     temp_pose.position.z = (current_pose.position.z + target_pose.position.z) / 2.0;
    
//     // 使用球面线性插值(SLERP)计算姿态中点
//     tf2::Quaternion q1, q2, q_result;
//     tf2::convert(current_pose.orientation, q1);
//     tf2::convert(target_pose.orientation, q2);
    
//     // 使用0.5作为插值参数t，计算中间姿态
//     q_result = q1.slerp(q2, 0.5);
//     q_result.normalize();
//     tf2::convert(q_result, temp_pose.orientation);
    
//     ROS_INFO("Moving to intermediate waypoint.");
//     bool waypoint_success;

//     if (recursion_depth == max_recursion_depth - 1) {
//       ROS_WARN("Maximum recursion depth reached. Attempting final target with temporary waypoint.(REMOVE THE CONSTRAINT)");
//       // temp_pose.position.x -= 0.05;
//       // temp_pose.position.y -= 0.05;
//       clear_constraint();
//       // set_height_constraint(0.1);
//       waypoint_success = move_arm(temp_pose, use_cartesian, recursion_depth + 1);
//       clear_constraint();
//       set_constraint();

//     } else {
//       waypoint_success = move_arm(temp_pose, use_cartesian, recursion_depth + 1);
//     }

//     if (waypoint_success) {
//       ROS_INFO("Successfully moved to intermediate waypoint. Attempting final target again.");
//       return move_arm(target_pose, use_cartesian, recursion_depth + 1);
//     } else {
//       ROS_ERROR("Failed to move to intermediate waypoint.");
//       return false;
//     }
//   }
  
//   return true;
// }

void cw2::pick_and_place(const std::string& obj_name, const geometry_msgs::Point& obj_loc, const geometry_msgs::Point& goal_loc) {
  Init_Pose obj_point;
  obj_point.position = obj_loc;
  pick_and_place(obj_name, obj_point, goal_loc);
}

void cw2::pick_and_place(const std::string& obj_name, const geometry_msgs::Pose& obj_loc, const geometry_msgs::Point& goal_loc) {
    ROS_INFO("picking up and placing: %s", obj_name.c_str());
    
    
    Init_Pose target_pos_down;
    Init_Pose target_pos_up;
    Init_Pose target_pos_down2;
    Init_Pose target_pos_up2;
    
    ///////////////////////////////////////////
    // initial position
    ///////////////////////////////////////////
    bool opgrip_success = move_gripper(1.0);
    
    target_pos_down = obj_loc;
    target_pos_down.position.z = 0.04 + 0.1;
    bool move_down_success = move_arm(target_pos_down, true);
    
    
    bool close_gripper_success = move_gripper(0.0);
    
    
    target_pos_up = obj_loc;
    target_pos_up.position.z = 0.5;
    bool move_up_success = move_arm(target_pos_up, true);
    
    ///////////////////////////////////////////
    // Move to goal position
    ///////////////////////////////////////////
    
    target_pos_down2.position = goal_loc;
    target_pos_down2.position.z = 0.5;
    // clear_constraint();
    bool move_goal_success = move_arm(target_pos_down2);
    
    
    ///////////////////////////////////////////
    // Place the object
    ///////////////////////////////////////////
    
    
    target_pos_down2.position = goal_loc;
    target_pos_down2.position.z = 0.25;
    bool move_down_success2 = move_arm(target_pos_down2, true);
    
    bool open_gripper_success = move_gripper(1.0);
    
    target_pos_up2.position = goal_loc;
    target_pos_up2.position.z = 0.5;
    bool move_up_success2 = move_arm(target_pos_up2, true);
       }
      

void cw2::set_height_constraint(double min_height = 0.1) {
  // 清除所有现有的约束
  clear_constraint();
  
  // 创建位置约束
  moveit_msgs::PositionConstraint position_constraint;
  position_constraint.header.frame_id = "world"; // 或者您的基准坐标系
  position_constraint.link_name = "panda_hand"; // 末端执行器链接名
  
  // 创建一个足够大的盒子，只限制z方向的下边界
  shape_msgs::SolidPrimitive box;
  box.type = shape_msgs::SolidPrimitive::BOX;
  box.dimensions.resize(3);
  box.dimensions[0] = 2.0; // x方向足够大
  box.dimensions[1] = 2.0; // y方向足够大
  box.dimensions[2] = 2.0; // z方向高度
  
  // 设置盒子位置，使其底部边界在min_height处
  geometry_msgs::Pose box_pose;
  box_pose.position.x = 0.0;
  box_pose.position.y = 0.0;
  box_pose.position.z = min_height + box.dimensions[2]/2.0;
  box_pose.orientation.w = 1.0;
  
  position_constraint.constraint_region.primitives.push_back(box);
  position_constraint.constraint_region.primitive_poses.push_back(box_pose);
  position_constraint.weight = 1.0;
  
  // 将位置约束添加到约束集合
  constraints_.position_constraints.push_back(position_constraint);
  
  // 应用约束
  arm_group_.setPathConstraints(constraints_);
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
  
  void cw2::clear_constraint() {
    constraints_.joint_constraints.clear();
    constraints_.position_constraints.clear();  
    arm_group_.clearPathConstraints();
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
  

inline double random_double(double min, double max) {
  std::random_device rd;
  std::mt19937 gen(rd());  // 随机种子
  std::uniform_real_distribution<> dist(min, max);  // 均匀分布
  double r = dist(gen);
  return r;
}

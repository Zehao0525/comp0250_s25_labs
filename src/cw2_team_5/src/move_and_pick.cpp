#include "cw2_class.h"
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

bool cw2::move_arm(geometry_msgs::Pose& target_pose, bool use_cartesian) {
  ROS_INFO("开始移动机械臂到目标位置");
  
  const int max_attempts = 10;
  arm_group_.setPlanningTime(10.0);
  // 1. 尝试笛卡尔路径规划（如果要求）
  if (use_cartesian) {
    ROS_INFO("使用笛卡尔路径规划");
    
    for (int attempt = 0; attempt < max_attempts; attempt++) {
      // 创建路径点
      std::vector<geometry_msgs::Pose> waypoints;
      waypoints.push_back(arm_group_.getCurrentPose().pose);
      waypoints.push_back(target_pose);
      
      // 计算笛卡尔路径
      moveit_msgs::RobotTrajectory trajectory;
      double fraction = arm_group_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
      
      if (fraction < 0.95) {
        ROS_WARN("仅计算了 %.2f%% 的路径，切换到普通规划", fraction * 100.0);
        break; // 退出循环，进入普通规划
      }
      
      // 添加时间参数化处理
      robot_trajectory::RobotTrajectory rt(arm_group_.getCurrentState()->getRobotModel(), arm_group_.getName());
      rt.setRobotTrajectoryMsg(*arm_group_.getCurrentState(), trajectory);
      
      // 降低执行速度
      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      if (!iptp.computeTimeStamps(rt, 0.3, 0.3)) {
        ROS_WARN("时间参数化失败，尝试 %d", attempt + 1);
        continue;
      }
      
      // 执行轨迹
      rt.getRobotTrajectoryMsg(trajectory);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      my_plan.trajectory_ = trajectory;
      auto exec_result = arm_group_.execute(my_plan);
      
      if (exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS || 
          exec_result == moveit::planning_interface::MoveItErrorCode::TIMED_OUT) {
        return true;
      }
      
      ROS_WARN("笛卡尔执行失败，尝试 %d。错误代码: %d", attempt + 1, exec_result.val);
      ros::Duration(0.5).sleep();
    }
    
    ROS_WARN("所有笛卡尔尝试失败，切换到普通规划");
    use_cartesian = false; // 切换到普通规划
    // 这里不使用递归调用，而是继续执行下面的普通规划代码
  }
  
  // 2. 普通规划部分
  arm_group_.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  for (int attempt = 0; attempt < max_attempts; attempt++) {
    ROS_INFO("普通规划尝试 %d...", attempt + 1);
    
    // 尝试规划
    bool plan_success = (arm_group_.plan(my_plan) == 
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    // 规划失败处理
    if (!plan_success) {
      if (attempt >= max_attempts - 1) {
        ROS_WARN("在 %d 次尝试后规划失败", max_attempts);
        return false;
      }
      
      // 尝试调整高度
      geometry_msgs::Pose adjusted_pose = target_pose;
      adjusted_pose.position.z += std::rand() % 10 * 0.01;

      
      ROS_INFO("规划失败。尝试调整高度为 %.3f", adjusted_pose.position.z);
      arm_group_.setPoseTarget(adjusted_pose);
      
      if (arm_group_.plan(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        continue; // 调整高度后规划仍失败，继续下一次尝试
      }
      
      // 执行调整高度的移动
      if (arm_group_.execute(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        continue; // 调整高度的移动执行失败，继续下一次尝试
      }
      
      // 成功移动到调整位置，重新尝试原始目标
      ROS_INFO("成功移动到调整位置，现在尝试原始目标位置");
      arm_group_.setPoseTarget(target_pose);
      if (arm_group_.plan(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        continue; // 重新规划原始目标失败，继续下一次尝试
      }
    }
    
    // 执行计划
    auto exec_result = arm_group_.execute(my_plan);
    if (exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS || 
        exec_result == moveit::planning_interface::MoveItErrorCode::TIMED_OUT) {
      return true;
    }
    
    ROS_WARN("计划执行失败，尝试 %d。错误代码: %d", attempt + 1, exec_result.val);
    
    // 执行失败后尝试恢复位置
    if (attempt < max_attempts - 1) {
      geometry_msgs::Pose recovery_pose = target_pose;
      // recovery_pose.position.z += 0.08 + 0.04 * attempt;
      recovery_pose.position.z += std::rand() % 10 * 0.01;
      
      ROS_INFO("尝试恢复位置 z=%.3f", recovery_pose.position.z);
      arm_group_.setPoseTarget(recovery_pose);
      
      if (arm_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS && 
          arm_group_.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_INFO("成功移动到恢复位置");
      }
    }
    
    ros::Duration(0.5).sleep();
  }
  
  ROS_WARN("所有规划和执行尝试都失败了");
  return false;
}

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
  
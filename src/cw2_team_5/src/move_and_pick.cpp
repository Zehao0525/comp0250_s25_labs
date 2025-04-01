// File: move_and_pick.cpp
#include "cw2_class.h"
#include "external.h"
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>


bool cw2::move_arm(geometry_msgs::Pose& target_pose, bool use_cartesian, int recursion_depth) {
  // 递归深度限制，防止无限递归
  const int max_recursion_depth = 3;
  if (recursion_depth > max_recursion_depth) {
    ROS_ERROR("Maximum recursion depth reached in move_arm. Aborting.");
    return false;
  }

  ROS_INFO("Setting pose target (recursion depth: %d).", recursion_depth);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool plan_success = false;
  bool exec_success = false;

  // 使用笛卡尔路径规划
  if (use_cartesian) {
    ROS_INFO("Using Cartesian path planning");
    
    // 获取当前末端执行器位置
    geometry_msgs::Pose start_pose = arm_group_.getCurrentPose().pose;
    
    // 创建一个包含起点和终点的路径点数组
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(start_pose);
    waypoints.push_back(target_pose);
    
    // 计算笛卡尔路径
    moveit_msgs::RobotTrajectory trajectory;

    double fraction = arm_group_.computeCartesianPath(waypoints, 
                                                    0.01,    // 路径点之间的最大步长(eef_step)
                                                    0.0,     // 跳跃阈值
                                                    trajectory);
    
    ROS_INFO("Cartesian path planning (%.2f%% achieved)", fraction * 100.0);

    if (fraction > 0.0) {
      // 构建完整的计划
      my_plan.trajectory_ = trajectory;
      plan_success = true;

      //////////////////////////////////////////////

      // 优化轨迹速度
      robot_trajectory::RobotTrajectory rt(arm_group_.getCurrentState()->getRobotModel(), "panda_arm");
      rt.setRobotTrajectoryMsg(*arm_group_.getCurrentState(), my_plan.trajectory_);

      // 应用速度和加速度缩放因子进行时间参数化
      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      double velocity_scaling_factor = 1.5;  // 速度缩放因子（提高速度）
      double acceleration_scaling_factor = 1;  // 加速度缩放因子
      iptp.computeTimeStamps(rt, velocity_scaling_factor, acceleration_scaling_factor);

      rt.getRobotTrajectoryMsg(my_plan.trajectory_);

      ///////////////////////////////////////////////


    // 检查优化后的轨迹
    ROS_INFO("checking trajectory timestamps...");
    checkTrajectoryTimestamps(my_plan.trajectory_);














      ///////////////////////////////////

      // 执行计划
      ROS_INFO("Executing Cartesian path plan...");

      auto exec_result = arm_group_.execute(my_plan);

      exec_success = exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS;

      // 考虑超时也算作一种成功
      if (exec_result == moveit::planning_interface::MoveItErrorCode::TIMED_OUT) {
        ROS_WARN("Execution timed out, but continuing as this may be close enough.");
        exec_success = true;
      }
    } else {
      ROS_WARN("Could not compute Cartesian path, falling back to joint space planning.");
      use_cartesian = false;  // 失败后尝试关节空间规划
    }
  }
  
  // 使用关节空间规划 (默认或笛卡尔规划失败后的回退)
  if (!use_cartesian || !plan_success || !exec_success) {
    arm_group_.setPoseTarget(target_pose);
    // arm_group_.setPlanningTime(20.0);
    
    int attempts = 0;
    const int max_attempts = 3;

    while ((!plan_success || !exec_success) && attempts < max_attempts) {
      ROS_INFO("Planning attempt %d...", attempts + 1);
      plan_success = (arm_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      
      attempts++;
      if (!plan_success) {
        ROS_WARN("Planning attempt %d failed.", attempts);
        continue;
      }

      /////////////////////////////////////////
      // 优化轨迹速度
      robot_trajectory::RobotTrajectory rt(arm_group_.getCurrentState()->getRobotModel(), "panda_arm");
      rt.setRobotTrajectoryMsg(*arm_group_.getCurrentState(), my_plan.trajectory_);

      // 应用速度和加速度缩放因子进行时间参数化
      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      double velocity_scaling_factor = 1.5;  // 速度缩放因子（提高速度）
      double acceleration_scaling_factor = 1;  // 加速度缩放因子
      iptp.computeTimeStamps(rt, velocity_scaling_factor, acceleration_scaling_factor);

      // 将优化后的轨迹应用回计划
      rt.getRobotTrajectoryMsg(my_plan.trajectory_);
      /////////////////////////////////////////


      ROS_INFO("Executing plan...");
      auto exec_result = arm_group_.execute(my_plan);
      exec_success = exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS;
      
      if (exec_result == moveit::planning_interface::MoveItErrorCode::TIMED_OUT) {
        ROS_WARN("Execution timed out, but continuing as this may be close enough.");
        exec_success = true;
      }
      
      if(exec_success) {
        ROS_INFO("Execution successful.");
        break;
      } else {
        ROS_WARN("Execution failed with code: %d", exec_result);
      }
    }
  }
  if (!plan_success || !exec_success) {
    ROS_WARN("All attempts failed. Trying intermediate waypoint approach.");
    
    // 获取当前位置
    Init_Pose current_pose;
    current_pose = arm_group_.getCurrentPose().pose;
    Init_Pose temp_pose;
    
    // 计算中点位置
    temp_pose.position.x = (current_pose.position.x + target_pose.position.x) / 2.0;
    temp_pose.position.y = (current_pose.position.y + target_pose.position.y) / 2.0;
    temp_pose.position.z = (current_pose.position.z + target_pose.position.z) / 2.0;
    
    // 使用球面线性插值(SLERP)计算姿态中点
    tf2::Quaternion q1, q2, q_result;
    tf2::convert(current_pose.orientation, q1);
    tf2::convert(target_pose.orientation, q2);
    
    // 使用0.5作为插值参数t，计算中间姿态
    q_result = q1.slerp(q2, 0.5);
    q_result.normalize();
    tf2::convert(q_result, temp_pose.orientation);
    
    ROS_INFO("Moving to intermediate waypoint.");
    bool waypoint_success;

    if (recursion_depth == max_recursion_depth - 1) {
      ROS_WARN("Maximum recursion depth reached. Attempting final target with temporary waypoint.(REMOVE THE CONSTRAINT)");
      reset_arm();
      waypoint_success = move_arm(temp_pose, use_cartesian, recursion_depth + 1);

    } else {
      waypoint_success = move_arm(temp_pose, use_cartesian, recursion_depth + 1);
    }

    if (waypoint_success) {
      ROS_INFO("Successfully moved to intermediate waypoint. Attempting final target again.");
      return move_arm(target_pose, use_cartesian, recursion_depth + 1);
    } else {
      ROS_ERROR("Failed to move to intermediate waypoint.");
      return false;
    }
  }
  
  return true;
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
    ///////////////////////////////////////////  这里应该移动到目标的正上方
    bool opgrip_success = move_gripper(1.0);
    


    clear_constraint();
    target_pos_down = obj_loc;
    target_pos_down.position.z = 0.04 + 0.1;
    bool move_down_success = move_arm(target_pos_down, true);
    
    
    bool close_gripper_success = move_gripper(0.0);
    
    
    target_pos_up = obj_loc;
    target_pos_up.position.z = 0.41;
    bool move_up_success = move_arm(target_pos_up, true);
    
    ///////////////////////////////////////////
    // Move to goal position
    ///////////////////////////////////////////

    // set_constraint();
    set_z_constraint(0.4, 1.2);
    target_pos_down2.position = goal_loc;
    target_pos_down2.position.z = 0.41;
    // clear_constraint();
    bool move_goal_success = move_arm(target_pos_down2);
    
    clear_constraint();
    
    ///////////////////////////////////////////
    // Place the object
    ///////////////////////////////////////////
    
    
    target_pos_down2.position = goal_loc;
    target_pos_down2.position.z = 0.25;
    bool move_down_success2 = move_arm(target_pos_down2, true);
    
    bool open_gripper_success = move_gripper(1.0);
    
    target_pos_up2.position = goal_loc;
    target_pos_up2.position.z = 0.41;
    bool move_up_success2 = move_arm(target_pos_up2, true);

    set_constraint();
       }
      



void cw2::set_constraint() {
    joint_constraints_[0].joint_name = "panda_joint1";
    joint_constraints_[0].position = 0.0;
    joint_constraints_[0].tolerance_above = M_PI * 0.8;
    joint_constraints_[0].tolerance_below = M_PI * 0.8;
    joint_constraints_[0].weight = 1.0;
  
    joint_constraints_[1].joint_name = "panda_joint2";
    joint_constraints_[1].position = 0.4;
    joint_constraints_[1].tolerance_above = M_PI * 0.8;
    joint_constraints_[1].tolerance_below = M_PI * 0.8;
    // joint_constraints_[1].tolerance_above = M_PI * 0.5;
    // joint_constraints_[1].tolerance_below = M_PI * 0.5;
    joint_constraints_[1].weight = 1.0;
  
    joint_constraints_[2].joint_name = "panda_joint3";
    joint_constraints_[2].position = 0.0;
    joint_constraints_[2].tolerance_above = M_PI * 1;
    joint_constraints_[2].tolerance_below = M_PI * 1;
    joint_constraints_[2].weight = 1.0;
  
    joint_constraints_[3].joint_name = "panda_joint4";
    joint_constraints_[3].position = -2.13;
    joint_constraints_[3].tolerance_above = M_PI * 0.5;
    joint_constraints_[3].tolerance_below = M_PI * 0.5;
    // joint_constraints_[3].tolerance_above = M_PI * 0.4;
    // joint_constraints_[3].tolerance_below = M_PI * 0.4;
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
    // constraints_.joint_constraints.push_back(joint_constraints_[1]);
    // constraints_.joint_constraints.push_back(joint_constraints_[2]);
    // constraints_.joint_constraints.push_back(joint_constraints_[3]);
    constraints_.joint_constraints.push_back(joint_constraints_[4]);        ////////5号关节限制运动
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
  



void cw2::adjustPoseByShapeAndRotation(geometry_msgs::Pose& target_pose, 
  const std::string& shape_type, 
  float rot_degree) {
// 旋转target_pose.orientation
tf2::Quaternion q_orig, q_rot, q_new;
tf2::convert(target_pose.orientation, q_orig);
q_rot.setRPY(0, 0, rot_degree * M_PI / 180); // 旋转角度转换为弧度
q_new = q_rot * q_orig;
q_new.normalize();
tf2::convert(q_new, target_pose.orientation);

// 根据物体旋转角度计算偏移量
float offset_distance = 0.0;
float offset_x = 0.0;
float offset_y = 0.0;
float rad_angle = rot_degree * M_PI / 180.0; // 角度转弧度

if (shape_type == "cross") {
offset_distance = 0.05;
// 根据旋转角度计算在旋转坐标系中的x、y分量
offset_x = offset_distance * cos(rad_angle);
offset_y = offset_distance * sin(rad_angle);

ROS_INFO("Cross offset: [x: %f, y: %f] (angle: %f deg)", offset_x, offset_y, rot_degree);

} else if (shape_type == "nought") {
offset_distance = 0.08;
// 对于nought形状，偏移方向是垂直于旋转方向的
// 相当于在旋转坐标系中的y轴方向上偏移
offset_x = -offset_distance * sin(rad_angle); // 注意这里是-sin
offset_y = offset_distance * cos(rad_angle);

ROS_INFO("Nought offset: [x: %f, y: %f] (angle: %f deg)", offset_x, offset_y, rot_degree);
}

// 应用计算出的偏移量
target_pose.position.x += offset_x;
target_pose.position.y += offset_y;
}



/**
 * 设置末端执行器Z轴高度的上下限制
 * @param min_height Z轴最小高度（米）
 * @param max_height Z轴最大高度（米）
 */
void cw2::set_z_constraint(double min_height, double max_height) {
  // 清除现有的约束（可选，取决于您是否想保留其他约束）
  moveit_msgs::Constraints temp_constraints = constraints_; // 保存当前约束
  constraints_.position_constraints.clear();  // 仅清除位置约束
  
  // 创建盒子形状来表示允许的区域
  shape_msgs::SolidPrimitive box;
  box.type = shape_msgs::SolidPrimitive::BOX;
  box.dimensions.resize(3);
  box.dimensions[0] = 2.0;  // x方向很大，不限制
  box.dimensions[1] = 2.0;  // y方向很大，不限制
  box.dimensions[2] = max_height - min_height;  // z方向限制范围
  
  // 设置盒子位置，中心在z=(min_height+max_height)/2处
  geometry_msgs::Pose box_pose;
  box_pose.position.x = 0.0;
  box_pose.position.y = 0.0;
  box_pose.position.z = (min_height + max_height) / 2.0;
  box_pose.orientation.w = 1.0;
  
  // 创建位置约束
  moveit_msgs::PositionConstraint pos_constraint;
  pos_constraint.header.frame_id = "world";  // 基准坐标系
  pos_constraint.link_name = "panda_link7";      // 末端执行器的链接名称
  
  // 设置约束区域
  pos_constraint.constraint_region.primitives.push_back(box);
  pos_constraint.constraint_region.primitive_poses.push_back(box_pose);
  pos_constraint.weight = 1.0;
  
  // 将位置约束添加到约束集合中
  constraints_.position_constraints.push_back(pos_constraint);
  
  // 恢复关节约束（如果有的话）
  constraints_.joint_constraints = temp_constraints.joint_constraints;
  
  // 应用所有约束
  arm_group_.setPathConstraints(constraints_);
  
  ROS_INFO("Set Z-axis constraint: min=%.2f, max=%.2f", min_height, max_height);
}



bool cw2::reset_arm(double min_height, double target_height)
{
  clear_constraint();
  // 获取当前机械臂位置
  geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();
  double current_z = current_pose.pose.position.z;
  
  // 检查当前高度是否低于最小值
  if (current_z < min_height) {
    ROS_INFO("Current arm height (%.3f) is below minimum (%.3f), adjusting to %.3f", 
             current_z, min_height, target_height);
    
    // 创建新的目标位置
    geometry_msgs::Pose target_pose = current_pose.pose;
    target_pose.position.z = target_height;
    
    // 设置目标位置
    arm_group_.setPoseTarget(target_pose);
    
    // 规划并执行移动
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (arm_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    if (success) {
      ROS_INFO("Planning successful, executing movement to safe height");
      arm_group_.execute(my_plan);
      return true;
    } else {
      ROS_WARN("Failed to plan movement to safe height");
      return false;
    }
  } else {
    // 如果高度已经安全，无需调整
    ROS_INFO("Arm height (%.3f) is already above minimum (%.3f), no adjustment needed", 
             current_z, min_height);
    return true;
  }

  set_z_constraint(0.4, 1.2); // 恢复Z轴约束
}

#include "cw2_class.h"
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

bool cw2::move_arm(geometry_msgs::Pose& target_pose, bool use_cartesian) {
  ROS_INFO("Setting pose target.");
  
  const int max_attempts = 5;  // 最大尝试次数（简化为3次）
  
  if (use_cartesian) {
    ROS_INFO("Using Cartesian Path for motion planning.");
    
    for (int attempt = 0; attempt < max_attempts; attempt++) {
      // 创建路径点
      std::vector<geometry_msgs::Pose> waypoints;
      waypoints.push_back(arm_group_.getCurrentPose().pose);
      waypoints.push_back(target_pose);
      
      // 计算笛卡尔路径
      moveit_msgs::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;  // 设置0以避免跳跃
      const double eef_step = 0.01;  // 每次步进 1cm
      double fraction = arm_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      
      if (fraction < 0.95) {
        ROS_WARN("Only %.2f%% of the path was computed. Fallback to regular planning.", fraction * 100.0);
        return move_arm(target_pose, false); // 回退到普通规划
      }
      
      // 添加时间参数化处理
      robot_trajectory::RobotTrajectory rt(arm_group_.getCurrentState()->getRobotModel(), arm_group_.getName());
      rt.setRobotTrajectoryMsg(*arm_group_.getCurrentState(), trajectory);
      
      // 使用时间参数化调整轨迹，降低执行速度
      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      bool success = iptp.computeTimeStamps(rt, 0.3, 0.3);  // 降低速度和加速度
      
      if (!success) {
        ROS_WARN("Time parameterization failed on attempt %d.", attempt + 1);
        if (attempt == max_attempts - 1) {
          return move_arm(target_pose, false);  // 最后一次尝试失败，回退到普通规划
        }
        continue;
      }
      
      // 将时间参数化的轨迹转回到plan中
      rt.getRobotTrajectoryMsg(trajectory);
      
      // 执行轨迹
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      my_plan.trajectory_ = trajectory;
      auto exec_result = arm_group_.execute(my_plan);
      
      if (exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS || 
          exec_result == moveit::planning_interface::MoveItErrorCode::TIMED_OUT) {
        return true;
      }
      
      ROS_WARN("Cartesian execution failed on attempt %d. Error code: %d", 
               attempt + 1, exec_result.val);
      
      // 短暂等待后重试
      ros::Duration(0.5).sleep();
    }
    
    // 所有笛卡尔尝试失败，回退到普通规划
    ROS_WARN("All Cartesian attempts failed. Switching to regular planning.");
    return move_arm(target_pose, false);
    
  } else {
    // 普通规划部分
    arm_group_.setPoseTarget(target_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
    for (int attempt = 0; attempt < max_attempts; attempt++) {
      ROS_INFO("Regular planning attempt %d...", attempt + 1);
      
      bool plan_success = (arm_group_.plan(my_plan) == 
                          moveit::planning_interface::MoveItErrorCode::SUCCESS);
      
      if (!plan_success) {
        if (attempt < max_attempts - 1) {
          // 创建调整后的位置（提高z坐标）
          geometry_msgs::Pose adjusted_pose = target_pose;
          adjusted_pose.position.z += 0.05 + 0.03 * attempt;  // 每次增加更多高度
          
          ROS_INFO("规划失败。先移动到调整高度 %.3f 的位置，然后再尝试原始目标。", 
                  adjusted_pose.position.z);
          
          // 递归调用move_arm执行调整后的位置
          bool adjusted_move_success = move_arm(adjusted_pose, false);
          
          if (adjusted_move_success) {
            ROS_INFO("成功移动到调整位置，现在尝试原始目标位置");
            // 重新设置原始目标并继续尝试
            arm_group_.setPoseTarget(target_pose);
          } else {
            ROS_WARN("移动到调整位置失败，继续尝试下一个调整");
            continue;
          }
        } else {
          ROS_WARN("在 %d 次尝试后规划失败", max_attempts);
          return false;
        }
      }
      
      // 执行计划（原始目标位置）
      auto exec_result = arm_group_.execute(my_plan);
      bool exec_success = (exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS || 
                          exec_result == moveit::planning_interface::MoveItErrorCode::TIMED_OUT);
      
      if (exec_success) {
        return true;
      }
      
      ROS_WARN("计划执行失败，尝试次数 %d。错误代码: %d", 
              attempt + 1, exec_result.val);
      
      // 如果执行失败，也尝试移动到调整位置然后再次尝试
      if (attempt < max_attempts - 1) {
        geometry_msgs::Pose recovery_pose = target_pose;
        recovery_pose.position.z += 0.08 + 0.04 * attempt;  // 使用更大的调整
        
        ROS_INFO("执行失败后尝试恢复位置 z=%.3f", recovery_pose.position.z);
        bool recovery_success = move_arm(recovery_pose, false);
        
        if (recovery_success) {
          ROS_INFO("成功移动到恢复位置，继续尝试");
        }
      }
      
      // 短暂等待后重试
      ros::Duration(0.5).sleep();
    }
    
    ROS_WARN("所有规划和执行尝试都失败了");
    return false;
  }
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
    bool move_down_success = move_arm(target_pos_down, true);
    
    
    bool close_gripper_success = move_gripper(0.0);
    
    
    target_pos_up.position = obj_loc;
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
  
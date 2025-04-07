// File: move_and_pick.cpp
#include "cw2_class.h"
#include "external.h"
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>


bool cw2::moveArm(geometry_msgs::Pose& target_pose, bool use_cartesian, int recursion_depth, bool fast) {
  
  // Limit recursion depth
  const int max_recursion_depth = 3;
  if (recursion_depth > max_recursion_depth) {
    ROS_ERROR("Maximum recursion depth reached in moveArm. Aborting.");
    return false;
  }

  ROS_INFO("Setting pose target (recursion depth: %d).", recursion_depth);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool plan_success = false;
  bool exec_success = false;

  // Use cartesian path finding
  // We didn't decide to use this in the end because its velosity is uncntrolable
  if (use_cartesian) {
    ROS_INFO("Using Cartesian path planning");
    
    // End effector position
    geometry_msgs::Pose start_pose = arm_group_.getCurrentPose().pose;
    
    // path from start to end
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(start_pose);
    waypoints.push_back(target_pose);
    
    // Compute cartisian path
    moveit_msgs::RobotTrajectory trajectory;

    double fraction = arm_group_.computeCartesianPath(waypoints, 
                                                    0.01,    // (eef_step)
                                                    trajectory);
    
    ROS_INFO("Cartesian path planning (%.2f%% achieved)", fraction * 100.0);

    if (fraction > 0.0) {
      // Construct complete plan
      my_plan.trajectory_ = trajectory;
      plan_success = true;

      //////////////////////////////////////////////
      

      // Optimize trajectory speed
      if(fast){
        robot_trajectory::RobotTrajectory rt(arm_group_.getCurrentState()->getRobotModel(), "panda_arm");
        rt.setRobotTrajectoryMsg(*arm_group_.getCurrentState(), my_plan.trajectory_);
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        double velocity_scaling_factor = 1.5;  // Scale velocity
        double acceleration_scaling_factor = 1;  // Scale acceleration
        iptp.computeTimeStamps(rt, velocity_scaling_factor, acceleration_scaling_factor);

        // Aply scaled trajectory
        rt.getRobotTrajectoryMsg(my_plan.trajectory_);
      }
      // Check optimized trajectory
      ROS_INFO("checking trajectory timestamps...");
      checkTrajectoryTimestamps(my_plan.trajectory_);

      ROS_INFO("Executing Cartesian path plan...");

      auto exec_result = arm_group_.execute(my_plan);
      exec_success = exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS;

      if (exec_result == moveit::planning_interface::MoveItErrorCode::TIMED_OUT) {
        ROS_WARN("Execution timed out, but continuing as this may be close enough.");
        exec_success = true;
      }
    } else {
      ROS_WARN("Could not compute Cartesian path, falling back to joint space planning.");
      use_cartesian = false; 
    }
  }
  
  // Use jointspace control (We always use this)
  if (!use_cartesian || !plan_success || !exec_success) {
    arm_group_.setPoseTarget(target_pose);
    
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
      // Increase trajectroy speed
      if(fast){
        robot_trajectory::RobotTrajectory rt(arm_group_.getCurrentState()->getRobotModel(), "panda_arm");
        rt.setRobotTrajectoryMsg(*arm_group_.getCurrentState(), my_plan.trajectory_);
        // Scale trajectory veosity and acceleration
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        double velocity_scaling_factor = 1.5;  
        double acceleration_scaling_factor = 1;  
        iptp.computeTimeStamps(rt, velocity_scaling_factor, acceleration_scaling_factor);

        // Apply trajectory
        rt.getRobotTrajectoryMsg(my_plan.trajectory_);
      }
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
  // Fallback: Intermediate waypoint
  if (!plan_success || !exec_success) {
    ROS_WARN("All attempts failed. Trying intermediate waypoint approach.");
    
    // Aquire current position
    InitPose current_pose;
    current_pose = arm_group_.getCurrentPose().pose;
    InitPose temp_pose;
    
    // Compute middle point
    temp_pose.position.x = (current_pose.position.x + target_pose.position.x) / 2.0;
    temp_pose.position.y = (current_pose.position.y + target_pose.position.y) / 2.0;
    temp_pose.position.z = (current_pose.position.z + target_pose.position.z) / 2.0;
    
    // use (SLERP) to compute pose midpoint
    tf2::Quaternion q1, q2, q_result;
    tf2::convert(current_pose.orientation, q1);
    tf2::convert(target_pose.orientation, q2);
    
    // Use 0.5 as the parameter to compuet middle position
    q_result = q1.slerp(q2, 0.5);
    q_result.normalize();
    tf2::convert(q_result, temp_pose.orientation);
    
    ROS_INFO("Moving to intermediate waypoint.");
    bool waypoint_success;

    if (recursion_depth == max_recursion_depth - 1) {
      ROS_WARN("Maximum recursion depth reached. Attempting final target with temporary waypoint.(REMOVE THE CONSTRAINT)");
      resetArm();
      waypoint_success = moveArm(temp_pose, use_cartesian, recursion_depth + 1, fast);

    } else {
      waypoint_success = moveArm(temp_pose, use_cartesian, recursion_depth + 1, fast);
    }

    if (waypoint_success) {
      ROS_INFO("Successfully moved to intermediate waypoint. Attempting final target again.");
      return moveArm(target_pose, use_cartesian, recursion_depth + 1, fast);
    } else {
      ROS_ERROR("Failed to move to intermediate waypoint.");
      return false;
    }
  }
  
  return true;
}

// Pick and place with poitn input
void cw2::pickAndPlace(const std::string& obj_name, const geometry_msgs::Point& obj_loc, const geometry_msgs::Point& goal_loc, std::vector<std::string> collision_obj_parts) {
  InitPose obj_point;
  obj_point.position = obj_loc;
  pickAndPlace(obj_name, obj_point, goal_loc, collision_obj_parts);
}


void cw2::pickAndPlace(const std::string& obj_name, const geometry_msgs::Pose& obj_loc, const geometry_msgs::Point& goal_loc, std::vector<std::string> collision_obj_parts) {
    ROS_INFO("picking up and placing: %s", obj_name.c_str());
    

    
    InitPose target_pos_down;
    InitPose target_pos_up;
    InitPose target_pos_down2;
    InitPose target_pos_up2;
    
    ///////////////////////////////////////////
    // initial position
    ///////////////////////////////////////////  
    // You should be above the object at this point
    // Open gripper
    bool opgrip_success = moveGripper(1.0);
    


    clearConstraint();
    target_pos_down = obj_loc;
    target_pos_down.position.z = 0.04 + 0.11; //estimated good grasping position
    bool move_down_success = moveArm(target_pos_down, true, 0, false);
    
    removeCollisionObjects(collision_obj_parts);
    bool close_gripper_success = moveGripper(0.0);
    
    
    target_pos_up = obj_loc;
    target_pos_up.position.z = 0.41;
    bool move_up_success = moveArm(target_pos_up, true, 0, false);
    
    ///////////////////////////////////////////
    // Move to goal position
    ///////////////////////////////////////////

    // setConstraint();
    setZConstraint(0.4, 1.2);
    target_pos_down2.position = goal_loc;
    target_pos_down2.position.z = 0.41;
    // clearConstraint();
    bool move_goal_success = moveArm(target_pos_down2, false, 0, false);
    
    clearConstraint();
    
    ///////////////////////////////////////////
    // Place the object
    ///////////////////////////////////////////
    
    
    target_pos_down2.position = goal_loc;
    target_pos_down2.position.z = 0.25;
    bool move_down_success2 = moveArm(target_pos_down2, true, 0, false);
    
    bool open_gripper_success = moveGripper(1.0);
    
    target_pos_up2.position = goal_loc;
    target_pos_up2.position.z = 0.41;
    bool move_up_success2 = moveArm(target_pos_up2, true);

    //setConstraint();
}
      



void cw2::setConstraint() {
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
  
  void cw2::clearConstraint() {
    constraints_.joint_constraints.clear();
    constraints_.position_constraints.clear();  

    // Re-apply the modified constraints
    arm_group_.clearPathConstraints();
  }


  bool cw2::moveGripper(float width) {
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
  float rot_degree, 
  float obj_size) {
  // Rotate by target_pose.orientation
  tf2::Quaternion q_orig, q_rot, q_new;
  tf2::convert(target_pose.orientation, q_orig);
  q_rot.setRPY(0, 0, rot_degree * M_PI / 180); // 旋转角度转换为弧度
  q_new = q_rot * q_orig;
  q_new.normalize();
  tf2::convert(q_new, target_pose.orientation);

  // initialize variables
  float offset_distance = 0.0;
  float offset_x = 0.0;
  float offset_y = 0.0;
  float rad_angle = rot_degree * M_PI / 180.0; // 角度转弧度

  if (shape_type == "cross") {
    offset_distance = obj_size * 1.5;
    // Compute x y ofset for cross
    offset_x = offset_distance * cos(rad_angle);
    offset_y = offset_distance * sin(rad_angle);

  ROS_INFO("Cross offset: [x: %f, y: %f] (angle: %f deg)", offset_x, offset_y, rot_degree);

  } else if (shape_type == "nought") {
    offset_distance = obj_size * 2;
    // For nought, we shift the grasp position in the y direction of the original object frame to reach the grasp position
    offset_x = -offset_distance * sin(rad_angle);
    offset_y = offset_distance * cos(rad_angle);

    ROS_INFO("Nought offset: [x: %f, y: %f] (angle: %f deg)", offset_x, offset_y, rot_degree);
  }

  // Apply offset
  target_pose.position.x += offset_x;
  target_pose.position.y += offset_y;
}



/**
 * Set end effector constraint
 * @param min_height 
 * @param max_height 
 */
void cw2::setZConstraint(double min_height, double max_height) {
  // Clear constraints
  moveit_msgs::Constraints temp_constraints = constraints_;
  constraints_.position_constraints.clear();  
  
  // Create box for allowed areas
  shape_msgs::SolidPrimitive box;
  box.type = shape_msgs::SolidPrimitive::BOX;
  box.dimensions.resize(3);
  box.dimensions[0] = 20.0;  // x方向很大，不限制
  box.dimensions[1] = 20.0;  // y方向很大，不限制
  box.dimensions[2] = max_height - min_height;  // z方向限制范围
  
  // Set box position
  geometry_msgs::Pose box_pose;
  box_pose.position.x = 0.0;
  box_pose.position.y = 0.0;
  box_pose.position.z = (min_height + max_height) / 2.0;
  box_pose.orientation.w = 1.0;
  
  // Set position constraint
  moveit_msgs::PositionConstraint pos_constraint;
  pos_constraint.header.frame_id = "world";  
  pos_constraint.link_name = "panda_link7";
  
  // Set constraint area
  pos_constraint.constraint_region.primitives.push_back(box);
  pos_constraint.constraint_region.primitive_poses.push_back(box_pose);
  pos_constraint.weight = 1.0;
  
  // Apply constraint
  constraints_.position_constraints.push_back(pos_constraint);
  
  // Recver joint constraint (if any)
  constraints_.joint_constraints = temp_constraints.joint_constraints;
  
  // Apply all constraints
  arm_group_.setPathConstraints(constraints_);
  
  ROS_INFO("Set Z-axis constraint: min=%.2f, max=%.2f", min_height, max_height);
}



bool cw2::resetArm(double min_height, double target_height)
{
  clearConstraint();
  // Aquire arm positions
  geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();
  double current_z = current_pose.pose.position.z;
  
  // Check if current z is too low
  if (current_z < min_height) {
    ROS_INFO("Current arm height (%.3f) is below minimum (%.3f), adjusting to %.3f", 
             current_z, min_height, target_height);
    
    // Create new target position
    geometry_msgs::Pose target_pose = current_pose.pose;
    target_pose.position.z = target_height;
    
    // Set target
    arm_group_.setPoseTarget(target_pose);
    
    // Plan and move
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
    // if the arm is high enough, don't move
    ROS_INFO("Arm height (%.3f) is already above minimum (%.3f), no adjustment needed", 
             current_z, min_height);
    return true;
  }

  setZConstraint(0.4, 1.2); // Reapply Z constraints
  return true;
}

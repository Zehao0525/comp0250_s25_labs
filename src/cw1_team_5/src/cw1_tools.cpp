#include "cw1_class.h"

// #include "add_or_remove_functions.cpp"

void cw1::reset_task() {
  cloud_ptr_.reset(new PointC);
  cloud_filtered_.reset(new PointC);
  cloud_filtered2_.reset(new PointC);
  cloud_world_.reset(new PointC);
  cloud_world_tmp_.reset(new PointC);
  tree_ptr_.reset(new pcl::search::KdTree<PointT>());
  remove_all_collisions();
  cloud_dirty_flag_ = false;
}

// =====================================================
//            Helper functions
// =====================================================

bool cw1::move_arm(geometry_msgs::Pose& target_pose) {
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

////////////////////////////////////////////////////////////////////////////////

bool cw1::move_gripper(float width) {
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

////////////////////////////////////////////////////////////////////////////////

void cw1::apply_voxel_filter(PointCPtr& in_cloud_ptr, PointCPtr& out_cloud_ptr) {
  voxel_grid_.setInputCloud(in_cloud_ptr);
  voxel_grid_.setLeafSize(vg_leaf_size_, vg_leaf_size_, vg_leaf_size_);
  voxel_grid_.filter(*out_cloud_ptr);
}

////////////////////////////////////////////////////////////////////////////////

void cw1::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg) {
  // Extract input point cloud info
  input_pc_frame_id_ = cloud_input_msg->header.frame_id;
  input_pc_time_ = pcl_conversions::toPCL(cloud_input_msg->header.stamp);
    
  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_input_msg, pcl_pc_);
  pcl::fromPCLPointCloud2(pcl_pc_, *cloud_ptr_);

  // Perform the filtering
  apply_voxel_filter(cloud_ptr_, cloud_filtered_);
  cloud_filtered_->header.frame_id = input_pc_frame_id_;
  cloud_filtered_->header.stamp = input_pc_time_;

  cloud_dirty_flag_ = true;
}

////////////////////////////////////////////////////////////////////////////////

void cw1::set_constraint() {
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

////////////////////////////////////////////////////////////////////////////////

void cw1::pick_place_cube(const std::string& obj_name, 
                          const geometry_msgs::Point& obj_loc, 
                          const geometry_msgs::Point& goal_loc) {
  ROS_INFO("picking up and placing: %s", obj_name.c_str());

  // Set constraint
  set_constraint();

  // We specify waypoints for the arm trajectory
  geometry_msgs::Pose target_pos, lift_pos, goal_pos;

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
  bool mvarm_success = move_arm(lift_pos);

  // Open gripper
  bool opgrip_success = move_gripper(1.0);

  mvarm_success = mvarm_success && move_arm(target_pos);
  // Close gripper
  remove_collision_object(obj_name);
  bool clgrip_success = move_gripper(0.0);
  
  // Move to lifted position
  mvarm_success = mvarm_success && move_arm(lift_pos);
  // Move to goal position
  mvarm_success = mvarm_success && move_arm(goal_pos);
  // Open gripper
  bool opgrip_success2 = move_gripper(0.8);
}

// Condition function 
bool enforce_color_similarity(const PointT& a, const PointT& b, float /*squared_dist*/) {
  float b_scale = std::max({b.r, b.g, b.b});
  float a_scale = std::max({a.r, a.g, a.b});
  float tol = 0.05f * b_scale * a_scale;
  // This below logic will ignore pure black clusters. (Or be undefined, but I think we wouldn't get 
  // floating point error on *0, so it is ignore)
  // This is the price we pay for saving the compute on handeling scale == 0 cases
  return (std::fabs(a.r * b_scale - b.r * a_scale) < tol) && 
         (std::fabs(a.g * b_scale - b.g * a_scale) < tol) && 
         (std::fabs(a.b * b_scale - b.b * a_scale) < tol);
}

void cw1::wait_for_new_point_cloud() {
  ros::Rate rate(10.0);  // 10 Hz loop rate (checks 10 times per second)

  while (ros::ok() && !cloud_dirty_flag_) {
    ros::spinOnce();  // Process incoming messages
    rate.sleep();     // Wait until next loop iteration (0.1 seconds)
  }

  if (cloud_dirty_flag_) {
    cloud_dirty_flag_ = false;  // Reset flag after detection if appropriate
  }
}

void cw1::convert_ptcld_to_world(PointCPtr& input_cloud, PointCPtr& transformed_cloud) {
  PointCPtr tmp_cloud(new PointC);
  // Wait for transform availability
  if (!tf_listener_.waitForTransform(base_frame_,
        input_cloud->header.frame_id,
        ros::Time(0), ros::Duration(3.0))) {
    ROS_WARN("Transform unavailable from %s to %s",
      input_cloud->header.frame_id.c_str(), base_frame_.c_str());
    return;
  }
  
  ROS_INFO("Transforming from %s to %s", 
           input_cloud->header.frame_id.c_str(), 
           base_frame_.c_str());

  // Perform the transformation
  try {
    pcl_ros::transformPointCloud(base_frame_, *input_cloud, *tmp_cloud, tf_listener_);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("Transformation error: %s", ex.what());
    return;
  }

  // Do a z filter to remove the ground plane
  // Reduce memory consumption
  pass_filter_.setInputCloud(tmp_cloud);
  pass_filter_.setFilterFieldName("z");
  pass_filter_.setFilterLimits(0.015, std::numeric_limits<float>::max()); // keeps points with z ≥ 0.015
  pass_filter_.filter(*transformed_cloud);

  // Update header
  transformed_cloud->header.frame_id = base_frame_;
  transformed_cloud->header.stamp = input_cloud->header.stamp;

  // Merge point clouds
  *cloud_world_ += *transformed_cloud;
  cloud_world_->header.frame_id = base_frame_;
  cloud_world_->header.stamp = pcl_conversions::toPCL(ros::Time::now());
}

////////////////////////////////////////////////////////////////////////////////

void cw1::detect_objects(PointCPtr& in_cloud_ptr, std::vector<DetectedObject>& detected_objects) {
  // --- 1. Cluster Extraction ---
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(in_cloud_ptr);
  
  // For each cluster, 
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::ConditionalEuclideanClustering<PointT> cec(true);   // 'true' to allow cluster size filtering
  cec.setInputCloud(in_cloud_ptr);
  cec.setConditionFunction(&enforce_color_similarity);
  cec.setClusterTolerance(0.02f);                        // 5mm spatial cluster radius
  cec.setMinClusterSize(25);  
  cec.setSearchMethod(tree); 

  // Cluster pointcloud
  cec.segment(cluster_indices);

  // Counters for cubes and baskets
  int cube_count = 0, basket_count = 0;

  geometry_msgs::Point obj_position_tmp;
  
  // --- 2. Process each cluster ---
  for (const auto &indices : cluster_indices) {
    // Create a new cloud for the current cluster
    PointCPtr cluster(new PointC);
    for (const auto &idx : indices.indices) {
      cluster->points.push_back(in_cloud_ptr->points[idx]);
    }
    
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

    cylinder_point_msg_.header.frame_id = input_pc_frame_id_;
    cylinder_point_msg_.header.stamp = ros::Time(0);
    cylinder_point_msg_.point.x = cluster_centroid[0];
    cylinder_point_msg_.point.y = cluster_centroid[1];
    cylinder_point_msg_.point.z = cluster_centroid[2];

    const float tol = 0.02f; // tolerance in meters

    bool is_cube = (std::fabs(dims[0] - 0.04f) < tol &&
                   std::fabs(dims[1] - 0.04f) < tol && 
                   std::fabs(max_pt.z - 0.05f) < tol);

    bool is_basket = (std::fabs(dims[0] - 0.1f) < tol &&
                     std::fabs(dims[1] - 0.1f) < tol && 
                     std::fabs(max_pt.z - 0.11) < tol);

    // obtain rgba data as the average rgb values
    int r = 0;
    int g = 0;
    int b = 0;
    
    // retrieve the RGB data
    for (int j = 0; j < cluster->points.size(); j++) {
      uint32_t rgba = cluster->points[j].rgba;
      uint8_t uint8_r = (rgba >> 16) & 0x0000ff;
      uint8_t uint8_g = (rgba >> 8) & 0x0000ff;
      uint8_t uint8_b = (rgba) & 0x0000ff;
      uint8_t uint8_a = (rgba >> 24) & 0x000000ff;

      r += uint8_r;
      g += uint8_g;
      b += uint8_b;
    }

    // take the average number of rgb of the image area
    r = r / cluster->points.size();
    g = g / cluster->points.size();
    b = b / cluster->points.size();
    
    // --- 4. Data analysis
    // We find the color of the object
    // Use better conditions
    DetectedObject cur_obj;
    float color_scale = std::max({r, g, b}) / 0.8; // This will scale the Color to the GT RGB colors

    // Get meta information
    cur_obj.w = dims[0];
    cur_obj.l = dims[1];
    cur_obj.h = dims[2];
    cur_obj.r = r;
    cur_obj.g = g;
    cur_obj.b = b;

    // Assign position
    cur_obj.position = cylinder_point_msg_.point;

    if(color_scale == 0) {
      cur_obj.color = "other";
    } else if (std::fabs(r / color_scale - red_color_[0]) < 0.1 && 
              std::fabs(g / color_scale - red_color_[1]) < 0.1 && 
              std::fabs(b / color_scale - red_color_[2]) < 0.1) {
      cur_obj.color = "red";
    } else if (std::fabs(r / color_scale - blue_color_[0]) < 0.1 && 
              std::fabs(g / color_scale - blue_color_[1]) < 0.1 && 
              std::fabs(b / color_scale - blue_color_[2]) < 0.1) {
      cur_obj.color = "blue";
    } else if (std::fabs(r / color_scale - purple_color_[0]) < 0.1 && 
              std::fabs(g / color_scale - purple_color_[1]) < 0.1 && 
              std::fabs(b / color_scale - purple_color_[2]) < 0.1) {
      cur_obj.color = "purple";
    } else {
      cur_obj.color = "other";
    }

    // We find the type of the object
    std::cout << "obj z: " << obj_position_tmp.z << std::endl;
    
    if (is_cube) {
      cur_obj.type = "cube";
      std::cout << "Detected cube. Dimensions (m): " << dims << std::endl;
    } else if (is_basket) {
      cur_obj.type = "basket";
      std::cout << "Detected basket. Dimensions (m): " << dims << std::endl;
    } else {
      cur_obj.type = "other";
      std::cout << "Unknown object. Dimensions (m): " << dims << std::endl;
    }

    detected_objects.push_back(cur_obj);
  }
}
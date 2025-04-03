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
  reset_arm();
  // setConstraint();
  ROS_INFO("The coursework solving callback for task 1 has been triggered");
  addPlane("plane");

  geometry_msgs::PointStamped object_point = request.object_point;
  geometry_msgs::PointStamped goal_point = request.goal_point;
  std::string shape_type = request.shape_type;

  ROS_INFO("Object Point: [x: %f, y: %f, z: %f]", object_point.point.x, object_point.point.y, object_point.point.z);
  ROS_INFO("Goal Point: [x: %f, y: %f, z: %f]", goal_point.point.x, goal_point.point.y, goal_point.point.z);
  ROS_INFO("Shape Type: %s", shape_type.c_str());


  Init_Pose target_pose;
  target_pose.position = object_point.point;
  target_pose.position.z = 0.5;


  // setConstraint();
  // clear_constraint();
  
  bool mvstate = move_arm(target_pose);
  ROS_INFO("Move state: %d", mvstate);
  set_z_constraint(0.4, 1.2);


  DetectedObject detected_object;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr obj_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PCL_cloud = convertToPCL(latest_cloud, tf_listener_); // frame to world
  filterPointCloudByHeight(PCL_cloud, obj_cloud_ptr, 0.04, 0.08); // 例如，保留 z 高度在 0.0 到 1.0 之间的点



  // 平移点云至原点
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << -detected_object.position.x, -detected_object.position.y, -detected_object.position.z;
  pcl::transformPointCloud(*obj_cloud_ptr, *obj_cloud_ptr, transform);
  

  pcl::io::savePCDFileASCII("1.pcd", *obj_cloud_ptr);

  
  //////////////////////////////
  ShapeDetectionResult detection_result = detectShapeRotation_multi(obj_cloud_ptr);
  if (detection_result.rotation_angle < 0) {
    ROS_ERROR("Cannot detect shape");
    return false;
  }
  
  // 使用检测到的形状类型和旋转角度
  std::string detected_shape = detection_result.shape_type;
  float rot_degree = detection_result.rotation_angle;
  
  ROS_INFO("Detected shape: %s (size: %.3f)", detected_shape.c_str(), detection_result.size);
  
  /////////////////////////////////////////

  std::vector<std::string> collision_obj_parts;
  if(detection_result.shape_type  == "cross"){
    collision_obj_parts = genCrossObj(detection_result.size, target_pose.position.x, target_pose.position.y, detection_result.rotation_angle);
  }
  else{
    collision_obj_parts = genNoughtObj(detection_result.size, target_pose.position.x, target_pose.position.y, detection_result.rotation_angle);
  }

  adjustPoseByShapeAndRotation(target_pose, shape_type, rot_degree);



  target_pose.position.z = 0.41;


  bool mvstate1 = move_arm(target_pose);
  ROS_INFO("Move state: %d", mvstate1);



  Init_Pose goal_pose;
  goal_pose.position = goal_point.point;
  addBasket("basket", goal_point.point);
  goal_pose.position.z = 0.2;


  pick_and_place(shape_type, target_pose,  goal_pose.position, collision_obj_parts);
  removeAllCollisions();
  return true;
}


///////////////////////////////////////////////////////////////////////////////

bool
cw2::t2_callback(cw2_world_spawner::Task2Service::Request &request,
  cw2_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */
  // setConstraint();
  reset_arm();
  set_z_constraint(0.4, 1.2);

  // add arm constraints
  // setConstraint();
  // add plane model
  addPlane("plane");

  // decoder
  std::vector<geometry_msgs::Point> ref_object_points;
  std::vector<ShapeDetectionResult> reference_shapes;

  ROS_INFO("The coursework solving callback for task 2 has been triggered");

  for (const auto& point : request.ref_object_points) {
    ROS_INFO("Reference Object Point: [x: %f, y: %f, z: %f]", point.point.x, point.point.y, point.point.z);
    ref_object_points.push_back(point.point);
  }


  // 检测每个参考对象
  for (const auto& point : request.ref_object_points) {
    geometry_msgs::PointStamped reference_point = point;

    // 移动到参考对象上方
    Init_Pose target_pose;
    target_pose.position = reference_point.point;
    target_pose.position.z = 0.5;
    bool mvstate = move_arm(target_pose);
    ROS_INFO("Move state: %d", mvstate);
  
    // 获取点云数据
    ROS_INFO("Converting to PCL...");
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PCL_cloud = convertToPCL(latest_cloud, tf_listener_);
    
    // 过滤点云数据
    ROS_INFO("Filtering pointcloud...");
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr obj_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    filterPointCloudByHeight(PCL_cloud, obj_cloud_ptr, 0.04, 0.08);
    
    // 平移点云至原点
    ROS_INFO("Traslating pointcloud for shape matching...");
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << -reference_point.point.x, -reference_point.point.y, -reference_point.point.z;
    pcl::transformPointCloud(*obj_cloud_ptr, *obj_cloud_ptr, transform);
    
    // 检测形状
    ROS_INFO("Matching shape to orientation, processing...");
    ShapeDetectionResult detection_result = detectShapeRotation_multi(obj_cloud_ptr);
    if (detection_result.rotation_angle < 0) {
      ROS_WARN("Cannot detect shape for reference object at position [%f, %f, %f]", 
               reference_point.point.x, reference_point.point.y, reference_point.point.z);
      // 继续尝试下一个参考对象
      continue;
    }
    
    // 存储检测结果
    ROS_INFO("Detected reference shape: %s (size: %.3f, angle: %.1f)", 
             detection_result.shape_type.c_str(), detection_result.size, detection_result.rotation_angle);
    reference_shapes.push_back(detection_result);
  }
  
  /////////////////////////////////////////

    // 如果没有成功检测到任何参考对象，返回失败
    if (reference_shapes.empty()) {
      ROS_ERROR("No reference shapes could be detected");
      return false;
    }

////////////////////////////////////////////

  // 处理神秘对象
  geometry_msgs::PointStamped mystery_point = request.mystery_object_point;
  ROS_INFO("Mystery Object Point: [x: %f, y: %f, z: %f]", 
           mystery_point.point.x, mystery_point.point.y, mystery_point.point.z);
  
  // 移动到神秘对象上方
  Init_Pose target_pose;
  target_pose.position = mystery_point.point;
  target_pose.position.z = 0.5;
  bool mvstate = move_arm(target_pose);
  ROS_INFO("Move state: %d", mvstate);

  // 获取点云数据
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PCL_cloud = convertToPCL(latest_cloud, tf_listener_);
  
  // 过滤点云数据
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr obj_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
  filterPointCloudByHeight(PCL_cloud, obj_cloud_ptr, 0.04, 0.08);
  
  // 平移点云至原点
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << -mystery_point.point.x, -mystery_point.point.y, -mystery_point.point.z;
  pcl::transformPointCloud(*obj_cloud_ptr, *obj_cloud_ptr, transform);
  
  // 检测神秘对象的形状
  ROS_INFO("Matching shapes, processing...");
  ShapeDetectionResult mystery_result = detectShapeRotation_multi(obj_cloud_ptr);
  if (mystery_result.rotation_angle < 0) {
    ROS_ERROR("Cannot detect shape for mystery object");
    return false;
  }
  
  ROS_INFO("Detected mystery shape: %s (size: %.3f, angle: %.1f)", 
           mystery_result.shape_type.c_str(), mystery_result.size, mystery_result.rotation_angle);
  
////////////////////////////////////////////



 // 匹配神秘对象与参考对象
 size_t match_idx = 0;
 bool found_match = false;
 
 for (size_t idx = 0; idx < reference_shapes.size(); ++idx) {
   // 只匹配形状类型
   if (mystery_result.shape_type == reference_shapes[idx].shape_type) {
     ROS_INFO("Matched object at index: %zu (shape: %s)", 
              idx, reference_shapes[idx].shape_type.c_str());
     match_idx = idx;
     found_match = true;
     break;
   }
 }
 
 // 如果没有找到匹配的形状类型，默认使用第一个参考对象
 if (!found_match) {
   ROS_WARN("No matching shape found for mystery object (detected as %s), defaulting to index 0", 
            mystery_result.shape_type.c_str());
   match_idx = 0;
 }
 
 response.mystery_object_num = match_idx + 1;
 ROS_INFO("Task 2 complete: Mystery object matches reference object %zu", match_idx + 1);
 removeAllCollisions();
  return true;

}


///////////////////////////////////////////////////////////////////////////////

bool
cw2::t3_callback(cw2_world_spawner::Task3Service::Request &request,
  cw2_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");
  reset_arm();
  // setConstraint();
  addPlane("plane");
  // setConstraint();




  combined_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
  set_z_constraint(0.4, 1.2);
  combined_cloud = scanPlatform();
  // clear_constraint();
  // setConstraint();


  // 1. 点云下采样 - 使用VoxelGrid滤波器减少点数量
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
  vg.setInputCloud(combined_cloud);
  vg.setLeafSize(0.002f, 0.002f, 0.002f); // 5mm体素大小
  vg.filter(*cloud_filtered);
  ROS_INFO("Cloud size after voxel grid filtering: %zu", cloud_filtered->points.size());
  
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered_3 = cloud_filtered;

    // 从保存点云文件后开始替换
  
  // 4. 保存最终的点云为PCD文件
  std::string pcd_filename = "task3_scan_result.pcd";
  pcl::io::savePCDFileASCII(pcd_filename, *cloud_filtered_3);
  ROS_INFO("Point cloud saved to %s", pcd_filename.c_str());
  
  ROS_INFO("====================================  Detected Objects  ====================================");
  
  // 检测物体
  std::vector<ShapeDetectionResult> detected_objects;

  clearObstacles();
  detect_objects(cloud_filtered_3, detected_objects, obstacles_); //识别到的坐标没有转回到世界坐标系

  if (obstacles_.size() > 0) {
    ROS_INFO("Detected %zu obstacles", obstacles_.size());
    for (const auto& obstacle : obstacles_) {
      addObstacle(obstacle);
    }
  } else {
    ROS_INFO("No obstacles detected");
  }
  
  
  // 处理结果
  response.total_num_shapes = detected_objects.size() - 1; // 排除篮子
  ShapeDetectionResult picked_object, candidate_cross, candidate_nought, candidate_basket;
  int cross_count = 0;
  int nought_count = 0;
  float size_c = 0.02;
  float size_n = 0.02;
  
  // 使用转换后的DetectedObject处理， 选择大的cross和nought方便夹取
  for (const auto& obj : detected_objects) {
    if (obj.shape_type == "cross") {
      cross_count++;
      if (obj.size >= size_c) {
        size_c = obj.size;
        candidate_cross = obj;
      }
    } else if (obj.shape_type == "nought") {
      nought_count++;
      if (obj.size >= size_n) {
        size_n = obj.size;
        candidate_nought = obj;
      }
    } else if (obj.shape_type == "basket") {
      candidate_basket = obj;
    }
  }
  // 确定要拾取的物体
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
  } else {
    response.num_most_common_shape = nought_count;
    ROS_INFO("Most common shape: nought");
    ROS_INFO("Nought count: %d", nought_count);
    picked_object = candidate_nought;
  }
  
  std::vector<std::string> collision_obj_parts;
  if(picked_object.shape_type  == "cross"){
    collision_obj_parts = genCrossObj(picked_object.size, picked_object.centroid[0], picked_object.centroid[1], picked_object.rotation_angle);
  }
  else{
    collision_obj_parts = genNoughtObj(picked_object.size, picked_object.centroid[0], picked_object.centroid[1], picked_object.rotation_angle);
  }

  // 对位置进行调整以提高抓取成功率
  if (picked_object.shape_type == "cross") { 
    picked_object.centroid[0] += 0.05; 
  } else if (picked_object.shape_type == "nought") { 
    picked_object.centroid[1]+= 0.08; 
  }

  geometry_msgs::Pose grasp_pose;
  grasp_pose.position.x = picked_object.centroid[0];
  grasp_pose.position.y = picked_object.centroid[1];
  grasp_pose.position.z = 0.41; // 设置z轴高度


  Init_Pose temp_pose;
  temp_pose.position.x = picked_object.centroid[0];
  temp_pose.position.y = picked_object.centroid[1];
  temp_pose.position.z = 0.41; // 设置z轴高度


  geometry_msgs::Point candidate_basket_point;
  candidate_basket_point.x = candidate_basket.centroid[0];
  candidate_basket_point.y = candidate_basket.centroid[1];
  addBasket("basket", candidate_basket_point);
  candidate_basket_point.z = 0.2; // 设置z轴高度

  // 执行抓取和放置
  adjustPoseByShapeAndRotation(grasp_pose, picked_object.shape_type, picked_object.rotation_angle);

  // move arm to above the object
  move_arm(temp_pose);

  // 执行抓取和放置操作 
  // pick and place
  pick_and_place(picked_object.shape_type, temp_pose.position, candidate_basket_point, collision_obj_parts);

  
  ROS_INFO("Task 3 complete");
  removeAllCollisions();
  return true;

}



void cw2::addObstacle(const Obstacle& obstacle) {
  
  // 创建碰撞对象
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "world";
  collision_object.id = "obstacle_" + std::to_string(obstacles_.size());
  
  // 直接使用obstacle中已定义的primitive和pose
  collision_object.primitives.push_back(obstacle.obstacle);
  collision_object.primitive_poses.push_back(obstacle.obstacle_pose);
  collision_object.operation = collision_object.ADD;
  
  // 将碰撞对象添加到规划场景
  planning_scene_interface_.applyCollisionObject(collision_object);
  
  ROS_INFO("Added obstacle at [x: %f, y: %f, z: %f]", 
           obstacle.obstacle_pose.position.x, 
           obstacle.obstacle_pose.position.y, 
           obstacle.obstacle_pose.position.z);
}

void cw2::clearObstacles() {
  // 清除所有障碍物
  obstacles_.clear();
  
  // 从规划场景中移除所有障碍物
  std::vector<std::string> object_ids;
  for(size_t i = 0; i < obstacles_.size(); i++) {
    object_ids.push_back("obstacle_" + std::to_string(i+1));
  }
  planning_scene_interface_.removeCollisionObjects(object_ids);
  
  ROS_INFO("Cleared all obstacles");
}

// 检查轨迹时间戳是否严格递增
// 检查并修复轨迹时间戳
// 检查并修复轨迹时间戳
void cw2::checkTrajectoryTimestamps(moveit_msgs::RobotTrajectory& trajectory) {
  if (trajectory.joint_trajectory.points.empty()) {
    ROS_WARN("Empty trajectory, no points to check");
    return;
  }
  
  bool needs_fixing = false;
  double last_time = -1.0;
  
  ROS_INFO("Checking trajectory timestamps (total points: %zu):", trajectory.joint_trajectory.points.size());
  for (size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i) {
    double current_time = trajectory.joint_trajectory.points[i].time_from_start.toSec();
    
    if (i > 0) {
      double time_diff = current_time - last_time;
      if (time_diff <= 0.0) {
        ROS_ERROR("  Point[%zu]: time=%.6f, interval=%.6f (invalid!)", i, current_time, time_diff);
        needs_fixing = true;
      } else {
        ROS_INFO("  Point[%zu]: time=%.6f, interval=%.6f", i, current_time, time_diff);
      }
    } else {
      ROS_INFO("  Point[%zu]: time=%.6f", i, current_time);
    }
    
    last_time = current_time;
  }
  
  // 如果需要修复时间戳
  if (needs_fixing) {
    ROS_WARN("Timestamp issues detected, performing automatic fix");
    
    // 最小时间步长（10毫秒）
    const double min_time_step = 0.01;
    
    // 从0开始，确保严格递增
    for (size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i) {
      double new_time = i * min_time_step;
      trajectory.joint_trajectory.points[i].time_from_start = ros::Duration(new_time);
    }
    
    // 再次检查修复后的时间戳
    ROS_INFO("Timestamps after fixing:");
    last_time = -1.0;
    for (size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i) {
      double current_time = trajectory.joint_trajectory.points[i].time_from_start.toSec();
      if (i > 0) {
        ROS_INFO("  Point[%zu]: time=%.6f, interval=%.6f", 
                 i, current_time, current_time - last_time);
      } else {
        ROS_INFO("  Point[%zu]: time=%.6f", i, current_time);
      }
      last_time = current_time;
    }
  }
}


// #include "cw1_tools.cpp"

///////////////////////////////////////////////////////////////////////////////

void cw2::addPlane(const std::string& object_name)
{
  // 0.02 thick, 0.1 on both sides for a tile
  // x center from 0.25 to 0.65 (0.2-0.7)
  // y center from -0.4 to 0.4 (-0.45-0.45)
  // z centres at 0.01
  geometry_msgs::Point plane_pos;
  plane_pos.x = 0.05;
  plane_pos.y = 0;
  plane_pos.z = 0.01;

  geometry_msgs::Vector3 plane_dim;
  plane_dim.x = 1.32;
  plane_dim.y = 0.92;
  plane_dim.z = 0.02;

  geometry_msgs::Quaternion plate_ori;
  plate_ori.w = 1.0;
  plate_ori.x = 0.0;
  plate_ori.y = 0.0;
  plate_ori.z = 0.0;

  addCollisionObject(object_name, plane_pos, plane_dim, plate_ori);
}


void cw2::addBasket(const std::string& name, const geometry_msgs::Point& basket_pos)
{
  // Define basket location
  geometry_msgs::Vector3 basket_dim;
  basket_dim.x = 0.37;
  basket_dim.y = 0.37;
  basket_dim.z = 0.02;

  geometry_msgs::Point basket_pos_1 = basket_pos;
  basket_pos_1.z = basket_pos.z + 0.02;

  geometry_msgs::Quaternion basket_ori;
  basket_ori.w = 1.0;
  basket_ori.x = 0.0;
  basket_ori.y = 0.0;
  basket_ori.z = 0.0;

  addCollisionObject(name, basket_pos_1, basket_dim, basket_ori);
}


void cw2::removeAllCollisions()
{
  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;

  collision_object.operation = collision_object.REMOVE;
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);
}

void cw2::addCollisionObject(const std::string& object_name,
                            const geometry_msgs::Point& centre, 
                            const geometry_msgs::Vector3& dimensions,
                            const geometry_msgs::Quaternion& orientation)
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
}

void cw2::removeCollisionObject(const std::string& object_name)
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
}



////////////////////////////////////////////////////////////////////////////////

void cw2::setConstraint() {
  // We Actually want to allow the arm to spin 360 degrees
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

  // We Also don't want this
  joint_constraints_[6].joint_name = "panda_joint7";
  joint_constraints_[6].position = 0.785;
  joint_constraints_[6].tolerance_above = M_PI * 0.25;
  joint_constraints_[6].tolerance_below = M_PI * 0.25;
  joint_constraints_[6].weight = 1.0;

  // constraints.joint_constraints.push_back(joint_constraints_[0]);
  // constraints_.joint_constraints.push_back(joint_constraints_[1]);
  constraints_.joint_constraints.push_back(joint_constraints_[2]);
  constraints_.joint_constraints.push_back(joint_constraints_[3]);
  constraints_.joint_constraints.push_back(joint_constraints_[4]);
  // constraints.joint_constraints.push_back(joint_constraints_[5]);
  // constraints.joint_constraints.push_back(joint_constraints_[6]);
  arm_group_.setPathConstraints(constraints_);
}
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


  point_cloud_sub_ = nh_.subscribe("/r200/camera/depth_registered/points", 1, &cw2::pointCloudCallback, this);
  // point_cloud_sub_ = nh_.subscribe("/camera/depth/points", 1, &cw2::pointCloudCallback, this);

  point_cloud_pub_test_ = nh_.advertise<sensor_msgs::PointCloud2>("transformed_cloud", 1);

  ROS_INFO("cw2 class initialised");
}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::t1_callback(cw2_world_spawner::Task1Service::Request &request,
  cw2_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */
  removeAllCollisions();
  resetArm();
  // setConstraint();
  ROS_INFO("The coursework solving callback for task 1 has been triggered");
  addPlane("plane");

  geometry_msgs::PointStamped object_point = request.object_point;
  geometry_msgs::PointStamped goal_point = request.goal_point;
  std::string shape_type = request.shape_type;

  ROS_INFO("Object Point: [x: %f, y: %f, z: %f]", object_point.point.x, object_point.point.y, object_point.point.z);
  ROS_INFO("Goal Point: [x: %f, y: %f, z: %f]", goal_point.point.x, goal_point.point.y, goal_point.point.z);
  ROS_INFO("Shape Type: %s", shape_type.c_str());


  InitPose target_pose;
  target_pose.position = object_point.point;
  target_pose.position.z = 0.5;


  // setConstraint();
  // clearConstraint();
  
  bool mvstate = moveArm(target_pose);
  ROS_INFO("Move state: %d", mvstate);
  setZConstraint(0.4, 1.2);


  DetectedObject detected_object;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr obj_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PCL_cloud_unfiltered = convertToPCL(latest_cloud, tf_listener_); // frame to world
  
  // Apply Vocel grid
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PCL_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
  vg.setInputCloud(PCL_cloud_unfiltered);
  vg.setLeafSize(0.002f, 0.002f, 0.002f); // 5mm voxels
  vg.filter(*PCL_cloud);
  
  // Apply height filter
  filterPointCloudByHeight(PCL_cloud, obj_cloud_ptr, 0.04, 0.08); // Keep z height between 0.04-0.08 m (estimated height of object is 0.02(plateform) + 0.04(object))



  // Move pointcloud to origin for further processing
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << -detected_object.position.x, -detected_object.position.y, -detected_object.position.z;
  pcl::transformPointCloud(*obj_cloud_ptr, *obj_cloud_ptr, transform);
  

  pcl::io::savePCDFileASCII("1.pcd", *obj_cloud_ptr);

  
  //////////////////////////////
  // Find attributes of detected shape
  ShapeDetectionResult detection_result = detectShapeRotationMulti(obj_cloud_ptr);
  if (detection_result.rotation_angle < 0) {
    ROS_ERROR("Cannot detect shape");
    return false;
  }
  
  std::string detected_shape = detection_result.shape_type;
  float rot_degree = detection_result.rotation_angle;
  
  ROS_INFO("Detected shape: %s (size: %.3f)", detected_shape.c_str(), detection_result.size);
  
  /////////////////////////////////////////
  // Create collision object
  std::vector<std::string> collision_obj_parts;
  if(detection_result.shape_type  == "cross"){
    collision_obj_parts = genCrossObj(detection_result.size, target_pose.position.x, target_pose.position.y, detection_result.rotation_angle);
  }
  else{
    collision_obj_parts = genNoughtObj(detection_result.size, target_pose.position.x, target_pose.position.y, detection_result.rotation_angle);
  }

  // Aquire grasp pose
  adjustPoseByShapeAndRotation(target_pose, shape_type, rot_degree, detection_result.size);


  // Perform grasp
  target_pose.position.z = 0.41;


  bool mvstate1 = moveArm(target_pose);
  ROS_INFO("Move state: %d", mvstate1);



  InitPose goal_pose;
  goal_pose.position = goal_point.point;
  addBasket("basket", goal_point.point);
  goal_pose.position.z = 0.2;


  pickAndPlace(shape_type, target_pose,  goal_pose.position, collision_obj_parts);
  return true;
}


///////////////////////////////////////////////////////////////////////////////

bool
cw2::t2_callback(cw2_world_spawner::Task2Service::Request &request,
  cw2_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */
  // setConstraint();
  removeAllCollisions();
  resetArm();
  setZConstraint(0.4, 1.2);

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


  // Consider all reference objects
  for (const auto& point : request.ref_object_points) {
    geometry_msgs::PointStamped reference_point = point;

    // Move to abve the object
    InitPose target_pose;
    target_pose.position = reference_point.point;
    target_pose.position.z = 0.5;
    bool mvstate = moveArm(target_pose);
    ROS_INFO("Move state: %d", mvstate);
  
    // Aquire pointcloud data
    ROS_INFO("Converting to PCL...");
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PCL_cloud = convertToPCL(latest_cloud, tf_listener_);
    
    // Filter pointcloud by height
    ROS_INFO("Filtering pointcloud...");
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr obj_cloud_ptr_prefilter(new pcl::PointCloud<pcl::PointXYZRGBA>);
    filterPointCloudByHeight(PCL_cloud, obj_cloud_ptr_prefilter, 0.04, 0.08);

    // Apply Voxel filter
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr obj_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
    vg.setInputCloud(obj_cloud_ptr_prefilter);
    vg.setLeafSize(0.002f, 0.002f, 0.002f); // 2mm voxel
    vg.filter(*obj_cloud_ptr);
    
    // Move pointcloud to origin for further process
    ROS_INFO("Traslating pointcloud for shape matching...");
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << -reference_point.point.x, -reference_point.point.y, -reference_point.point.z;
    pcl::transformPointCloud(*obj_cloud_ptr, *obj_cloud_ptr, transform);
    
    // Shape detection
    ROS_INFO("Matching shape to orientation, processing...");
    ShapeDetectionResult detection_result = detectShapeRotationMulti(obj_cloud_ptr);
    if (detection_result.rotation_angle < 0) {
      ROS_WARN("Cannot detect shape for reference object at position [%f, %f, %f]", 
               reference_point.point.x, reference_point.point.y, reference_point.point.z);
      // Try next candidate
      continue;
    }
    
    // Save result
    ROS_INFO("Detected reference shape: %s (size: %.3f, angle: %.1f)", 
             detection_result.shape_type.c_str(), detection_result.size, detection_result.rotation_angle);
    reference_shapes.push_back(detection_result);
  }
  
  /////////////////////////////////////////

    // If no reference, return error
    if (reference_shapes.empty()) {
      ROS_ERROR("No reference shapes could be detected");
      return false;
    }

////////////////////////////////////////////

  // processing mysterious object
  geometry_msgs::PointStamped mystery_point = request.mystery_object_point;
  ROS_INFO("Mystery Object Point: [x: %f, y: %f, z: %f]", 
           mystery_point.point.x, mystery_point.point.y, mystery_point.point.z);
  
  // Move to mysteriouse object
  InitPose target_pose;
  target_pose.position = mystery_point.point;
  target_pose.position.z = 0.5;
  bool mvstate = moveArm(target_pose);
  ROS_INFO("Move state: %d", mvstate);

  // Aquire and process pointcloud
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PCL_cloud = convertToPCL(latest_cloud, tf_listener_);
  
  // filter pointcloud
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr obj_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
  filterPointCloudByHeight(PCL_cloud, obj_cloud_ptr, 0.04, 0.48);

  // Apply Voxel filter
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
  vg.setInputCloud(obj_cloud_ptr);
  vg.setLeafSize(0.002f, 0.002f, 0.002f); // 5mm体素大小
  vg.filter(*cloud_filtered);
  
  // move pointcloud to origin for further process
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << -mystery_point.point.x, -mystery_point.point.y, -mystery_point.point.z;
  pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, transform);
  
  // Mysterious object shape detection
  ROS_INFO("Matching shapes, processing...");
  ShapeDetectionResult mystery_result = detectShapeRotationMulti(cloud_filtered);
  if (mystery_result.rotation_angle < 0) {
    ROS_ERROR("Cannot detect shape for mystery object");
    return false;
  }
  
  ROS_INFO("Detected mystery shape: %s (size: %.3f, angle: %.1f)", 
           mystery_result.shape_type.c_str(), mystery_result.size, mystery_result.rotation_angle);
  
////////////////////////////////////////////



 // matching mysterious object with reference
 size_t match_idx = 0;
 bool found_match = false;
 
 for (size_t idx = 0; idx < reference_shapes.size(); ++idx) {
   // Shape matching only
   if (mystery_result.shape_type == reference_shapes[idx].shape_type) {
     ROS_INFO("Matched object at index: %zu (shape: %s)", 
              idx, reference_shapes[idx].shape_type.c_str());
     match_idx = idx;
     found_match = true;
     break;
   }
 }
 
 // Default to 0 if no matches found
 if (!found_match) {
   ROS_WARN("No matching shape found for mystery object (detected as %s), defaulting to index 0", 
            mystery_result.shape_type.c_str());
   match_idx = 0;
 }
 
 response.mystery_object_num = match_idx + 1;
 ROS_INFO("Task 2 complete: Mystery object matches reference object %zu", match_idx + 1);
  return true;

}


///////////////////////////////////////////////////////////////////////////////

bool
cw2::t3_callback(cw2_world_spawner::Task3Service::Request &request,
  cw2_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");
  removeAllCollisions();
  resetArm();
  // setConstraint();
  addPlane("plane");
  // setConstraint();




  combined_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
  setZConstraint(0.475, 1.2);
  combined_cloud = scanPlatform();
  // clearConstraint();
  // setConstraint();


  // 1. Use Voxel grid to down sample
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
  vg.setInputCloud(combined_cloud);
  vg.setLeafSize(0.002f, 0.002f, 0.002f); // 2mm voxel
  vg.filter(*cloud_filtered);
  ROS_INFO("Cloud size after voxel grid filtering: %zu", cloud_filtered->points.size());
  
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered_3 = cloud_filtered;

  // 4. Save the pointcloud file as pcd
  std::string pcd_filename = "task3_scan_result.pcd";
  pcl::io::savePCDFileASCII(pcd_filename, *cloud_filtered_3);
  ROS_INFO("Point cloud saved to %s", pcd_filename.c_str());
  
  ROS_INFO("====================================  Detected Objects  ====================================");
  
  std::vector<ShapeDetectionResult> detected_objects;

  // Call detect object to detect all objects
  clearObstacles();
  detectObjects(cloud_filtered_3, detected_objects, obstacles_); //识别到的坐标没有转回到世界坐标系

  // Add obstacles
  if (obstacles_.size() > 0) {
    ROS_INFO("Detected %zu obstacles", obstacles_.size());
    int id = 0;
    for (const auto& obstacle : obstacles_) {
      addObstacle(obstacle, id++);
    }
  } else {
    ROS_INFO("No obstacles detected");
  }
  
  
  // Process result
  response.total_num_shapes = detected_objects.size() - 1; // Exclude basket from detected objects
  ShapeDetectionResult picked_object, candidate_cross, candidate_nought, candidate_basket;
  int cross_count = 0;
  float cross_score = 0;
  std::vector<std::string> collision_cross;
  int nought_count = 0;
  float nought_score = 0;
  std::vector<std::string> collision_nought;

  int id = 0;
  std::vector<std::string> collision_tmp;
  
  // We try to pick up the shape with the highest score (i.e. the most accurate data)
  for (const auto& obj : detected_objects) {
    if (obj.shape_type == "cross") {
      cross_count++;
      collision_tmp = genCrossObj(obj.size, obj.centroid[0], obj.centroid[1], obj.rotation_angle, std::to_string(id++));
      if (obj.overlap_score >= cross_score) {
        cross_score = obj.overlap_score;
        candidate_cross = obj;
        collision_cross = collision_tmp;
      }
    } else if (obj.shape_type == "nought") {
      nought_count++;
      collision_tmp = genNoughtObj(obj.size, obj.centroid[0], obj.centroid[1], obj.rotation_angle, std::to_string(id++));
      if (obj.overlap_score >= nought_score) {
        nought_score = obj.overlap_score;
        candidate_nought = obj;
        collision_nought = collision_tmp;
      }
    } else if (obj.shape_type == "basket") {
      candidate_basket = obj;
    }
  }
  // Determine the candidate object
  if (cross_count >= nought_count) {
    response.num_most_common_shape = cross_count;
    ROS_INFO("Most common shape: cross");
    ROS_INFO("Cross count: %d", cross_count);
    ROS_INFO_STREAM("Score:" << cross_score);
    picked_object = candidate_cross;
  } else {
    response.num_most_common_shape = nought_count;
    ROS_INFO("Most common shape: nought");
    ROS_INFO("Nought count: %d", nought_count);
    ROS_INFO_STREAM("Score:" << nought_score);
    picked_object = candidate_nought;
  }
  
  // generate collision object
  std::vector<std::string> collision_obj_parts;
  if(picked_object.shape_type  == "cross"){
    collision_obj_parts = collision_cross;
  }
  else{
    collision_obj_parts = collision_nought;
  }

  // get grasp position
  InitPose grasp_pose;
  grasp_pose.position.x = picked_object.centroid[0];
  grasp_pose.position.y = picked_object.centroid[1];
  grasp_pose.position.z = 0.41; // Set Z height
  grasp_pose.orientation.x = 0.9239;
  grasp_pose.orientation.y = -0.3827;
  grasp_pose.orientation.z = 0.0;
  grasp_pose.orientation.w = 0.0;

  adjustPoseByShapeAndRotation(grasp_pose, picked_object.shape_type, picked_object.rotation_angle, picked_object.size);

  InitPose temp_pose;
  temp_pose.position.x = picked_object.centroid[0];
  temp_pose.position.y = picked_object.centroid[1];
  temp_pose.position.z = 0.41; // Set Z height


  geometry_msgs::Point candidate_basket_point;
  candidate_basket_point.x = candidate_basket.centroid[0];
  candidate_basket_point.y = candidate_basket.centroid[1];
  addBasket("basket", candidate_basket_point);
  candidate_basket_point.z = 0.2; // Set Z height

  clearConstraint();

  // move arm to above the object
  moveArm(temp_pose, false);

  // move arm to above the object
  moveArm(grasp_pose, false);


  // pick and place
  pickAndPlace(picked_object.shape_type, grasp_pose, candidate_basket_point, collision_obj_parts);

  
  ROS_INFO("Task 3 complete");
  return true;

}



void cw2::addObstacle(const Obstacle& obstacle, const int id) {
  
  // Create collision object
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "world";
  collision_object.id = "obstacle_" + std::to_string(id);

  // Use object primitive
  collision_object.primitives.push_back(obstacle.obstacle);
  collision_object.primitive_poses.push_back(obstacle.obstacle_pose);
  collision_object.operation = collision_object.ADD;
  
  // Add collision object to planner
  planning_scene_interface_.applyCollisionObject(collision_object);
  
  ROS_INFO("Added obstacle at [x: %f, y: %f, z: %f]", 
           obstacle.obstacle_pose.position.x, 
           obstacle.obstacle_pose.position.y, 
           obstacle.obstacle_pose.position.z);
}

void cw2::clearObstacles() {
  // Clear all collision obstacles
  obstacles_.clear();
  
  std::vector<std::string> object_ids;
  for(size_t i = 0; i < obstacles_.size(); i++) {
    object_ids.push_back("obstacle_" + std::to_string(i+1));
  }
  planning_scene_interface_.removeCollisionObjects(object_ids);
  
  ROS_INFO("Cleared all obstacles");
}

// Check and fix trajectroy timestamps (Sometimes they are not strictly ascending)
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
  
  // if time fix needed
  if (needs_fixing) {
    ROS_WARN("Timestamp issues detected, performing automatic fix");
    
    const double min_time_step = 0.01;
    
    // Ensure order
    for (size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i) {
      double new_time = i * min_time_step;
      trajectory.joint_trajectory.points[i].time_from_start = ros::Duration(new_time);
    }
    
    // Double check fixed time
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



////////////////////////////////////////////////////////////////////////////////
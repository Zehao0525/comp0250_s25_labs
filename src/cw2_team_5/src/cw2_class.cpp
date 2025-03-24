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
  target_pose.position.z = 0.5;


  set_constraint();
  bool mvstate = move_arm(target_pose);
  ROS_INFO("Move state: %d", mvstate);



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



  adjustPoseByShapeAndRotation(target_pose, shape_type, rot_degree);



  target_pose.position.z = 0.25;


  bool mvstate1 = move_arm(target_pose);
  ROS_INFO("Move state: %d", mvstate1);




  Init_Pose goal_pose;
  goal_pose.position = goal_point.point;
  goal_pose.position.z = 0.2;



  pick_and_place(shape_type, target_pose,  goal_pose.position);


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
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PCL_cloud = convertToPCL(latest_cloud, tf_listener_);
    
    // 过滤点云数据
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr obj_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    filterPointCloudByHeight(PCL_cloud, obj_cloud_ptr, 0.04, 0.08);
    
    // 平移点云至原点
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << -reference_point.point.x, -reference_point.point.y, -reference_point.point.z;
    pcl::transformPointCloud(*obj_cloud_ptr, *obj_cloud_ptr, transform);
    
    // 检测形状
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
  ShapeDetectionResult mystery_result = detectShapeRotation(obj_cloud_ptr);
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
 
 response.mystery_object_num = match_idx;
 ROS_INFO("Task 2 complete: Mystery object matches reference object %zu", match_idx);
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
  
  combined_cloud = scanPlatform();


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
  detect_objects(cloud_filtered_3, detected_objects);
  
  // 转换函数：将ShapeDetectionResult转换为DetectedObject
  auto convertToDetectedObject = [](const ShapeDetectionResult& result) -> DetectedObject {
    DetectedObject obj;
    obj.type = result.shape_type;
    
    // 从centroid转换位置
    obj.position.x = result.centroid[0];
    obj.position.y = result.centroid[1];
    obj.position.z = result.centroid[2];
    
    // 设置尺寸（简单估计）
    obj.w = result.size;
    obj.l = result.size;
    obj.h = result.size;
    
    // 默认颜色
    obj.r = 128; // 为了兼容性，我们设置默认颜色值
    obj.g = 128;
    obj.b = 128;
    
    return obj;
  };
  
  // 转换检测结果
  std::vector<DetectedObject> converted_objects;
  for (const auto& result : detected_objects) {
    converted_objects.push_back(convertToDetectedObject(result));
    
    // 输出检测结果信息
    ROS_INFO("Detected object: %s", result.shape_type.c_str());
    ROS_INFO("Position: [x: %f, y: %f, z: %f]", 
             result.centroid[0], result.centroid[1], result.centroid[2]);
    ROS_INFO("Size: %f, Overlap score: %f", result.size, result.overlap_score);
  }
  
  // 处理结果
  response.total_num_shapes = detected_objects.size() - 1; // 排除篮子
  DetectedObject picked_object, candidate_cross, candidate_nought, candidate_basket;
  int cross_count = 0;
  int nought_count = 0;
  
  // 使用转换后的DetectedObject处理
  for (const auto& obj : converted_objects) {
    if (obj.type == "cross") {
      cross_count++;
      candidate_cross = obj;
    } else if (obj.type == "nought") {
      nought_count++;
      candidate_nought = obj;
    } else if (obj.type == "basket") {
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
  
  // 执行抓取和放置
  // 对位置进行调整以提高抓取成功率
  if (picked_object.type == "cross") { 
    picked_object.position.x += 0.05; 
  } else if (picked_object.type == "nought") { 
    picked_object.position.y += 0.08; 
  }
  
  // 执行抓取和放置操作
  pick_and_place(picked_object.type, picked_object.position, candidate_basket.position);
  
  ROS_INFO("Task 3 complete");
  
  return true;

}
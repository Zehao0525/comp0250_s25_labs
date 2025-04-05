#include "cw2_class.h"

void cw2::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg) {
  ptcoud_updated = true;
    latest_cloud = cloud_input_msg;
    
  }

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cw2::convertToPCL(
    sensor_msgs::PointCloud2ConstPtr cloud_msg, 
    tf::TransformListener& tf_listener, 
    const std::string& target_frame) 
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    try {
        tf_listener.waitForTransform(target_frame, cloud_msg->header.frame_id, ros::Time(0), ros::Duration(3.0));
        
        // Print pre transform coud info
        ROS_INFO("Original point cloud frame: %s", cloud_msg->header.frame_id.c_str());
        ROS_INFO("Target frame: %s", target_frame.c_str());
        ROS_INFO("Original point cloud size: %zu", pcl_cloud->points.size());

        pcl_ros::transformPointCloud(target_frame, *pcl_cloud, *transformed_cloud, tf_listener);

        // Print post transform cloud info
        ROS_INFO("Transformed point cloud frame: %s", target_frame.c_str());
        ROS_INFO("Transformed point cloud size: %zu", transformed_cloud->points.size());
    } catch (tf::TransformException &ex) {
        ROS_ERROR("Transform error: %s", ex.what());
        return pcl_cloud;  
    }
    return transformed_cloud;
}
  
void cw2::publishPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "world";
    point_cloud_pub_test_.publish(output);
    ROS_INFO("Published point cloud");
}

void cw2::publishPointCloudTimerCallback(const ros::TimerEvent& event) {
    if (publish_cloud_) {
      publishPointCloud(pcl_cloud_);
    }
  }


float cw2::calculateOverlapOld(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) {
  // Shift both clouds to origin
  translatePointCloudToOriginOld(cloud1);
  translatePointCloudToOriginOld(cloud2);
  
  // Create cloud 1's projection on the XY plane
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1_xy(new pcl::PointCloud<pcl::PointXYZRGBA>);
  cloud1_xy->points.resize(cloud1->points.size());
  for (size_t i = 0; i < cloud1->points.size(); ++i) {
    cloud1_xy->points[i] = cloud1->points[i];
    cloud1_xy->points[i].z = 0; 
  }
  cloud1_xy->width = cloud1->width;
  cloud1_xy->height = cloud1->height;
  cloud1_xy->is_dense = cloud1->is_dense;

  // Create KD tree
  pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
  kdtree.setInputCloud(cloud1_xy);

  // init intersection computation（intersection）
  int intersection_count = 0;
  float distance_threshold = 0.01;

  // Iterate through points in cloud 2
  for (const auto& point : cloud2->points) {
    std::vector<int> point_idx_radius_search;
    std::vector<float> point_radius_squared_distance;

    pcl::PointXYZRGBA search_point;
    search_point.x = point.x;
    search_point.y = point.y;
    search_point.z = 0;

    if (kdtree.radiusSearch(search_point, distance_threshold, point_idx_radius_search, point_radius_squared_distance) > 0) {
      intersection_count++;
    }
  }

  // Compute union
  int union_count = cloud1->points.size() + cloud2->points.size() - intersection_count;
  
  // Compute intersection over union
  float iou = static_cast<float>(intersection_count) / static_cast<float>(union_count);
  
  ROS_INFO("IoU: %.4f (Intersection: %d, Union: %d)", iou, intersection_count, union_count);
  
  return iou;
}


void cw2::filterPointCloudByHeight(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output_cloud, float min_height, float max_height) {
  pcl::PassThrough<pcl::PointXYZRGBA> pass;
  pass.setInputCloud(input_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(min_height, max_height);
  pass.filter(*output_cloud);
}

void cw2::translatePointCloudToOriginOld(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {
  // Compute centroid
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  
  // Compute shift to center
  float tx = -centroid[0];
  float ty = -centroid[1];
  float tz = -centroid[2];
  
  // Apply tranformation
  for (auto& point : cloud->points) {
    point.x += tx;
    point.y += ty;
    point.z += tz;
  }
  
}

void cw2::translatePointCloudToOriginOld(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  // Compute centroid
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  
  // Copute transform
  float tx = -centroid[0];
  float ty = -centroid[1];
  float tz = -centroid[2];
  
  // Apply transform
  for (auto& point : cloud->points) {
    point.x += tx;
    point.y += ty;
    point.z += tz;
  }
}









/**
 * Scanning the platform
 * @param platform_width 
 * @param platform_height 
 * @param scan_interval Distances between scans
 * @param min_height Height to perform the scan at
 * @param skip_center true
 * @param output_filename Output file name
 * @return Combined pointcloud
 */
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cw2::scanPlatform(
    float platform_width,
    float platform_height,
    float scan_interval,
    float scan_height,
    bool skip_center,
    const std::string& output_filename)
{
  // init return cloud
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  
  // Compuet scanned reigion
  float half_width = platform_width / 2.0;
  float half_height = platform_height / 2.0;
  
  
  // Grid scanning
  for (float x = scan_interval/2; x < platform_width; x += scan_interval) {
    for (float y = scan_interval/2; y < platform_height; y += scan_interval) {
      // Compute translation
      geometry_msgs::Point goal_point;
      goal_point.x = -half_width + x;
      goal_point.y = -half_height + y;
      goal_point.z = scan_height;
      
      // if skip center
      if (skip_center && 
          (goal_point.x < 0.2 && goal_point.x > -0.2) && 
          (goal_point.y < 0.25 && goal_point.y > -0.25)) {
        // ROS_INFO("跳过中心区域点: [%.2f, %.2f]", goal_point.x, goal_point.y);
        continue;
      }
      
      // Move to scan position
      InitPose target_pose;
      target_pose.position = goal_point;
      bool mvstate = moveArm(target_pose, false);
      
      // if move failed, don't scan
      if (!mvstate) {
        continue;
      }

      // Wait for the pointcloud to update
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      ptcoud_updated = false;
      while(!ptcoud_updated){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }

      
      // Obtain and process pointcloud info
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr current_cloud = 
          convertToPCL(latest_cloud, tf_listener_, "world");
      
      // combine pointcloud
      if (combined_cloud->empty()) {
        combined_cloud = current_cloud;
      } else {
        *combined_cloud += *current_cloud;
      }
      
    }
  }

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr combined_filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  filterPointCloudByHeight(combined_cloud, combined_filtered_cloud, 0.04, 0.48);
  
  // Save resut
  if (!output_filename.empty()) {
    pcl::io::savePCDFileASCII(output_filename, *combined_filtered_cloud);
    // ROS_INFO("点云已保存到: %s (总点数: %zu)", 
    //          output_filename.c_str(), combined_cloud->points.size());
  }
  
  return combined_filtered_cloud;
}


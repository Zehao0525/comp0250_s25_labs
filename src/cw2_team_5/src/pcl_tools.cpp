#include "cw2_class.h"

void cw2::point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg) {
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
        
        // 打印变换前的点云信息
        ROS_INFO("Original point cloud frame: %s", cloud_msg->header.frame_id.c_str());
        ROS_INFO("Target frame: %s", target_frame.c_str());
        ROS_INFO("Original point cloud size: %zu", pcl_cloud->points.size());

        pcl_ros::transformPointCloud(target_frame, *pcl_cloud, *transformed_cloud, tf_listener);

        // 打印变换后的点云信息
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


float cw2::calculateOverlap(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) {
  // 将两个点云平移到原点
  translatePointCloudToOrigin(cloud1);
  translatePointCloudToOrigin(cloud2);
  
  // 创建一个新的点云，将cloud1投影到XY平面上（Z=0）
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1_xy(new pcl::PointCloud<pcl::PointXYZRGBA>);
  cloud1_xy->points.resize(cloud1->points.size());
  for (size_t i = 0; i < cloud1->points.size(); ++i) {
    cloud1_xy->points[i] = cloud1->points[i];
    cloud1_xy->points[i].z = 0;  // 将Z坐标设为0
  }
  cloud1_xy->width = cloud1->width;
  cloud1_xy->height = cloud1->height;
  cloud1_xy->is_dense = cloud1->is_dense;

  // 创建一个KD树用于最近邻搜索
  pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
  kdtree.setInputCloud(cloud1_xy);

  // 计算交集（intersection）
  int intersection_count = 0;
  float distance_threshold = 0.01; // 距离阈值，单位：米

  // 遍历cloud2中的每个点，计算与cloud1中最近点的距离
  for (const auto& point : cloud2->points) {
    std::vector<int> point_idx_radius_search;
    std::vector<float> point_radius_squared_distance;

    pcl::PointXYZRGBA search_point;
    search_point.x = point.x;
    search_point.y = point.y;
    search_point.z = 0;  // Z坐标设为0

    if (kdtree.radiusSearch(search_point, distance_threshold, point_idx_radius_search, point_radius_squared_distance) > 0) {
      intersection_count++;
    }
  }

  // 计算并集（union）= 点云1大小 + 点云2大小 - 交集大小
  // 注意：这是一个近似值，因为我们没有精确地计算哪些点在cloud1中与cloud2重叠
  // 为了更准确的IoU计算，我们应该执行双向搜索，但这会增加计算成本
  int union_count = cloud1->points.size() + cloud2->points.size() - intersection_count;
  
  // 计算IoU(Intersection over Union)
  float iou = static_cast<float>(intersection_count) / static_cast<float>(union_count);
  
  ROS_INFO("IoU: %.4f (Intersection: %d, Union: %d)", iou, intersection_count, union_count);
  
  return iou;
}


void cw2::delete_groud_plane(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr out_cloud) {
  // 遍历输入点云中的每个点
  for (const auto& point : in_cloud->points) {
      // 检查点的RGB值是否为浅绿色
      if (!(point.r < 100 && point.g > 200 && point.b < 100)) {
          // 如果不是浅绿色，将点添加到输出点云中
          out_cloud->points.push_back(point);
      }
  }
  // 更新输出点云的宽度和高度
  out_cloud->width = out_cloud->points.size();
  out_cloud->height = 1;
  out_cloud->is_dense = true;
}

void cw2::filterPointCloudByHeight(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output_cloud, float min_height, float max_height) {
  pcl::PassThrough<pcl::PointXYZRGBA> pass;
  pass.setInputCloud(input_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(min_height, max_height);
  pass.filter(*output_cloud);
}

void cw2::translatePointCloudToOrigin(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {
  // 计算点云的几何中心
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  
  // 计算从中心点到原点的平移向量
  float tx = -centroid[0];
  float ty = -centroid[1];
  float tz = -centroid[2];
  
  // 应用平移
  for (auto& point : cloud->points) {
    point.x += tx;
    point.y += ty;
    point.z += tz;
  }
  
  // ROS_INFO("点云已平移: 中心(%f, %f, %f) -> 原点(0, 0, 0)", 
  //          centroid[0], centroid[1], centroid[2]);
}

void cw2::translatePointCloudToOrigin(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  // 计算点云的几何中心
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  
  // 计算从中心点到原点的平移向量
  float tx = -centroid[0];
  float ty = -centroid[1];
  float tz = -centroid[2];
  
  // 应用平移
  for (auto& point : cloud->points) {
    point.x += tx;
    point.y += ty;
    point.z += tz;
  }
  
  // ROS_INFO("点云已平移: 中心(%f, %f, %f) -> 原点(0, 0, 0)", 
  //          centroid[0], centroid[1], centroid[2]);
}









/**
 * 执行平台的全局扫描并合并点云数据
 * @param platform_width 平台宽度
 * @param platform_height 平台高度
 * @param scan_interval 扫描间隔
 * @param min_height 扫描高度
 * @param skip_center 是否跳过中心区域
 * @param output_filename 输出文件名
 * @return 合并后的点云
 */
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cw2::scanPlatform(
    float platform_width,
    float platform_height,
    float scan_interval,
    float scan_height,
    bool skip_center,
    const std::string& output_filename)
{
  // 初始化新的点云对象
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  
  // 计算扫描区域
  float half_width = platform_width / 2.0;
  float half_height = platform_height / 2.0;
  
  // ROS_INFO("开始平台扫描 (宽度: %.2f, 高度: %.2f, 间隔: %.2f)", 
  //          platform_width, platform_height, scan_interval);
  
  // 网格扫描模式
  for (float x = scan_interval/2; x < platform_width; x += scan_interval) {
    for (float y = scan_interval/2; y < platform_height; y += scan_interval) {
      // 计算实际坐标（平移到平台中心）
      geometry_msgs::Point goal_point;
      goal_point.x = -half_width + x;
      goal_point.y = -half_height + y;
      goal_point.z = scan_height;
      
      // 如果需要跳过中心区域
      if (skip_center && 
          (goal_point.x < 0.2 && goal_point.x > -0.2) && 
          (goal_point.y < 0.25 && goal_point.y > -0.25)) {
        // ROS_INFO("跳过中心区域点: [%.2f, %.2f]", goal_point.x, goal_point.y);
        continue;
      }
      
      // 移动到扫描位置
      Init_Pose target_pose;
      target_pose.position = goal_point;
      bool mvstate = move_arm(target_pose, false);
      // ROS_INFO("移动到: [%.2f, %.2f] %s", 
      //          goal_point.x, goal_point.y, 
      //          mvstate ? "成功" : "失败");
      
      if (!mvstate) {
        // ROS_WARN("移动失败，跳过该点");
        continue;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      ptcoud_updated = false;
      while(!ptcoud_updated){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }

      
      // 获取并处理点云数据
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr current_cloud = 
          convertToPCL(latest_cloud, tf_listener_, "world");
      
      // 合并点云
      if (combined_cloud->empty()) {
        combined_cloud = current_cloud;
      } else {
        *combined_cloud += *current_cloud;
      }
      
      // ROS_INFO("当前点云大小: %zu", combined_cloud->points.size());
    }
  }
  
  // 保存结果（如果指定了文件名）
  if (!output_filename.empty()) {
    pcl::io::savePCDFileASCII(output_filename, *combined_cloud);
    // ROS_INFO("点云已保存到: %s (总点数: %zu)", 
    //          output_filename.c_str(), combined_cloud->points.size());
  }
  
  return combined_cloud;
}


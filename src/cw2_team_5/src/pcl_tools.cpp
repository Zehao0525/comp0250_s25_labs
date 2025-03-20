#include "cw2_class.h"

void cw2::point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg) {
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

void cw2::reset_arm(){
    Init_Pose target_pose;
    target_pose.position.x = -0.1;
    target_pose.position.y = -0.1;
    target_pose.position.z = 0.5;
    move_arm(target_pose);
}

float cw2::calculateOverlap(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) {
  // 创建一个新的点云，复制cloud1但将所有Z坐标设为0
  translatePointCloudToOrigin(cloud1);
  translatePointCloudToOrigin(cloud2);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1_xy(new pcl::PointCloud<pcl::PointXYZRGBA>);
  cloud1_xy->points.resize(cloud1->points.size());
  for (size_t i = 0; i < cloud1->points.size(); ++i) {
    cloud1_xy->points[i] = cloud1->points[i];
    cloud1_xy->points[i].z = 0;  // 将Z坐标设为0
  }
  cloud1_xy->width = cloud1->width;
  cloud1_xy->height = cloud1->height;
  cloud1_xy->is_dense = cloud1->is_dense;

  // 创建一个KD树用于最近邻搜索，使用修改后的点云
  pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
  kdtree.setInputCloud(cloud1_xy);

  int overlap_count = 0;
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
      overlap_count++;
    }
  }

  // 计算重叠率
  float overlap_ratio = static_cast<float>(overlap_count) / static_cast<float>(cloud2->points.size());
  return overlap_ratio;
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
  
  ROS_INFO("点云已平移: 中心(%f, %f, %f) -> 原点(0, 0, 0)", 
           centroid[0], centroid[1], centroid[2]);
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
  
  ROS_INFO("点云已平移: 中心(%f, %f, %f) -> 原点(0, 0, 0)", 
           centroid[0], centroid[1], centroid[2]);
}
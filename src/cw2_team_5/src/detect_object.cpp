#include "cw2_class.h"
#include "detect_object.h"
std::mutex result_mutex;


// void detect_objects(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& in_cloud_ptr, std::vector<ShapeDetectionResult>& detected_objects) {
//     // 清空检测结果列表
//     detected_objects.clear();
//     bool have_basket = false;
    
//     // 使用拷贝的点云，以便在循环中可以修改它
//     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr working_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
//     *working_cloud = *in_cloud_ptr;
    
//     // 持续检测物体，直到无法找到更多物体为止
//     while (working_cloud->points.size() > 1000) {  // 最小聚类大小
//         // --- 1. 点云聚类 ---
//         pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
//         tree->setInputCloud(working_cloud);
        
//         std::vector<pcl::PointIndices> cluster_indices;
//         pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBA> cec(true);   // 'true' 允许聚类大小过滤
//         cec.setInputCloud(working_cloud);
//         cec.setConditionFunction(&enforce_color_similarity);
//         cec.setClusterTolerance(0.005f);          // 2cm空间聚类半径
//         cec.setMinClusterSize(1000);  
//         cec.setSearchMethod(tree); 
        
//         // 执行聚类
//         cec.segment(cluster_indices);
        
//         // 如果没有找到更多聚类，退出循环
//         if (cluster_indices.empty()) {
//             break;
//         }

//         // 按照聚类的点云数量排序（从大到小）
//         std::sort(cluster_indices.begin(), cluster_indices.end(), 
//             [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
//                 return a.indices.size() > b.indices.size();
//             });

//         bool found_object = false;

//         // 保留前10个聚类（如果有10个以上）
//         // if (cluster_indices.size() > 15) {
//         //     cluster_indices.resize(15);
//         // }

        
//         // --- 2. 处理每个聚类 ---
//         for (size_t i = 0; i < cluster_indices.size(); ++i)
//          {
//             const auto &indices = cluster_indices[i];
//             // 为当前聚类创建新的点云
//             pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
//             for (const auto &idx : indices.indices) {
//                 cluster->points.push_back(working_cloud->points[idx]);
//             }

//             cluster->width = cluster->points.size();
//             cluster->height = 1;
//             cluster->is_dense = true;

//             // 计算轴向边界框
//             pcl::PointXYZRGBA min_pt, max_pt;
//             pcl::getMinMax3D(*cluster, min_pt, max_pt);
      
//             Eigen::Vector3f dims(1.0f, 2.0f, 3.0f);
//             dims[0] = std::fabs(max_pt.x - min_pt.x);
//             dims[1] = std::fabs(max_pt.y - min_pt.y);
//             dims[2] = std::fabs(max_pt.z - min_pt.z);



//               // 计算聚类中心
//             Eigen::Vector4f cluster_centroid;
//             pcl::compute3DCentroid(*cluster, cluster_centroid);

//               /// save the point cloud to a file
//               /////////////////////////////////////////////////
//               pcl::io::savePCDFileASCII("cluster.pcd", *cluster);
//               /////////////////////////////////////////////////
      
//             geometry_msgs::Point object_point;
//             object_point.x = cluster_centroid[0];
//             object_point.y = cluster_centroid[1];
//             object_point.z = cluster_centroid[2];







//             if (!have_basket) { // 仅仅保留第一个检测到的篮子 最大的篮子
//               // --- 3. 检测篮子 ---
//               bool is_basket = checkBasket(cluster, dims, object_point, working_cloud, detected_objects);
//               if (is_basket) {
//                   found_object = true;
//                   have_basket = true;
//                   continue; // 跳到下一个聚类
//               }
//             }



//             ////////////////////检查颜色绝对值

//                 // 检查颜色一致性
//               uint8_t r_min = 255, r_max = 0, g_min = 255, g_max = 0, b_min = 255, b_max = 0;
              
//               // 查找颜色最大值和最小值
//               for (int j = 0; j < cluster->points.size(); j++) {
//                   uint32_t rgba = cluster->points[j].rgba;
//                   uint8_t r = (rgba >> 16) & 0x0000ff;
//                   uint8_t g = (rgba >> 8) & 0x0000ff;
//                   uint8_t b = rgba & 0x0000ff;
                  
//                   r_min = std::min(r_min, r);
//                   r_max = std::max(r_max, r);
//                   g_min = std::min(g_min, g);
//                   g_max = std::max(g_max, g);
//                   b_min = std::min(b_min, b);
//                   b_max = std::max(b_max, b);
//               }
              
//               // 计算最大颜色差异
//               uint8_t r_diff = r_max - r_min;
//               uint8_t g_diff = g_max - g_min;
//               uint8_t b_diff = b_max - b_min;
              
//               // 如果颜色差异过大，跳过该聚类
//               const uint8_t color_variance_threshold = 150; // 可调整的阈值
//               if (r_diff > color_variance_threshold || 
//                   g_diff > color_variance_threshold || 
//                   b_diff > color_variance_threshold) {
//                   ROS_INFO("jump large change color: R[%d-%d], G[%d-%d], B[%d-%d]", 
//                           r_min, r_max, g_min, g_max, b_min, b_max);
//                   continue; // 直接处理下一个聚类
//               }

//               /////////////////////////





//               // 输出每个聚类的平均rgb
//               int r = 0, g = 0, b = 0;
//               for (int j = 0; j < cluster->points.size(); j++) {
//                   uint32_t rgba = cluster->points[j].rgba;
//                   uint8_t uint8_r = (rgba >> 16) & 0x0000ff;
//                   uint8_t uint8_g = (rgba >> 8) & 0x0000ff;
//                   uint8_t uint8_b = (rgba) & 0x0000ff;
//                   uint8_t uint8_a = (rgba >> 24) & 0x000000ff;

//                   r += uint8_r;
//                   g += uint8_g;
//                   b += uint8_b;
//               }
//               r = r / cluster->points.size();
//               g = g / cluster->points.size();
//               b = b / cluster->points.size();

//               ROS_INFO("average color: R: %d, G: %d, B: %d", r, g, b);



//             // 添加尺寸阈值判断，跳过过大的聚类
//             const float max_size_threshold = 0.25f; // 设置最大尺寸阈值（单位：米）
//             if (dims[0] > max_size_threshold || dims[1] > max_size_threshold) {
//                 ROS_INFO("jump: dim %.3f x %.3f x %.3f", dims[0], dims[1], dims[2]);
//                 continue; // 直接处理下一个聚类
//             }      


//             if (r > 145 && r < 155 && g > 145 && g < 155 && b > 145 && b < 155) {
//               ROS_INFO("skip gray color");
//               continue;
//             }

//             if (r < 25 && g < 25 && b < 25) {
//               ROS_INFO("skip black color");
//               continue;
//             }

//             if (g > 127){
//               ROS_INFO("skip green color");
//               continue;
//             }

//             // --- 4. 检测其他形状 ---
//             ShapeDetectionResult detection_result = detectShapeRotation_multi(cluster);
            
//             // 添加到检测结果中
//             detected_objects.push_back(detection_result);
            
//             // 从工作点云中移除当前聚类
//             subtractPointCloud(working_cloud, cluster, 0.01);
            
//             found_object = true;
//         }
        
//         if (!found_object) {
//             break;  // 如果当前所有聚类都不是目标物体，退出循环
//         }
//     }
    
//     ROS_INFO("Detected %zu objects", detected_objects.size());
// }



// Condition function 
bool enforce_color_similarity(const pcl::PointXYZRGBA& a, const pcl::PointXYZRGBA& b, float squared_dist) {

  if (squared_dist > 0.02f * 0.02f) { // 2cm以内
  return false;}

  // 使用简单的RGB阈值判断，正负5以内视为相同颜色
  const float color_threshold = 10.0f;
  
  // 将RGBA值转换为0-255范围的整数进行比较
  uint32_t rgba_a = a.rgba;
  uint32_t rgba_b = b.rgba;
  
  uint8_t r_a = (rgba_a >> 16) & 0x0000ff;
  uint8_t g_a = (rgba_a >> 8) & 0x0000ff;
  uint8_t b_a = rgba_a & 0x0000ff;
  
  uint8_t r_b = (rgba_b >> 16) & 0x0000ff;
  uint8_t g_b = (rgba_b >> 8) & 0x0000ff;
  uint8_t b_b = rgba_b & 0x0000ff;

  
  return (std::abs(r_a - r_b) <= color_threshold) && 
         (std::abs(g_a - g_b) <= color_threshold) && 
         (std::abs(b_a - b_b) <= color_threshold);


}





//   void merge_clouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_cloud) {
//     std::lock_guard<std::mutex> lock(cloud_mutex);
//     if (combined_cloud->empty()) {
//         *combined_cloud = *new_cloud;
//     } else {
//         *combined_cloud += *new_cloud;
//     }
// }

/**
 * 从主点云中移除另一个点云的点
 * @param cloud_main 主点云，将被修改
 * @param cloud_to_remove 需要从主点云中移除的点云
 * @param distance_threshold 距离阈值，小于此值的点被认为是同一个点
 */
void subtractPointCloud(
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_main,
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_to_remove,
  double distance_threshold = 0.01)
{
  // 创建KdTree进行最近邻搜索
  pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
  kdtree.setInputCloud(cloud_to_remove);

  // 存储要保留的点的索引
  std::vector<int> indices_to_keep;

  // 对主点云中的每个点，检查它是否存在于要移除的点云中
  for (size_t i = 0; i < cloud_main->points.size(); ++i)
  {
      std::vector<int> pointIdxNKNSearch(1);
      std::vector<float> pointNKNSquaredDistance(1);
      
      // 查找最近的点
      int found = kdtree.nearestKSearch(cloud_main->points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance);
      
      // 如果没有找到点或者最近点的距离大于阈值，则保留该点
      if (found == 0 || std::sqrt(pointNKNSquaredDistance[0]) > distance_threshold)
      {
          indices_to_keep.push_back(i);
      }
  }

  // 创建新的点云来存储结果
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
  
  // 提取要保留的点
  pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  inliers->indices = indices_to_keep;
  
  extract.setInputCloud(cloud_main);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*cloud_filtered);
  
  // 将结果复制回主点云
  *cloud_main = *cloud_filtered;
}


ShapeDetectionResult detectShapeRotation(
  const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& obj_cloud_ptr, 
  float angle_step) 
{
  // 初始化结果结构体
  ShapeDetectionResult result;
  result.rotation_angle = -1.0;
  result.shape_type = "unknown";
  result.size = 0.0;
  result.overlap_score = 0.0;

  // 定义要测试的形状类型和尺寸
  std::vector<std::string> shape_types = {"cross", "nought"};
  std::vector<float> sizes = {0.02, 0.03, 0.04};
  
  // 全局最大重叠度
  float global_max_overlap = 0.0;
  
  // 测试所有形状类型和尺寸组合
  for (const auto& shape_type : shape_types) {
    for (const auto& size : sizes) {
      // 生成参考形状点云
      pcl::PointCloud<pcl::PointXYZ>::Ptr shape_checker_ptr;
      if (shape_type == "cross") {
        shape_checker_ptr = generateCrossShapePointCloud(size);
      } else if (shape_type == "nought") {
        shape_checker_ptr = generateOughtShapePointCloud(size);
      }
      
      // 本形状的最大重叠度和对应角度
      float shape_max_overlap = 0.0;
      float shape_best_angle = -1.0;
      
      // 测试0-90度范围内的所有角度
      for (float degrees = 0; degrees < 90; degrees += angle_step) {
        // 创建旋转变换
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf(degrees * M_PI / 180, Eigen::Vector3f::UnitZ()));
        
        // 旋转参考形状
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*shape_checker_ptr, *transformed_cloud, transform);
        
        // 计算重叠率
        float overlap = calculateOverlap(obj_cloud_ptr, transformed_cloud);
        
        // 更新本形状的最佳匹配
        if (overlap > shape_max_overlap) {
          shape_max_overlap = overlap;
          shape_best_angle = degrees;
        }
      }
      
      // 如果当前形状和尺寸的最大重叠度大于全局最大值，更新全局最佳匹配
      if (shape_max_overlap > global_max_overlap) {
        global_max_overlap = shape_max_overlap;
        result.rotation_angle = shape_best_angle;
        result.shape_type = shape_type;
        result.size = size;
        result.overlap_score = shape_max_overlap;
        
        ROS_INFO("New best match: type=%s, size=%.3f, angle=%.1f, overlap=%.3f", 
                 shape_type.c_str(), size, shape_best_angle, shape_max_overlap);
      }
    }
  }
  
  if (result.rotation_angle < 0) {
    ROS_WARN("No valid shape match found!");
  } else {
    ROS_INFO("Best shape match: type=%s, size=%.3f, rotation=%.1f deg, overlap=%.3f", 
             result.shape_type.c_str(), result.size, result.rotation_angle, result.overlap_score);
  }
  
  return result;
}



float calculateOverlap(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) {
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
  
  // 创建cloud2的XY投影
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_xy(new pcl::PointCloud<pcl::PointXYZ>);
  cloud2_xy->points.resize(cloud2->points.size());
  for (size_t i = 0; i < cloud2->points.size(); ++i) {
    cloud2_xy->points[i] = cloud2->points[i];
    cloud2_xy->points[i].z = 0;  // 将Z坐标设为0
  }
  cloud2_xy->width = cloud2->width;
  cloud2_xy->height = cloud2->height;
  cloud2_xy->is_dense = cloud2->is_dense;

  // 创建两个KD树用于双向搜索
  pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree1;
  kdtree1.setInputCloud(cloud1_xy);
  
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree2;
  kdtree2.setInputCloud(cloud2_xy);

  float distance_threshold = 0.01; // 距离阈值，单位：米
  
  // 第一步：从cloud2到cloud1搜索
  std::vector<bool> cloud2_has_match(cloud2->points.size(), false);
  int intersection_count_2to1 = 0;

  for (size_t i = 0; i < cloud2->points.size(); ++i) {
    std::vector<int> point_idx_radius_search;
    std::vector<float> point_radius_squared_distance;

    pcl::PointXYZRGBA search_point;
    search_point.x = cloud2->points[i].x;
    search_point.y = cloud2->points[i].y;
    search_point.z = 0;  // Z坐标设为0

    if (kdtree1.radiusSearch(search_point, distance_threshold, point_idx_radius_search, point_radius_squared_distance) > 0) {
      cloud2_has_match[i] = true;
      intersection_count_2to1++;
    }
  }
  
  // 第二步：从cloud1到cloud2搜索
  std::vector<bool> cloud1_has_match(cloud1->points.size(), false);
  int intersection_count_1to2 = 0;

  for (size_t i = 0; i < cloud1->points.size(); ++i) {
    std::vector<int> point_idx_radius_search;
    std::vector<float> point_radius_squared_distance;

    pcl::PointXYZ search_point;
    search_point.x = cloud1->points[i].x;
    search_point.y = cloud1->points[i].y;
    search_point.z = 0;  // Z坐标设为0

    if (kdtree2.radiusSearch(search_point, distance_threshold, point_idx_radius_search, point_radius_squared_distance) > 0) {
      cloud1_has_match[i] = true;
      intersection_count_1to2++;
    }
  }
  
  // 计算精确的交集与并集
  int cloud1_matched = std::count(cloud1_has_match.begin(), cloud1_has_match.end(), true);
  int cloud2_matched = std::count(cloud2_has_match.begin(), cloud2_has_match.end(), true);
  
  // // 并集计算 = 总点数 - 匹配的点数
  // int union_count = cloud1->points.size() + cloud2->points.size() - (cloud1_matched + cloud2_matched);
  // // 交集计算 = 匹配的点数
  int intersection_count = cloud1_matched + cloud2_matched;


  // int intersection_count = cloud1_matched; // 或 cloud2_matched
  int union_count = cloud1->points.size() + cloud2->points.size() - intersection_count;

  
  // 计算IoU(Intersection over Union)
  float iou = static_cast<float>(intersection_count) / static_cast<float>(union_count);
  
  // ROS_INFO("IoU(双向): %.4f (Intersection: %d, Union: %d)", iou, intersection_count, union_count);
  // ROS_INFO("cloud1匹配点: %d/%zu, cloud2匹配点: %d/%zu", 
  //          cloud1_matched, cloud1->points.size(),
  //          cloud2_matched, cloud2->points.size());
  
  return iou;
}


void translatePointCloudToOrigin(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
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

}

void translatePointCloudToOrigin(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {
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
  
}









void processShapeCombination(
  const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& obj_cloud_ptr,
  const std::string& shape_type,
  float size,
  float angle_step,
  ShapeDetectionResult& global_result)
{
  // 生成参考形状点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr shape_checker_ptr;
  if (shape_type == "cross") {
    shape_checker_ptr = generateCrossShapePointCloud(size);
  } else if (shape_type == "nought") {
    shape_checker_ptr = generateOughtShapePointCloud(size);
  }

  float shape_max_overlap = 0.0;
  float shape_best_angle = -1.0;

  for (float degrees = 0; degrees < 90; degrees += angle_step) {
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(degrees * M_PI / 180, Eigen::Vector3f::UnitZ()));

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*shape_checker_ptr, *transformed_cloud, transform);

    float overlap = calculateOverlap(obj_cloud_ptr, transformed_cloud);

    if (overlap > shape_max_overlap) {
      shape_max_overlap = overlap;
      shape_best_angle = degrees;
    }
  }

  // 计算点云的几何中心
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*obj_cloud_ptr, centroid);

  // 用锁保护全局最优结果的更新
  std::lock_guard<std::mutex> lock(result_mutex);
  if (shape_max_overlap > global_result.overlap_score) {
    global_result.rotation_angle = shape_best_angle;
    global_result.shape_type = shape_type;
    global_result.size = size;
    global_result.overlap_score = shape_max_overlap;
    global_result.centroid = centroid;


    ROS_INFO("New best match: type=%s, size=%.3f, angle=%.1f, overlap=%.3f", 
             shape_type.c_str(), size, shape_best_angle, shape_max_overlap);
  }
}

ShapeDetectionResult detectShapeRotation_multi(
  const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& obj_cloud_ptr,
  float angle_step)
{
  ShapeDetectionResult result;
  result.rotation_angle = -1.0;
  result.shape_type = "unknown";
  result.size = 0.0;
  result.overlap_score = 0.0;

  std::vector<std::string> shape_types = {"cross", "nought"};
  std::vector<float> sizes = {0.02, 0.03, 0.04};

  std::vector<std::thread> threads;

  // 启动6个线程，每个线程处理一个 shape + size 组合
  for (const auto& shape_type : shape_types) {
    for (const auto& size : sizes) {
      threads.emplace_back(processShapeCombination,
                           obj_cloud_ptr,
                           shape_type,
                           size,
                           angle_step,
                           std::ref(result));  // 传引用用于写入全局最优结果
    }
  }

  // 等待所有线程完成
  for (auto& t : threads) {
    t.join();
  }

  if (result.rotation_angle < 0) {
    ROS_WARN("No valid shape match found!");
  } else {
    ROS_INFO("Best shape match: type=%s, size=%.3f, rotation=%.1f deg, overlap=%.3f", 
             result.shape_type.c_str(), result.size, result.rotation_angle, result.overlap_score);
  }

  return result;
}







/**
 * 检查点云簇是否为篮子
 * @param cluster 待检查的点云簇
 * @param dims 点云簇的尺寸
 * @param object_point 点云簇的中心点
 * @param working_cloud 工作点云，如果检测到篮子将会从中移除该簇
 * @param detected_objects 检测到的物体列表，如果检测到篮子将添加到此列表
 * @return 如果检测到篮子返回true，否则返回false
 */
bool checkBasket(
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster,
  const Eigen::Vector3f& dims,
  const geometry_msgs::Point& object_point,
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr working_cloud,
  std::vector<ShapeDetectionResult>& detected_objects)
{
  // 定义篮子的目标颜色 (rgb: 0.5 0.2 0.2)
  uint32_t basket_color = (uint32_t)(0.5 * 255) << 16 | (uint32_t)(0.2 * 255) << 8 | (uint32_t)(0.2 * 255);
  const float color_tolerance = 0.1 * 255; // 颜色容差

  uint8_t basket_r = (basket_color >> 16) & 0x0000ff;
  uint8_t basket_g = (basket_color >> 8) & 0x0000ff;
  uint8_t basket_b = (basket_color) & 0x0000ff;

  for (int j = 0; j < cluster->points.size(); j++) {
      uint32_t point_color = cluster->points[j].rgba;
      uint8_t point_r = (point_color >> 16) & 0x0000ff;
      uint8_t point_g = (point_color >> 8) & 0x0000ff;
      uint8_t point_b = (point_color) & 0x0000ff;

      if (std::abs(point_r - basket_r) <= color_tolerance &&
          std::abs(point_g - basket_g) <= color_tolerance &&
          std::abs(point_b - basket_b) <= color_tolerance) {
          // 创建一个检测结果
          ShapeDetectionResult basket_result;
          basket_result.shape_type = "basket";
          basket_result.size = (dims[0] + dims[1] + dims[2]) / 3.0; // 简单尺寸表示
          basket_result.overlap_score = 1.0; // 篮子颜色匹配，给予高置信度
          
          // 设置位置
          Eigen::Vector4f centroid;
          pcl::compute3DCentroid(*cluster, centroid);
          basket_result.centroid = centroid;
          
          // 添加到检测结果中
          detected_objects.push_back(basket_result);
          
          // 从工作点云中移除当前聚类
          subtractPointCloud(working_cloud, cluster, 0.01);
          
          return true; // 找到篮子
      }
  }
  return false; // 未找到篮子
}

















void detect_objects(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& in_cloud_ptr, std::vector<ShapeDetectionResult>& detected_objects) {
    // 清空检测结果列表
    detected_objects.clear();


    // 清空./temp文件夹
    std::string temp_folder_path = "./temp";
    for (const auto& entry : std::filesystem::directory_iterator(temp_folder_path)) {
      std::filesystem::remove_all(entry.path());
    }

    bool have_basket = false;
    
    // 使用拷贝的点云，以便在循环中可以修改它
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr working_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    *working_cloud = *in_cloud_ptr;
    
    // 存储所有有效的聚类
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> valid_clusters;
    
    // 执行点云聚类
    clusterPointCloud(working_cloud, valid_clusters);
    
    ROS_INFO("Found %zu valid clusters for processing", valid_clusters.size());
    
    // 对每个有效聚类进行处理
    for (size_t i = 0; i < valid_clusters.size(); ++i) {
        auto& cluster = valid_clusters[i];

        if (i > 30){break;} // 仅处理前16个聚类
        
        // 计算轴向边界框和聚类中心
        pcl::PointXYZRGBA min_pt, max_pt;
        pcl::getMinMax3D(*cluster, min_pt, max_pt);
        
        Eigen::Vector3f dims(1.0f, 2.0f, 3.0f);
        dims[0] = std::fabs(max_pt.x - min_pt.x);
        dims[1] = std::fabs(max_pt.y - min_pt.y);
        dims[2] = std::fabs(max_pt.z - min_pt.z);
        
        Eigen::Vector4f cluster_centroid;
        pcl::compute3DCentroid(*cluster, cluster_centroid);
        
        geometry_msgs::Point object_point;
        object_point.x = cluster_centroid[0];
        object_point.y = cluster_centroid[1];
        object_point.z = cluster_centroid[2];


        ///////////////////////

        pcl::io::savePCDFileASCII("cluster.pcd", *cluster);
        ////////////////////////
        
        // 检查是否是篮子（仅检查第一个篮子）
        if (!have_basket) {
            bool is_basket = checkBasket(cluster, dims, object_point, working_cloud, detected_objects);
            if (is_basket) {
                have_basket = true;
                continue;
            }
        }
        
        // 检查是否是其他形状
        if (isValidObjectCluster(cluster, dims)) {
            // 形状识别
            ShapeDetectionResult detection_result = detectShapeRotation_multi(cluster);
            if (detection_result.overlap_score < 2) {
                ROS_INFO("Invalid shape detection result, skipping");
                continue;
            }
            detected_objects.push_back(detection_result);

            pcl::io::savePCDFileASCII("./temp/cluster" + std::to_string(i) + ".pcd", *cluster);

        }
    }
    
    ROS_INFO("Detected %zu objects", detected_objects.size());
}

// 点云聚类函数
void clusterPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud, 
                      std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>& valid_clusters) {
    // 清空输出集合
    valid_clusters.clear();
    
    // 创建KdTree用于聚类
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud(cloud);
    
    // 执行条件聚类
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBA> cec(true);  // 'true'允许聚类大小过滤
    cec.setInputCloud(cloud);
    cec.setConditionFunction(&enforce_color_similarity);
    cec.setClusterTolerance(0.005f);  // 5mm空间聚类半径
    cec.setMinClusterSize(1000);  // 最小聚类大小
    cec.setSearchMethod(tree);
    
    // 执行聚类
    cec.segment(cluster_indices);
    
    // 如果没有找到聚类
    if (cluster_indices.empty()) {
        ROS_WARN("No clusters found in the point cloud");
        return;
    }
    
    // 按照聚类的点云数量排序（从大到小）
    std::sort(cluster_indices.begin(), cluster_indices.end(), 
        [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
            return a.indices.size() > b.indices.size();
        });
    
    // 为每个聚类创建点云
    for (const auto& indices : cluster_indices) {
        // 创建新的点云
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
        for (const auto& idx : indices.indices) {
            cluster->points.push_back(cloud->points[idx]);
        }
        
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;
        
        valid_clusters.push_back(cluster);
    }
}

// 判断聚类是否是有效的对象聚类
bool isValidObjectCluster(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cluster, const Eigen::Vector3f& dims) {
    // 检查颜色一致性
    uint8_t r_min = 255, r_max = 0, g_min = 255, g_max = 0, b_min = 255, b_max = 0;
    
    // 查找颜色最大值和最小值
    for (int j = 0; j < cluster->points.size(); j++) {
        uint32_t rgba = cluster->points[j].rgba;
        uint8_t r = (rgba >> 16) & 0x0000ff;
        uint8_t g = (rgba >> 8) & 0x0000ff;
        uint8_t b = rgba & 0x0000ff;
        
        r_min = std::min(r_min, r);
        r_max = std::max(r_max, r);
        g_min = std::min(g_min, g);
        g_max = std::max(g_max, g);
        b_min = std::min(b_min, b);
        b_max = std::max(b_max, b);
    }
    
    // 计算最大颜色差异
    uint8_t r_diff = r_max - r_min;
    uint8_t g_diff = g_max - g_min;
    uint8_t b_diff = b_max - b_min;
    
    // 如果颜色差异过大，则非有效聚类
    const uint8_t color_variance_threshold = 150;
    if (r_diff > color_variance_threshold || 
        g_diff > color_variance_threshold || 
        b_diff > color_variance_threshold) {
        ROS_INFO("Invalid cluster with large color variation: R[%d-%d], G[%d-%d], B[%d-%d]", 
                 r_min, r_max, g_min, g_max, b_min, b_max);
        return false;
    }
    
    // 计算聚类的平均颜色
    int r = 0, g = 0, b = 0;
    for (int j = 0; j < cluster->points.size(); j++) {
        uint32_t rgba = cluster->points[j].rgba;
        uint8_t uint8_r = (rgba >> 16) & 0x0000ff;
        uint8_t uint8_g = (rgba >> 8) & 0x0000ff;
        uint8_t uint8_b = (rgba) & 0x0000ff;
        
        r += uint8_r;
        g += uint8_g;
        b += uint8_b;
    }
    r = r / cluster->points.size();
    g = g / cluster->points.size();
    b = b / cluster->points.size();
    
    ROS_INFO("Cluster average color: R: %d, G: %d, B: %d", r, g, b);
    
    // 尺寸检查
    const float max_size_threshold = 0.25f;
    if (dims[0] > max_size_threshold || dims[1] > max_size_threshold) {
        ROS_INFO("Invalid cluster with dimensions: %.3f x %.3f x %.3f", dims[0], dims[1], dims[2]);
        return false;
    }
    
    // 颜色过滤
    if (r > 145 && r < 155 && g > 145 && g < 155 && b > 145 && b < 155) {
        ROS_INFO("Skipping gray color cluster");
        return false;
    }
    
    if (r < 25 && g < 25 && b < 25) {
        ROS_INFO("Skipping black color cluster");
        return false;
    }
    
    if (g > 100) {
        ROS_INFO("Skipping green color cluster");
        return false;
    }
    // rgb 0.5 0.2 0.2
    int tol = 3;
    if (r < 255*0.5+tol && r > 255*0.5-tol && g < 255*0.2+tol && g > 255*0.2-tol && b < 255*0.2+tol && b > 255*0.2-tol) {
        ROS_INFO("Skipping basket cluster");
        return false;
    }

    return true;
}
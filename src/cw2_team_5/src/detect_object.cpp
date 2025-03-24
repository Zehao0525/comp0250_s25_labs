#include "cw2_class.h"
#include "detect_object.h"
std::mutex result_mutex;



// void detect_objects(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& in_cloud_ptr, DetectedObject detected_object, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& obj_cloud_ptr) {
//     // --- 1. Cluster Extraction ---
//     pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
//     tree->setInputCloud(in_cloud_ptr);
    
//     // For each cluster, 
//     std::vector<pcl::PointIndices> cluster_indices;
//     pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBA> cec(true);   // 'true' to allow cluster size filtering
//     cec.setInputCloud(in_cloud_ptr);
//     cec.setConditionFunction(&enforce_color_similarity);
//     cec.setClusterTolerance(0.02f);                        // 5mm spatial cluster radius
//     cec.setMinClusterSize(25);  
//     cec.setSearchMethod(tree); 
  
//     // Cluster pointcloud
//     cec.segment(cluster_indices);
  
//     // Counters for cubes and baskets
//     int cube_count = 0, basket_count = 0;
  
//     geometry_msgs::Point obj_position_tmp;
    
//     // --- 2. Process each cluster ---
//     for (const auto &indices : cluster_indices) {
//       // Create a new cloud for the current cluster
//       pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
//       for (const auto &idx : indices.indices) {
//         cluster->points.push_back(in_cloud_ptr->points[idx]);
//       }
      
//       cluster->width = cluster->points.size();
//       cluster->height = 1;
//       cluster->is_dense = true;
  
//       // Compute the axis-aligned bounding box (assuming objects are well-aligned)
//       pcl::PointXYZRGBA min_pt, max_pt;
//       pcl::getMinMax3D(*cluster, min_pt, max_pt);
  
//       Eigen::Vector3f dims(1.0f, 2.0f, 3.0f);
//       dims[0] = std::fabs(max_pt.x - min_pt.x);
//       dims[1] = std::fabs(max_pt.y - min_pt.y);
//       dims[2] = std::fabs(max_pt.z - min_pt.z);
  
//       // --- 3. Classification based on dimensions ---
//       // Here we assume:
//       //   • A cube will have roughly equal sides of ~0.04 (with some tolerance)
//       //   • A basket (rectangular cuboid outline) will have outer dimensions ~0.1
//       // We find the coordinates of the object
  
//       // The x and y coordinates are the x and y, but z is the minimum z
//       Eigen::Vector4f cluster_centroid;
//       pcl::compute3DCentroid(*cluster, cluster_centroid);
  
//       geometry_msgs::Point cylinder_point;


//       cylinder_point.x = cluster_centroid[0];
//       cylinder_point.y = cluster_centroid[1];
//       cylinder_point.z = cluster_centroid[2];
  
//       const float tol = 0.01f; // tolerance in meters

//       Eigen::Vector3f dim20mm(0.02f, 0.02f, 0.04f);
//       Eigen::Vector3f dim30mm(0.03f, 0.03f, 0.04f);
//       Eigen::Vector3f dim40mm(0.04f, 0.04f, 0.04f);

//       Eigen::Vector3f scale(5, 5, 1);

//         bool is_dim20 = dims.isApprox(dim20mm.cwiseProduct(scale), tol);
//         bool is_dim30 = dims.isApprox(dim30mm.cwiseProduct(scale), tol);
//         bool is_dim40 = dims.isApprox(dim40mm.cwiseProduct(scale), tol);

//         ROS_INFO("dim20: %d, dim30: %d, dim40: %d", is_dim20, is_dim30, is_dim40);
//         // std::cout << "Detected object with dimensions: " << dim20mm.cwiseProduct(scale) << std::endl;
//         ROS_INFO("Detected object with dimensions: %f, %f, %f", dims[0], dims[1], dims[2]);

//         // save the point cloud to a file
//         // pcl::io::savePCDFileASCII("mystery_point.pcd", *cluster);

//         if (is_dim20 || is_dim30 || is_dim40) {
//           DetectedObject cur_obj;
//           cur_obj.w = dims[0];
//           cur_obj.l = dims[1];
//           cur_obj.h = dims[2];
//           cur_obj.position = cylinder_point;
//           detected_object = cur_obj;
//           obj_cloud_ptr = cluster;
//           break ;
//         } 
    
//     } // end for loop

//     // already filled w l h position
//       if (obj_cloud_ptr != nullptr)
//       {    // check shape
//         pcl::PointXYZ query_point;
//         query_point.x = detected_object.position.x;
//         query_point.y = detected_object.position.y;
//         query_point.z = detected_object.position.z;
      
      
//         pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
      
//         pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
//         pcl::copyPointCloud(*obj_cloud_ptr, *obj_cloud_xyz);
//         kdtree.setInputCloud(obj_cloud_xyz);
      
//         std::vector<int> pointIdxNKNSearch(1);
//         std::vector<float> pointNKNSquaredDistance(1);
      
//         kdtree.nearestKSearch(query_point, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
//         double distance = std::sqrt(pointNKNSquaredDistance[0]);
      
//         if (distance < 0.02) {
//           ROS_INFO("This is the cross");
//           detected_object.type = "cross";
//         } else {
//           ROS_INFO("This is the nought");
//             detected_object.type = "nought";
//         }
//     // end check shape


//     // now we need to fill the color
    
//     int r = 0, g = 0, b = 0;
//     // retrieve the RGB data
//     for (int j = 0; j < obj_cloud_ptr->points.size(); j++) {
//       uint32_t rgba = obj_cloud_ptr->points[j].rgba;
//       uint8_t uint8_r = (rgba >> 16) & 0x0000ff;
//       uint8_t uint8_g = (rgba >> 8) & 0x0000ff;
//       uint8_t uint8_b = (rgba) & 0x0000ff;
//       uint8_t uint8_a = (rgba >> 24) & 0x000000ff;

//       r += uint8_r;
//       g += uint8_g;
//       b += uint8_b;
//     }
//     // take the average number of rgb of the image area
//     r = r / obj_cloud_ptr->points.size();
//     g = g / obj_cloud_ptr->points.size();
//     b = b / obj_cloud_ptr->points.size();

//     // implement color
//     detected_object.r = r;
//     detected_object.g = g;
//     detected_object.b = b;
//     }
//   }


// void detect_objects(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& in_cloud_ptr, DetectedObject detected_object){
//   std::vector<DetectedObject> detected_objects;
//   detect_objects(in_cloud_ptr, detected_objects);
//   if (detected_objects.size() > 0){
//   detected_object = detected_objects[0];
//   }
// }

void detect_objects(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& in_cloud_ptr, std::vector<ShapeDetectionResult>& detected_objects) {
    // 清空检测结果列表
    detected_objects.clear();
    
    // 使用拷贝的点云，以便在循环中可以修改它
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr working_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    *working_cloud = *in_cloud_ptr;
    
    // 持续检测物体，直到无法找到更多物体为止
    while (working_cloud->points.size() > 25) {  // 最小聚类大小
        // --- 1. 点云聚类 ---
        pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
        tree->setInputCloud(working_cloud);
        
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBA> cec(true);   // 'true' 允许聚类大小过滤
        cec.setInputCloud(working_cloud);
        cec.setConditionFunction(&enforce_color_similarity);
        cec.setClusterTolerance(0.02f);          // 2cm空间聚类半径
        cec.setMinClusterSize(25);  
        cec.setSearchMethod(tree); 
        
        // 执行聚类
        cec.segment(cluster_indices);
        
        // 如果没有找到更多聚类，退出循环
        if (cluster_indices.empty()) {
            break;
        }

        // 按照聚类的点云数量排序（从大到小）
        std::sort(cluster_indices.begin(), cluster_indices.end(), 
            [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
                return a.indices.size() > b.indices.size();
            });

        bool found_object = false;
        
        // --- 2. 处理每个聚类 ---
        for (const auto &indices : cluster_indices) {
            // 为当前聚类创建新的点云
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
            for (const auto &idx : indices.indices) {
                cluster->points.push_back(working_cloud->points[idx]);
            }
            
            cluster->width = cluster->points.size();
            cluster->height = 1;
            cluster->is_dense = true;
      
            // 计算轴向边界框
            pcl::PointXYZRGBA min_pt, max_pt;
            pcl::getMinMax3D(*cluster, min_pt, max_pt);
      
            Eigen::Vector3f dims(1.0f, 2.0f, 3.0f);
            dims[0] = std::fabs(max_pt.x - min_pt.x);
            dims[1] = std::fabs(max_pt.y - min_pt.y);
            dims[2] = std::fabs(max_pt.z - min_pt.z);
      
            // 计算聚类中心
            Eigen::Vector4f cluster_centroid;
            pcl::compute3DCentroid(*cluster, cluster_centroid);
      
            geometry_msgs::Point object_point;
            object_point.x = cluster_centroid[0];
            object_point.y = cluster_centroid[1];
            object_point.z = cluster_centroid[2];

            // --- 3. 检测篮子 ---
            bool is_basket = checkBasket(cluster, dims, object_point, working_cloud, detected_objects);
            if (is_basket) {
                found_object = true;
                continue; // 跳到下一个聚类
            }

            // --- 4. 检测其他形状 ---
            ShapeDetectionResult detection_result = detectShapeRotation_multi(cluster);
            
            // 添加到检测结果中
            detected_objects.push_back(detection_result);
            
            // 从工作点云中移除当前聚类
            subtractPointCloud(working_cloud, cluster, 0.01);
            
            found_object = true;
        }
        
        if (!found_object) {
            break;  // 如果当前所有聚类都不是目标物体，退出循环
        }
    }
    
    ROS_INFO("Detected %zu objects", detected_objects.size());
}

  // Condition function 
bool enforce_color_similarity(const pcl::PointXYZRGBA& a, const pcl::PointXYZRGBA& b, float /*squared_dist*/) {
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
  
  // 并集计算 = 总点数 - 匹配的点数
  int union_count = cloud1->points.size() + cloud2->points.size() - (cloud1_matched + cloud2_matched);
  // 交集计算 = 匹配的点数
  int intersection_count = cloud1_matched + cloud2_matched;
  
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
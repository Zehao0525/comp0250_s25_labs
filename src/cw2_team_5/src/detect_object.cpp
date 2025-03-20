#include "cw2_class.h"
#include "detect_object.h"


void detect_objects(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& in_cloud_ptr, DetectedObject detected_object, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& obj_cloud_ptr) {
    // --- 1. Cluster Extraction ---
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud(in_cloud_ptr);
    
    // For each cluster, 
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBA> cec(true);   // 'true' to allow cluster size filtering
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
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
      for (const auto &idx : indices.indices) {
        cluster->points.push_back(in_cloud_ptr->points[idx]);
      }
      
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;
  
      // Compute the axis-aligned bounding box (assuming objects are well-aligned)
      pcl::PointXYZRGBA min_pt, max_pt;
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
  
      geometry_msgs::Point cylinder_point;


      cylinder_point.x = cluster_centroid[0];
      cylinder_point.y = cluster_centroid[1];
      cylinder_point.z = cluster_centroid[2];
  
      const float tol = 0.01f; // tolerance in meters

      Eigen::Vector3f dim20mm(0.02f, 0.02f, 0.04f);
      Eigen::Vector3f dim30mm(0.03f, 0.03f, 0.04f);
      Eigen::Vector3f dim40mm(0.04f, 0.04f, 0.04f);

      Eigen::Vector3f scale(5, 5, 1);

        bool is_dim20 = dims.isApprox(dim20mm.cwiseProduct(scale), tol);
        bool is_dim30 = dims.isApprox(dim30mm.cwiseProduct(scale), tol);
        bool is_dim40 = dims.isApprox(dim40mm.cwiseProduct(scale), tol);

        ROS_INFO("dim20: %d, dim30: %d, dim40: %d", is_dim20, is_dim30, is_dim40);
        // std::cout << "Detected object with dimensions: " << dim20mm.cwiseProduct(scale) << std::endl;
        ROS_INFO("Detected object with dimensions: %f, %f, %f", dims[0], dims[1], dims[2]);

        // save the point cloud to a file
        // pcl::io::savePCDFileASCII("mystery_point.pcd", *cluster);

        if (is_dim20 || is_dim30 || is_dim40) {
          DetectedObject cur_obj;
          cur_obj.w = dims[0];
          cur_obj.l = dims[1];
          cur_obj.h = dims[2];
          cur_obj.position = cylinder_point;
          detected_object = cur_obj;
          obj_cloud_ptr = cluster;
          break ;
        } 
    
    } // end for loop

    // already filled w l h position
      if (obj_cloud_ptr != nullptr)
      {    // check shape
        pcl::PointXYZ query_point;
        query_point.x = detected_object.position.x;
        query_point.y = detected_object.position.y;
        query_point.z = detected_object.position.z;
      
      
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
      
        pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*obj_cloud_ptr, *obj_cloud_xyz);
        kdtree.setInputCloud(obj_cloud_xyz);
      
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
      
        kdtree.nearestKSearch(query_point, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
        double distance = std::sqrt(pointNKNSquaredDistance[0]);
      
        if (distance < 0.02) {
          ROS_INFO("This is the cross");
          detected_object.type = "cross";
        } else {
          ROS_INFO("This is the nought");
            detected_object.type = "nought";
        }
    // end check shape


    // now we need to fill the color
    
    int r = 0, g = 0, b = 0;
    // retrieve the RGB data
    for (int j = 0; j < obj_cloud_ptr->points.size(); j++) {
      uint32_t rgba = obj_cloud_ptr->points[j].rgba;
      uint8_t uint8_r = (rgba >> 16) & 0x0000ff;
      uint8_t uint8_g = (rgba >> 8) & 0x0000ff;
      uint8_t uint8_b = (rgba) & 0x0000ff;
      uint8_t uint8_a = (rgba >> 24) & 0x000000ff;

      r += uint8_r;
      g += uint8_g;
      b += uint8_b;
    }
    // take the average number of rgb of the image area
    r = r / obj_cloud_ptr->points.size();
    g = g / obj_cloud_ptr->points.size();
    b = b / obj_cloud_ptr->points.size();

    // implement color
    detected_object.r = r;
    detected_object.g = g;
    detected_object.b = b;
    }
  }


void detect_objects(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& in_cloud_ptr, DetectedObject detected_object){
  std::vector<DetectedObject> detected_objects;
  detect_objects(in_cloud_ptr, detected_objects);
  if (detected_objects.size() > 0){
  detected_object = detected_objects[0];
  }
}

void detect_objects(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& in_cloud_ptr, std::vector<DetectedObject>& detected_objects) {
    // 清空检测结果列表
    detected_objects.clear();
    
    // 使用拷贝的点云，以便在循环中可以修改它
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr working_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    *working_cloud = *in_cloud_ptr;
    
    // 持续检测物体，直到无法找到更多物体为止
    while (working_cloud->points.size() > 25) {  // 最小聚类大小
        // --- 1. Cluster Extraction ---
        pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
        tree->setInputCloud(working_cloud);
        
        // For each cluster
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBA> cec(true);   // 'true' to allow cluster size filtering
        cec.setInputCloud(working_cloud);
        cec.setConditionFunction(&enforce_color_similarity);
        cec.setClusterTolerance(0.02f);                        // 5mm spatial cluster radius
        cec.setMinClusterSize(25);  
        cec.setSearchMethod(tree); 
      
        // Cluster pointcloud
        cec.segment(cluster_indices);
        
        // 如果没有找到更多聚类，退出循环
        if (cluster_indices.empty()) {
            break;
        }

        // 按照聚类的点云数量排序 从达到小
        std::sort(cluster_indices.begin(), cluster_indices.end(), 
            [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
                return a.indices.size() > b.indices.size();
            });

        bool found_object = false;
        
        // --- 2. Process each cluster ---
        for (const auto &indices : cluster_indices) {
            // Create a new cloud for the current cluster
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
            for (const auto &idx : indices.indices) {
                cluster->points.push_back(working_cloud->points[idx]);
            }
            
            cluster->width = cluster->points.size();
            cluster->height = 1;
            cluster->is_dense = true;
      
            // Compute the axis-aligned bounding box
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
      
            const float tol = 0.01f; // tolerance in meters
            Eigen::Vector3f dim20mm(0.02f, 0.02f, 0.04f);
            Eigen::Vector3f dim30mm(0.03f, 0.03f, 0.04f);
            Eigen::Vector3f dim40mm(0.04f, 0.04f, 0.04f);
            Eigen::Vector3f scale(5, 5, 1);

            bool is_dim20 = dims.isApprox(dim20mm.cwiseProduct(scale), tol);
            bool is_dim30 = dims.isApprox(dim30mm.cwiseProduct(scale), tol);
            bool is_dim40 = dims.isApprox(dim40mm.cwiseProduct(scale), tol);



            ROS_INFO("dim20: %d, dim30: %d, dim40: %d", is_dim20, is_dim30, is_dim40);
            ROS_INFO("Detected object with dimensions: %f, %f, %f", dims[0], dims[1], dims[2]);

            if (is_dim20 || is_dim30 || is_dim40) {
                DetectedObject detected_obj;
                detected_obj.w = dims[0];
                detected_obj.l = dims[1];
                detected_obj.h = dims[2];
                detected_obj.position = object_point;
                
                // 检查形状
                pcl::PointXYZ query_point;
                query_point.x = detected_obj.position.x;
                query_point.y = detected_obj.position.y;
                query_point.z = detected_obj.position.z;
                
                pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
                
                pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::copyPointCloud(*cluster, *obj_cloud_xyz);
                kdtree.setInputCloud(obj_cloud_xyz);
                
                std::vector<int> pointIdxNKNSearch(1);
                std::vector<float> pointNKNSquaredDistance(1);
                
                kdtree.nearestKSearch(query_point, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
                double distance = std::sqrt(pointNKNSquaredDistance[0]);
                
                if (distance < 0.02) {
                    ROS_INFO("This is the cross");
                    detected_obj.type = "cross";
                } else {
                    ROS_INFO("This is the nought");
                    detected_obj.type = "nought";
                }
                
                // 填充颜色
                int r = 0, g = 0, b = 0;
                // 检索RGB数据
                for (int j = 0; j < cluster->points.size(); j++) {
                    uint32_t rgba = cluster->points[j].rgba;
                    uint8_t uint8_r = (rgba >> 16) & 0x0000ff;
                    uint8_t uint8_g = (rgba >> 8) & 0x0000ff;
                    uint8_t uint8_b = (rgba) & 0x0000ff;
                    
                    r += uint8_r;
                    g += uint8_g;
                    b += uint8_b;
                }
                // 取平均颜色
                r = r / cluster->points.size();
                g = g / cluster->points.size();
                b = b / cluster->points.size();
                
                // 设置对象颜色
                detected_obj.r = r;
                detected_obj.g = g;
                detected_obj.b = b;
                
                // 添加到检测结果中
                detected_objects.push_back(detected_obj);
                
                // 从工作点云中移除当前聚类
                subtractPointCloud(working_cloud, cluster, 0.01);
                
                found_object = true;
                break;
            }


            // check basket
            uint32_t basket_color = 0; // rgb: 0.5 0.2 0.2
            basket_color = (uint32_t)(0.5 * 255) << 16 | (uint32_t)(0.2 * 255) << 8 | (uint32_t)(0.2 * 255);

            const float color_tolerance = 0.1 * 255; // 颜色容差

            for (int j = 0; j < cluster->points.size(); j++) {
                uint32_t point_color = cluster->points[j].rgba;
                uint8_t point_r = (point_color >> 16) & 0x0000ff;
                uint8_t point_g = (point_color >> 8) & 0x0000ff;
                uint8_t point_b = (point_color) & 0x0000ff;

                uint8_t basket_r = (basket_color >> 16) & 0x0000ff;
                uint8_t basket_g = (basket_color >> 8) & 0x0000ff;
                uint8_t basket_b = (basket_color) & 0x0000ff;

                if (std::abs(point_r - basket_r) <= color_tolerance &&
                    std::abs(point_g - basket_g) <= color_tolerance &&
                    std::abs(point_b - basket_b) <= color_tolerance) {
                    DetectedObject detected_obj;
                    detected_obj.w = dims[0];
                    detected_obj.l = dims[1];
                    detected_obj.h = dims[2];
                    detected_obj.position = object_point;
                    detected_obj.type = "basket";
                    detected_obj.r = 0.5 * 255;
                    detected_obj.g = 0.2 * 255;
                    detected_obj.b = 0.2 * 255;
                    detected_objects.push_back(detected_obj);
                    
                    // 从工作点云中移除当前聚类
                    subtractPointCloud(working_cloud, cluster, 0.01);
                    
                    found_object = true;
                    break;
                }
            }
            

          
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
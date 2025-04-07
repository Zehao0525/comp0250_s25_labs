#include "cw2_class.h"
#include "detect_object.h"
std::mutex result_mutex;


// Condition function 
bool enforceColorSimilarity(const pcl::PointXYZRGBA& a, const pcl::PointXYZRGBA& b, float squared_dist) {

  if (squared_dist > 0.02f * 0.02f) { // 2cm以内
  return false;}

  // if two colors are +- 10 appart they are the same
  const float color_threshold = 10.0f;
  
  // Transform RGB
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





/**
 * Remove pointcloud from main pointcloud
 * @param cloud_main Modified main pointcloud
 * @param cloud_to_remove Pointcloud to remove
 * @param distance_threshold Threshold for two point to be considered the same
 */
void subtractPointCloud(
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_main,
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_to_remove,
  double distance_threshold = 0.01)
{
  // Create kd tree
  pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
  kdtree.setInputCloud(cloud_to_remove);

  // Save indecies to keep
  std::vector<int> indices_to_keep;

  // For every point in main pointcloud, chech if we should keep it
  for (size_t i = 0; i < cloud_main->points.size(); ++i)
  {
      std::vector<int> pointIdxNKNSearch(1);
      std::vector<float> pointNKNSquaredDistance(1);
      
      // find closest point
      int found = kdtree.nearestKSearch(cloud_main->points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance);
      
      // if no point less than threshod, keep point
      if (found == 0 || std::sqrt(pointNKNSquaredDistance[0]) > distance_threshold)
      {
          indices_to_keep.push_back(i);
      }
  }

  // Stor new pointcloud to keep resut
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
  
  // extract neded points
  pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  inliers->indices = indices_to_keep;
  
  extract.setInputCloud(cloud_main);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*cloud_filtered);
  
  // Copy result back to mian pointcloud
  *cloud_main = *cloud_filtered;
}




float calculateOverlap_backup(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr global_cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud2) {
  // Move two pointcouds to origin
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGBA>(*global_cloud1));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>(*global_cloud2));
  translatePointCloudToOrigin(cloud1);
  translatePointCloudToOrigin(cloud2);
  
  // project cloud 1 to xy plane
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1_xy(new pcl::PointCloud<pcl::PointXYZRGBA>);
  cloud1_xy->points.resize(cloud1->points.size());
  for (size_t i = 0; i < cloud1->points.size(); ++i) {
    cloud1_xy->points[i] = cloud1->points[i];
    cloud1_xy->points[i].z = 0;  // 将Z坐标设为0
  }
  cloud1_xy->width = cloud1->width;
  cloud1_xy->height = cloud1->height;
  cloud1_xy->is_dense = cloud1->is_dense;
  
  // project cloud 2 to xy plane
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_xy(new pcl::PointCloud<pcl::PointXYZ>);
  cloud2_xy->points.resize(cloud2->points.size());
  for (size_t i = 0; i < cloud2->points.size(); ++i) {
    cloud2_xy->points[i] = cloud2->points[i];
    cloud2_xy->points[i].z = 0;  // 将Z坐标设为0
  }
  cloud2_xy->width = cloud2->width;
  cloud2_xy->height = cloud2->height;
  cloud2_xy->is_dense = cloud2->is_dense;

  // KD trees for search
  pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree1;
  kdtree1.setInputCloud(cloud1_xy);
  
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree2;
  kdtree2.setInputCloud(cloud2_xy);

  float distance_threshold = 0.006; // voxel grids have size 0.002. max distance / diagnal being 0.0028. 
  
  // Search from cloud 2 to 1
  std::vector<bool> cloud2_has_match(cloud2->points.size(), false);
  int intersection_count_2to1 = 0;

  for (size_t i = 0; i < cloud2->points.size(); ++i) {
    std::vector<int> point_idx_radius_search;
    std::vector<float> point_radius_squared_distance;

    pcl::PointXYZRGBA search_point;
    search_point.x = cloud2->points[i].x;
    search_point.y = cloud2->points[i].y;
    search_point.z = 0;  // Set Z to 0

    if (kdtree1.radiusSearch(search_point, distance_threshold, point_idx_radius_search, point_radius_squared_distance) > 0) {
      cloud2_has_match[i] = true;
      intersection_count_2to1++;
    }
  }
  
  // Search from cloud 1 to 2
  std::vector<bool> cloud1_has_match(cloud1->points.size(), false);
  int intersection_count_1to2 = 0;

  for (size_t i = 0; i < cloud1->points.size(); ++i) {
    std::vector<int> point_idx_radius_search;
    std::vector<float> point_radius_squared_distance;

    pcl::PointXYZ search_point;
    search_point.x = cloud1->points[i].x;
    search_point.y = cloud1->points[i].y;
    search_point.z = 0;  // Z coord to 0

    if (kdtree2.radiusSearch(search_point, distance_threshold, point_idx_radius_search, point_radius_squared_distance) > 0) {
      cloud1_has_match[i] = true;
      intersection_count_1to2++;
    }
  }
  
  // Compute number of matches
  int cloud1_matched = std::count(cloud1_has_match.begin(), cloud1_has_match.end(), true);
  int cloud2_matched = std::count(cloud2_has_match.begin(), cloud2_has_match.end(), true);
  
  // Find the total number of points that has matches
  int total_matches = cloud1_matched + cloud2_matched;


  // int intersection_count = cloud1_matched; // 或 cloud2_matched
  int union_count = cloud1->points.size() + cloud2->points.size();

  
  // Compute the ratio of matches vs the sum of the size of the two pointclouds
  // We can't assume they are sets and compute union, because the points are not bijections
  float iou = static_cast<float>(total_matches) / static_cast<float>(union_count);

  return iou;
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
  // Compute centroid
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  
  // Compute required translation
  float tx = -centroid[0];
  float ty = -centroid[1];
  float tz = -centroid[2];
  
  // Apply translation
  for (auto& point : cloud->points) {
    point.x += tx;
    point.y += ty;
    point.z += tz;
  }

}

void translatePointCloudToOrigin(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {
  // Compute centroid
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  
  // Compute required translation
  float tx = -centroid[0];
  float ty = -centroid[1];
  float tz = -centroid[2];
  
  // Apply translation
  for (auto& point : cloud->points) {
    point.x += tx;
    point.y += ty;
    point.z += tz;
  }
  
}


void processShapeCombination_backup(
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

    float overlap = calculateOverlap_backup(obj_cloud_ptr, transformed_cloud);

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


void processShapeCombination(
  const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& obj_cloud_ptr,
  const std::string& shape_type,
  float size,
  float angle_step,
  ShapeDetectionResult& global_result)
{
  // Generate reference pointclouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr shape_checker_ptr;
  if (shape_type == "cross") {
    shape_checker_ptr = generateCrossShapePointCloud(size);
  } else if (shape_type == "nought") {
    shape_checker_ptr = generateOughtShapePointCloud(size);
  }


  // Search for the rough orientation of the shape using PCA
  // project to xy plane
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr obj_cloud_ptr_flat(new pcl::PointCloud<pcl::PointXYZRGBA>(*obj_cloud_ptr));
  for (size_t i = 0; i < obj_cloud_ptr->points.size(); ++i) {
    obj_cloud_ptr_flat->points[i] = obj_cloud_ptr->points[i];
    obj_cloud_ptr_flat->points[i].z = 0; 
  }

  // Find Principle component
  // First find covarience
  Eigen::Vector4f centroid;
  Eigen::Matrix<float, 3,3> cov_mat;
  pcl::compute3DCentroid(*obj_cloud_ptr_flat, centroid);
  pcl::computeCovarianceMatrixNormalized(*obj_cloud_ptr_flat, centroid, cov_mat);

  // Second get principle component
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(cov_mat);
  Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();
  Eigen::Vector3f principle_axis = eigen_vectors.col(2);

  float shape_best_angle_rough = std::atan2(principle_axis.y(), principle_axis.x()) * 180.0 / M_PI;
  shape_best_angle_rough = (shape_type == "cross") ? fmod(fmod(shape_best_angle_rough,90.0)+90.0,90.0) : fmod(fmod(shape_best_angle_rough + 45.0,90.0)+90.0,90.0);
  
  // initialize angle and overlap
  float shape_max_overlap = 0.0;
  float shape_best_angle = -1.0;
  float degrees = 0.0;
  
  // The principle axis aligns with noughts but not crosses
  // For crosses, it will be aligned with then diagnals of one of the two the 1*5 rectangles that froms the cross
  // arctan (1/5) is roughly 11.3 degrees. 
  // We test +11.3, -11.3, and 0
  for (float degrees_it = (shape_best_angle_rough-11.3); degrees_it < (shape_best_angle_rough+11.4); degrees_it += 11.3) {
    // Rotate shape
    degrees = fmod(fmod(degrees_it,90.0)+90.0,90.0);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(degrees * M_PI / 180, Eigen::Vector3f::UnitZ()));

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*shape_checker_ptr, *transformed_cloud, transform);

    // Checl overlap
    float overlap = calculateOverlap_backup(obj_cloud_ptr_flat, transformed_cloud);

    if (overlap > shape_max_overlap) {
      shape_max_overlap = overlap;
      shape_best_angle = degrees;
    }
  }

  // The above test assumes perfect pointcloud, but there may be noise
  // If our detection is good enough to be considered a "Hit", but not great in overlap, we do a finer search. 
  if(shape_max_overlap > 0.4 && shape_max_overlap < 0.8){
    // Iterate through finer search
    for (float degrees_it = (shape_best_angle_rough-12); degrees_it < (shape_best_angle_rough+12); degrees_it += angle_step) {
      // Exclude 0 
      if(degrees_it < 1e-8 && degrees_it > -1e-8){
        continue;
      }
      // Rotate pointcloud
      degrees = fmod(fmod(degrees_it,90.0)+90.0,90.0);
      Eigen::Affine3f transform = Eigen::Affine3f::Identity();
      transform.rotate(Eigen::AngleAxisf(degrees * M_PI / 180, Eigen::Vector3f::UnitZ()));
  
      pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::transformPointCloud(*shape_checker_ptr, *transformed_cloud, transform);
  
      // Compute overlap
      float overlap = calculateOverlap_backup(obj_cloud_ptr_flat, transformed_cloud);
  
      if (overlap > shape_max_overlap) {
        shape_max_overlap = overlap;
        shape_best_angle = degrees;
      }
    }
  }


  // use lock to protect global optimal update
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


ShapeDetectionResult detectShapeRotationMulti(
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

  // Activate 6 threads for each shape + size combination
  ROS_INFO("Initializing threads for all possible shapse and sizes...");
  for (const auto& shape_type : shape_types) {
    for (const auto& size : sizes) {
      threads.emplace_back(processShapeCombination_backup,
                           obj_cloud_ptr,
                           shape_type,
                           size,
                           angle_step,
                           std::ref(result));
    }
  }

  // Wait for all threads to finish
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
 * Check if pointcloud is basket
 * @param cluster clusters to check
 * @param dims 
 * @param object_point Centroid
 * @param working_cloud The combined pointcloud. We will remove the basket if we detect it.
 * @param detected_objects detected objects (insert if basket detected)
 * @return True if basket detected, false otherwise
 */
bool checkBasket(
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster,
  const Eigen::Vector3f& dims,
  const geometry_msgs::Point& object_point,
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr working_cloud,
  std::vector<ShapeDetectionResult>& detected_objects)
{
  // Define basket color (rgb: 0.5 0.2 0.2)
  uint32_t basket_color = (uint32_t)(0.5 * 255) << 16 | (uint32_t)(0.2 * 255) << 8 | (uint32_t)(0.2 * 255);
  const float color_tolerance = 0.1 * 255; // 颜色容差

  uint8_t basket_r = (basket_color >> 16) & 0x0000ff;
  uint8_t basket_g = (basket_color >> 8) & 0x0000ff;
  uint8_t basket_b = (basket_color) & 0x0000ff;

  // Iterate through all clusters
  for (int j = 0; j < cluster->points.size(); j++) {
      uint32_t point_color = cluster->points[j].rgba;
      uint8_t point_r = (point_color >> 16) & 0x0000ff;
      uint8_t point_g = (point_color >> 8) & 0x0000ff;
      uint8_t point_b = (point_color) & 0x0000ff;

      if (std::abs(point_r - basket_r) <= color_tolerance &&
          std::abs(point_g - basket_g) <= color_tolerance &&
          std::abs(point_b - basket_b) <= color_tolerance) {
          // Create detection result struct
          ShapeDetectionResult basket_result;
          basket_result.shape_type = "basket";
          basket_result.size = (dims[0] + dims[1] + dims[2]) / 3.0; // Size representation
          basket_result.overlap_score = 1.0; // Give high score if color matches
          
          // Find basket position
          Eigen::Vector4f centroid;
          pcl::compute3DCentroid(*cluster, centroid);
          basket_result.centroid = centroid;
          
          // Add basket to result
          detected_objects.push_back(basket_result);
          
          // Remove basket from pointcloud
          subtractPointCloud(working_cloud, cluster, 0.01);
          
          return true; 
      }
  }
  // no basket found
  return false; 
}

















void detectObjects(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& in_cloud_ptr, std::vector<ShapeDetectionResult>& detected_objects, 
                    std::vector<Obstacle>& obstacles){
    // Clear Obstacle list
    detected_objects.clear();

///////////////////////////////// C++17
    // Clear tmp file
    std::string temp_folder_path = "./temp";
    boost::filesystem::path dir(temp_folder_path);
//////////////////////////////////

    //////////////////////////////////////////////
    // create .tmp if it doesn't exist
    if (!boost::filesystem::exists(dir)) {
      ROS_INFO("Temp folder does not exist, creating: %s", temp_folder_path.c_str());
      boost::system::error_code ec;
      if (boost::filesystem::create_directory(dir, ec)) {
        ROS_INFO("Successfully created temp folder");
      } else {
        ROS_ERROR("Failed to create temp folder: %s", ec.message().c_str());
      }
    } else {
      // if file exist, clear content
      boost::filesystem::directory_iterator end_itr;
      for (boost::filesystem::directory_iterator itr(dir); itr != end_itr; ++itr) {
        boost::filesystem::remove_all(itr->path());
      }
      ROS_INFO("Cleared existing temp folder contents");
    }

    for (const auto& entry : std::filesystem::directory_iterator(temp_folder_path)) {
      std::filesystem::remove_all(entry.path());
    }
  /////////////////////////////////////////////


    bool have_basket = false;
    
    // Copy working pointcloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr working_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    *working_cloud = *in_cloud_ptr;
    
    // Save all cluster centroid
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> valid_clusters;
    std::vector<Eigen::Vector4f> cluster_centroids;

    
    // Cluster
    clusterPointCloud(working_cloud, valid_clusters, cluster_centroids);
    
    ROS_INFO("Found %zu valid clusters for processing", valid_clusters.size());
    
    // process each cluster
    for (size_t i = 0; i < valid_clusters.size(); ++i) {

        auto& cluster = valid_clusters[i];
        auto& centroid = cluster_centroids[i];


        // if (i > 25){break;} // Process first 25 clusters
        
        // Compute cluster size
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
        
        // Check basket
        if (!have_basket) {
            bool is_basket = checkBasket(cluster, dims, object_point, working_cloud, detected_objects);
            if (is_basket) {
                have_basket = true;

                continue;
            }
        }
        
        // Check if it is a valid cluster
        if (isValidObjectCluster(cluster, dims, obstacles)) {
            // Shape recognition
            ROS_INFO("Matching shape to orientation, processing...");
            ShapeDetectionResult detection_result = detectShapeRotationMulti(cluster);
            // At least 90% of the points in both pointclouds has to find matches
            // For we to consider this a Successful match
            if (detection_result.overlap_score < 0.6) {
                ROS_INFO("Invalid shape detection result, skipping");
                continue;
            }

            pcl::io::savePCDFileASCII("./temp/cluster" + std::to_string(i) + ".pcd", *cluster);

            detection_result.centroid = centroid;
            detected_objects.push_back(detection_result);

        }
    }
    


    ROS_INFO("Detected %zu objects", detected_objects.size() - 1);
}

// Cluster pointcloud
void clusterPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud, 
                      std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>& valid_clusters,
                      std::vector<Eigen::Vector4f>& cluster_centroids) {
    // Clear output
    valid_clusters.clear();
    cluster_centroids.clear();
    
    // Create KD tree 
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud(cloud);
    
    // Execute clustering
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBA> cec(true);  // 'true'允许聚类大小过滤
    cec.setInputCloud(cloud);
    cec.setConditionFunction(&enforceColorSimilarity);
    cec.setClusterTolerance(0.005f);  // 5mmPoints must be ess than 5mm appart
    cec.setMinClusterSize(500);  // min cluster size
    cec.setSearchMethod(tree);
    
    // Cluster
    cec.segment(cluster_indices);
    
    // If no clusters found
    if (cluster_indices.empty()) {
        ROS_WARN("No clusters found in the point cloud");
        return;
    }
    
    // Sort cluster (large to small)
    std::sort(cluster_indices.begin(), cluster_indices.end(), 
        [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
            return a.indices.size() > b.indices.size();
        });
    
    // Compute each cluster, create centroid
    for (const auto& indices : cluster_indices) {
        // Create new pointcloud
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
        for (const auto& idx : indices.indices) {
            cluster->points.push_back(cloud->points[idx]);
        }
        
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;
        
        // Compute cluster center
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster, centroid);
        
        // Add Pointcloud and centroid to result
        valid_clusters.push_back(cluster);
        cluster_centroids.push_back(centroid);
        
        ROS_INFO("Cluster %zu has %zu points, centroid at [%.3f, %.3f, %.3f]", 
                 cluster_centroids.size()-1, cluster->points.size(),
                 centroid[0], centroid[1], centroid[2]);
    }
}

// Determine if cluster is valid
bool isValidObjectCluster(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cluster, const Eigen::Vector3f& dims, std::vector<Obstacle>& obstacles) {
    // Color uniformity
    uint8_t r_min = 255, r_max = 0, g_min = 255, g_max = 0, b_min = 255, b_max = 0;
    
    // Find max and min color value
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
    
    // Find largest color diff
    uint8_t r_diff = r_max - r_min;
    uint8_t g_diff = g_max - g_min;
    uint8_t b_diff = b_max - b_min;
    
    // if color diff too large: invlaid cluster
    const uint8_t color_variance_threshold = 150;
    if (r_diff > color_variance_threshold || 
        g_diff > color_variance_threshold || 
        b_diff > color_variance_threshold) {
        ROS_INFO("Invalid cluster with large color variation: R[%d-%d], G[%d-%d], B[%d-%d]", 
                 r_min, r_max, g_min, g_max, b_min, b_max);
        return false;
    }
    
    // Compute average color
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
    
    // Check size
    const float max_size_threshold = 0.04*5*2;
    if (dims[0] > max_size_threshold || dims[1] > max_size_threshold) {
        ROS_INFO("Invalid cluster with dimensions: %.3f x %.3f x %.3f", dims[0], dims[1], dims[2]);
        return false;
    }
    

    const float min_size_threshold = 0.02*5/2; //(1.5 -> sqrt2)

    if (dims[0] < min_size_threshold || dims[1] < min_size_threshold) {
        ROS_INFO("Invalid cluster with dimensions: %.3f x %.3f x %.3f", dims[0], dims[1], dims[2]);
        return false;
    }


    // Filter color
    if (r > 145 && r < 155 && g > 145 && g < 155 && b > 145 && b < 155) {
        ROS_INFO("Skipping gray color cluster");
        return false;
    }
    
    if (r < 25 && g < 25 && b < 25) {
    // Record black obstacles
    pcl::PointXYZRGBA min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);
    
    // Create obstacle
    Obstacle obstacle_data;
    
    // Compute obstacle geometry
    obstacle_data.obstacle.type = shape_msgs::SolidPrimitive::BOX;
    obstacle_data.obstacle.dimensions.resize(3);
    obstacle_data.obstacle.dimensions[shape_msgs::SolidPrimitive::BOX_X] = max_pt.x - min_pt.x;
    obstacle_data.obstacle.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = max_pt.y - min_pt.y;
    obstacle_data.obstacle.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = max_pt.z - min_pt.z;
    
    // Compute obstacle center
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);
    
    // Compute obstacle pose
    obstacle_data.obstacle_pose.position.x = centroid[0];
    obstacle_data.obstacle_pose.position.y = centroid[1];
    obstacle_data.obstacle_pose.position.z = centroid[2];
    obstacle_data.obstacle_pose.orientation.w = 1.0; // 默认方向(无旋转)
    
    // Add obstacle to result
    obstacles.push_back(obstacle_data);
    
    ROS_INFO("Black obstacle detected. Dimensions: x=%.3f, y=%.3f, z=%.3f at position (%.3f, %.3f, %.3f)",
             obstacle_data.obstacle.dimensions[0], 
             obstacle_data.obstacle.dimensions[1], 
             obstacle_data.obstacle.dimensions[2],
             obstacle_data.obstacle_pose.position.x,
             obstacle_data.obstacle_pose.position.y,
             obstacle_data.obstacle_pose.position.z);
    
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




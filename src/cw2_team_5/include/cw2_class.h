/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw2_team_<your_team_number> package */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef cw2_CLASS_H_
#define cw2_CLASS_H_

// system includes
#include <ros/ros.h>

// include services from the spawner package - we will be responding to these
#include "cw2_world_spawner/Task1Service.h"
#include "cw2_world_spawner/Task2Service.h"
#include "cw2_world_spawner/Task3Service.h"



// standard c++ library includes (std::string, std::vector)
#include <stdlib.h>
#include <iostream>
#include <typeinfo>
#include <array>
#include <cmath>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <future>
#include <random>
#include <filesystem>

// system includes
#include <ros/ros.h>

// ros mesgs
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>

// TF specific includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>

// rosmoveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// pcl_ros transform and its implementation
// Apparently we need the implementation to run pcl ros with our point type
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>

// // include any services created in this package
// #include "cw2_team_x/example.h"

#include "data_structure.h"
#include "detect_object.h"
#include "shape_generator.h"

#include <boost/filesystem.hpp>
#include <boost/system/error_code.hpp>


class cw2
{
public:

  /* ----- class member functions ----- */

  // constructor
  cw2(ros::NodeHandle nh);

  // service callbacks for tasks 1, 2, and 3
  bool 
  t1_callback(cw2_world_spawner::Task1Service::Request &request,
    cw2_world_spawner::Task1Service::Response &response);
  bool 
  t2_callback(cw2_world_spawner::Task2Service::Request &request,
    cw2_world_spawner::Task2Service::Response &response);
  bool 
  t3_callback(cw2_world_spawner::Task3Service::Request &request,
    cw2_world_spawner::Task3Service::Response &response);

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg);
  void publishPointCloudTimerCallback(const ros::TimerEvent& event);

  /* ----- class member variables ----- */

  ros::NodeHandle nh_;
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;

  ros::Subscriber point_cloud_sub_;
  bool publish_cloud_;

  ros::Publisher point_cloud_pub_test_;
  ros::Timer point_cloud_timer_;
  void publishPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_cloud_;



  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};

  sensor_msgs::PointCloud2ConstPtr latest_cloud;
  bool ptcoud_updated = false;

  bool moveArm(geometry_msgs::Pose& target_pose, bool use_cartesian = false, int recursion_depth = 0, bool fast = true);
  
  pcl::PCLPointCloud2 pcl_pc_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertToPCL(
    sensor_msgs::PointCloud2ConstPtr cloud_msg, 
    tf::TransformListener& tf_listener, 
    const std::string& target_frame = "world");

  tf::TransformListener tf_listener_;

  std::string base_frame_ = "world";


  /** \brief MoveIt interface to interact with the moveit planning scene 
    * (eg collision objects). */
   moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

   moveit_msgs::Constraints constraints_;
   moveit_msgs::JointConstraint joint_constraints_[7];
   
   double gripper_open_ = 100e-3;
   double gripper_closed_ = 0.0;

  /////////////////////////////
  // move functions
  /////////////////////////////

  bool moveGripper(float width);
  void setConstraint();
  void clearConstraint();

  /// @brief function to add the ground plane into motion planning for collision avoidence.
  /// @param object_name std::string name of the plane added
  void 
  addPlane(const std::string& object_name);

  /// @brief function to add the baskets into motion planning for collision avoidence. 
  /// @param name std::string prefix for the objects added. 
  /// @param basket_pos geometry_msgs::Point location of the object.
  void
  addBasket(const std::string& name, const geometry_msgs::Point& basket_pos);


  /// @brief Movelt function to remove object in motion planning.
  /// @param object_name std::string name of object to be removed.
  void
  removeCollisionObject(const std::string& object_name);

  /// @brief Movelt function to add object into motion planning for collision avoidence. 
  /// @param object_name std::string name of object to add.
  /// @param centre geometry_msgs::Point centre position of the object.
  /// @param dimensions geometry_msgs::Vector3 dimensions of the object.
  /// @param orientation geometrymsgs::Quaternion orientation of the object.
  void
  addCollisionObject(const std::string& object_name, 
                      const geometry_msgs::Point& centre, 
                      const geometry_msgs::Vector3& dimensions, 
                      const geometry_msgs::Quaternion& orientation);


  /// \brief remove all collision objects
  void
  removeAllCollisions();

  void pickAndPlace(const std::string& obj_name, 
    const geometry_msgs::Point& obj_loc, 
    const geometry_msgs::Point& goal_loc,
    std::vector<std::string> collision_obj_parts) ;


  /////////////////
  // Adding Collision Object
  /////////////////
  void genRect(const std::string& name, const float x, const float y, const float dim_x, const float dim_y, const float theta);
  void removeCollisionObjects(std::vector<std::string> obj_names);
  std::vector<std::string> genNoughtObj(const float cell_size, const float x, const float y, const float theta, const std::string prefix = "");
  std::vector<std::string> genCrossObj(const float cell_size, const float x, const float y, const float theta, const std::string prefix = "");

  /////////////////
  // Temporary
  /////////////////
  // geometry_msgs::PointStamped cylinder_point_msg_;
  
    /** \brief RGB of red. */
    double red_color_[3] = {0.8, 0.1, 0.1};

    /** \brief RGB of purple. */
    double purple_color_[3] = {0.8, 0.1, 0.8};
  
    /** \brief RGB of blue. */
    double blue_color_[3] = {0.1, 0.1, 0.8};
  // std::string input_pc_frame_id_;


  // void merge_clouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_cloud);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr combined_cloud;
  float calculateOverlapOld(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr global_cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud2);
  void filterPointCloudByHeight(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output_cloud, float min_height, float max_height);
  void translatePointCloudToOriginOld(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
  void translatePointCloudToOriginOld(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  void pickAndPlace(const std::string& obj_name, const geometry_msgs::Pose& obj_loc, const geometry_msgs::Point& goal_loc, std::vector<std::string> collision_obj_parts) ;




  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scanPlatform(
    float platform_width = 1.5,
    float platform_height = 1,
    float scan_interval = 0.25,
    float scan_height = 0.55,
    bool skip_center = true,
    const std::string& output_filename = "task3_scan_result1.pcd");

  
void adjustPoseByShapeAndRotation(geometry_msgs::Pose& target_pose, 
  const std::string& shape_type, 
  float rot_degree,
  float obj_size = 0.02);

  bool isPointInWorkspace(const geometry_msgs::Pose& target_pose, bool check_collisions = true);


  void clearObstacles();
  void addObstacle(const Obstacle& obstacle, const int id);
  std::vector<Obstacle> obstacles_;  // 全局障碍物列表

  bool resetArm(double min_height = 0.4, double target_height = 0.41);
  bool clearAllConstraints();
  // void resetArm();
private:
  void setZConstraint(double min_height = 0.4, double max_height = 1.2);
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  void checkTrajectoryTimestamps(moveit_msgs::RobotTrajectory& trajectory);
};



#endif // end of include guard for cw2_CLASS_H_

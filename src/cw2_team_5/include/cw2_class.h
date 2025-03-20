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

  void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg);
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

  bool move_arm(geometry_msgs::Pose& target_pose, bool use_cartesian = false);
  
  pcl::PCLPointCloud2 pcl_pc_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertToPCL(
    sensor_msgs::PointCloud2ConstPtr cloud_msg, 
    tf::TransformListener& tf_listener, 
    const std::string& target_frame = "world");

  tf::TransformListener tf_listener_;

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

  bool move_gripper(float width);
  void set_constraint();
  void pick_and_place(const std::string& obj_name, 
    const geometry_msgs::Point& obj_loc, 
    const geometry_msgs::Point& goal_loc) ;

    void reset_arm();

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


  std::mutex cloud_mutex;
  // void merge_clouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_cloud);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr combined_cloud;
  float calculateOverlap(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);
  void delete_groud_plane(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr out_cloud);
  void filterPointCloudByHeight(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output_cloud, float min_height, float max_height);
  void translatePointCloudToOrigin(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
  void translatePointCloudToOrigin(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
 

};

#endif // end of include guard for cw2_CLASS_H_

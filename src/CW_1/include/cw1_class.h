/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef CW1_CLASS_H_
#define CW1_CLASS_H_

// system includes
#include <ros/ros.h>
#include <stdlib.h>

#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <typeinfo>
#include <array>
#include <cmath>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
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
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_clusters.h>


// TF specific includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// include services from the spawner package - we will be responding to these
#include "cw1_world_spawner/Task1Service.h"
#include "cw1_world_spawner/Task2Service.h"
#include "cw1_world_spawner/Task3Service.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

// // include any services created in this package
// #include "cw1_team_x/example.h"

class cw1
{
public:

  /* ----- class member functions ----- */

  /// @brief The constructor of the cw1 class
  /// @param nh ros::NodeHandle, the ros node 
  cw1(ros::NodeHandle nh);

  /// @brief Function which solve for task 1.
  /// @param request containing two inputs geometry_msgs/PoseStamped object_loc & goal_loc
  /// @param response empty response
  /// @return true
  bool 
  t1_callback(cw1_world_spawner::Task1Service::Request &request,
    cw1_world_spawner::Task1Service::Response &response);

  /// @brief Function which solve for task 2 - Check the existency and color of basket in some given points.
  /// @param request geometry_msgs/PointStamped[] basket_locs. 
  /// @param response string[] basket_colours.
  /// @return true     
  bool 
  t2_callback(cw1_world_spawner::Task2Service::Request &request,
    cw1_world_spawner::Task2Service::Response &response);
  
  /// @brief Function solve for task 3. Place cubes in corresponding color baskets.
  /// @param request empty request
  /// @param response empty response
  /// @return true  
  bool 
  t3_callback(cw1_world_spawner::Task3Service::Request &request,
    cw1_world_spawner::Task3Service::Response &response);

  /// @brief Movelt function to move last link (but not end effector) to a given pose.
  /// @param target_pose geometry_msgs::Pose target pose to approach
  /// @return true
  bool 
  moveArm(geometry_msgs::Pose target_pose);

  /// @brief Movelt function to open/close gripper.
  /// @param width float the width between two fingers.
  /// @return true
  bool 
  moveGripper(float width);

  /// @brief Movelt function to remove object in motion planning.
  /// @param object_name std::string name of object to be removed.
  void
  removeCollisionObject(std::string object_name);

  /// @brief Movelt function to add object into motion planning for collision avoidence. 
  /// @param object_name std::string name of object to add.
  /// @param centre geometry_msgs::Point centre position of the object.
  /// @param dimensions geometry_msgs::Vector3 dimensions of the object.
  /// @param orientation geometrymsgs::Quaternion orientation of the object.
  void
  addCollisionObject(std::string object_name, geometry_msgs::Point centre, 
    geometry_msgs::Vector3 dimensions, geometry_msgs::Quaternion orientation);

  /// @brief Callbacks to do filtering of the subscribed raw cloud point data (Task2).
  /// @param cloud_input_msg sensor_msgs::PointCloud2ConstPtr pointer to the subscribed input cloud point data.
  void
  cloudCallBackOne (const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg);

  /// @brief Callbacks to do filtering of the subscribed raw cloud point data (Task3).
  /// @param cloud_input_msg_task_3 sensor_msgs::PointCloud2ConstPtr pointer to the input cloud point data
  void
  cloudCallBackTwo (const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg_task_3);
  
  /// @brief Downsampling a PointCLoud using a VoxelGrid filter.
  /// @param in_cloud_ptr pointer to the raw point cloud
  /// @param out_cloud_ptr pointer to the filtered point cloud
  void
  applyVX (PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr);

  /// @brief Filtering a PointCloud using a PassThrough filter (i.e., cut along z direction).
  /// @param in_cloud_ptr pointer to the raw point cloud data.
  /// @param out_cloud_ptr pointer to the filtered Point Cloud data.
  void
  applyPT_x (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr);

  /// @brief Filtering a PointCloud using a PassThrough filter (i.e., cut along y direction).
  /// @param in_cloud_ptr pointer to the raw point cloud data.
  /// @param out_cloud_ptr pointer to the filtered Point Cloud data.
  void
  applyPT_y (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr);

  /// @brief Esimating surface normals in a PoiintCloud.
  /// @param in_cloud_ptr pointer to the point cloud investigated.
  void
  findNormals (PointCPtr &in_cloud_ptr);

  void
  cluster (PointCPtr &in_cloud_ptr);

  void
  segPlane (PointCPtr &in_cloud_ptr);

  void
  segCylind (PointCPtr &in_cloud_ptr);

  void
  findCylPose (PointCPtr &in_cloud_ptr);

  /// @brief Publish the filtered point cloud to be visualised on rviz.
  /// @param pc_pub certain publisher
  /// @param pc pointer to the filtered Point Cloud data.
  void
  pubFilteredPCMsg (ros::Publisher &pc_pub, PointC &pc);

  void
  publishPose (geometry_msgs::PointStamped &cyl_pt_msg);

  void
  target_pose_callback (const geometry_msgs::PointStamped &cyl_pt_msg);

  std::vector<geometry_msgs::Point>
  discard_repeat_pos (std::vector<geometry_msgs::Point> in_vector);

  void
  add_collision ();

  void
  print_cube_basket ();

  void
  add_plate ();

  void
  add_cube (std::vector<geometry_msgs::Point> in_vec, std::string name);

  void
  add_basket (std::vector<geometry_msgs::Point> in_vec, std::string name);

  void
  remove_all_collisions ();

  void
  pick_and_place (std::vector<geometry_msgs::Point> cube, 
                  std::vector<geometry_msgs::Point> basket,
                  std::string cube_name);

  /// @brief set constraint to the joint rotating angle
  void
  set_constraint ();

  /* ----- class member variables ----- */

  ros::NodeHandle nh_;
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;

  /** \brief Define some useful constant values. */
  std::string base_frame_ = "panda_link0";
  double gripper_open_ = 80e-3;
  double gripper_closed_ = 0.0;
  std::string basket_name = "basket", cube_name = "cube", plate_name = "plate";

  /** \brief MoveIt interface to move groups to seperate the arm and the gripper,
    * these are defined in urdf. */
  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};
  
  /** \brief MoveIt interface to interact with the moveit planning scene 
    * (eg collision objects). */
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;



  // variables for PCL 


  /** \brief RGB of red. */
  double red[3] = {0.8, 0.1, 0.1};

  /** \brief RGB of purple. */
  double purple[3] = {0.8, 0.1, 0.8};

  /** \brief RGB of blue. */
  double blue[3] = {0.1, 0.1, 0.8};

  /** \brief RGB data. */
  uint32_t rgba;

  /** \brief Node handle. */
  ros::NodeHandle g_nh;
    
  /** \brief The input point cloud frame id. */
  std::string g_input_pc_frame_id_;

  ros::Publisher g_pub_task_2;

  ros::Publisher g_pub_task_3;

  ros::Publisher pub_cluster;

  /** \brief ROS publishers. */
  ros::Publisher g_pub_cloud;
    
  /** \brief ROS geometry message point. */
  geometry_msgs::PointStamped g_cyl_pt_msg;
    
  /** \brief ROS pose publishers. */
  ros::Publisher g_pub_pose;
    
  /** \brief Voxel Grid filter's leaf size. */
  double g_vg_leaf_sz;
    
  /** \brief Point Cloud (input) pointer. */
  PointCPtr g_cloud_ptr;

  /** \brief Point Cloud (input) pointer. */
  PointCPtr g_cloud_ptr_task_3;
    
  /** \brief Point Cloud (filtered) pointer. */
  PointCPtr g_cloud_filtered, g_cloud_filtered2, g_cloud_filtered_task_3, g_cloud_filtered2_task_3;
    
  /** \brief Point Cloud (filtered) sensros_msg for publ. */
  sensor_msgs::PointCloud2 g_cloud_filtered_msg;
    
  /** \brief Point Cloud (input). */
  pcl::PCLPointCloud2 g_pcl_pc;

  /** \brief Point Cloud (input). */
  pcl::PCLPointCloud2 g_pcl_pc_task_3;
    
  /** \brief Voxel Grid filter. */
  pcl::VoxelGrid<PointT> g_vx;
    
  /** \brief Pass Through filter. */
  pcl::PassThrough<PointT> g_pt;
    
  /** \brief Pass Through min and max threshold sizes. */
  double g_pt_thrs_min_x, g_pt_thrs_max_x, g_pt_thrs_min_y, g_pt_thrs_max_y;
    
  /** \brief KDTree for nearest neighborhood search. */
  pcl::search::KdTree<PointT>::Ptr g_tree_ptr;
    
  /** \brief Normal estimation. */
  pcl::NormalEstimation<PointT, pcl::Normal> g_ne;
    
  /** \brief Cloud of normals. */
  pcl::PointCloud<pcl::Normal>::Ptr g_cloud_normals, g_cloud_normals2;
    
  /** \brief Nearest neighborhooh size for normal estimation. */
  double g_k_nn;
    
  /** \brief SAC segmentation. */
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> g_seg; 

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> g_ec;
    
  /** \brief Extract point cloud indices. */
  pcl::ExtractIndices<PointT> g_extract_pc;

  std::vector<pcl::PointIndices> cluster_indices;
  
  /** \brief Extract point cloud normal indices. */
  pcl::ExtractIndices<pcl::Normal> g_extract_normals;
    
  /** \brief Point indices for plane. */
  pcl::PointIndices::Ptr g_inliers_plane;
      
  /** \brief Point indices for cylinder. */
  pcl::PointIndices::Ptr g_inliers_cylinder;

  /** \brief Point indices for cylinder. */
  pcl::PointIndices::Ptr singal_cluster_ptr;

  pcl::PointCloud<PointT>::Ptr singal_cluster;
    
  /** \brief Model coefficients for the plane segmentation. */
  pcl::ModelCoefficients::Ptr g_coeff_plane;
    
  /** \brief Model coefficients for the culinder segmentation. */
  pcl::ModelCoefficients::Ptr g_coeff_cylinder;
    
  /** \brief Point cloud to hold plane and cylinder points. */
  PointCPtr g_cloud_plane, g_cloud_cylinder;
    
  /** \brief cw1Q1: TF listener definition. */
  tf::TransformListener g_listener_;

  std_msgs::Header cyl_pt_msg_header;

  geometry_msgs::Point cyl_pt_msg_point;

  geometry_msgs::Point task_3_position;

  moveit_msgs::Constraints constraints;

  moveit_msgs::JointConstraint joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7;

  bool segCylind_done = false;
  bool cluster_done = false;
  bool moveArm_done = false;
  bool cluster_or_not = false;
  bool task_3_triger = false;

  std::vector<geometry_msgs::Point> red_cube, blue_cube, purple_cube;
  std::vector<geometry_msgs::Point> red_basket, blue_basket, purple_basket;

    
protected:
    /** \brief Debug mode. */
    bool debug_;


};

#endif // end of include guard for CW1_CLASS_H_

/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef CW1_CLASS_H_
#define CW1_CLASS_H_

// standard c++ library includes (std::string, std::vector)
#include <stdlib.h>
#include <iostream>
#include <typeinfo>
#include <array>
#include <cmath>

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


// rosmoveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// include services from the spawner package - we will be responding to these
#include "cw1_world_spawner/Task1Service.h"
#include "cw1_world_spawner/Task2Service.h"
#include "cw1_world_spawner/Task3Service.h"

// // include any services created in this package
// #include "cw1_team_5/example.h"


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;



class cw1
{
public:

  /* ----- class member functions ----- */

  // constructor
  cw1(ros::NodeHandle nh);

  // ===============================================================================
  // service callbacks for tasks 1, 2, and 3
  // ===============================================================================
  bool 
  t1_callback(cw1_world_spawner::Task1Service::Request &request,
    cw1_world_spawner::Task1Service::Response &response);
  bool 
  t2_callback(cw1_world_spawner::Task2Service::Request &request,
    cw1_world_spawner::Task2Service::Response &response);
  bool 
  t3_callback(cw1_world_spawner::Task3Service::Request &request,
    cw1_world_spawner::Task3Service::Response &response);



  // ===============================================================================
  // Helper Functions
  // ===============================================================================

  /// @brief function to add the ground plane into motion planning for collision avoidence.
  /// @param name_prefix std::string name of the plane added
  void 
  addPlane(std::string object_name);

  /// @brief function to add the baskets into motion planning for collision avoidence. 
  /// @param name_prefix std::string prefix for the objects added (the object name will be prefix + index). 
  /// @param basket_locs std::vector<geometry_msgs::Point> location of the objects.
  void
  addBaskets(std::string name_prefix, std::vector<geometry_msgs::Point> basket_locs);
  void 
  addBaskets(std::string name_prefix, std::vector<geometry_msgs::PointStamped> basket_locs);

  /// @brief function to add the baskets into motion planning for collision avoidence. 
  /// @param name_prefix std::string prefix for the objects added (the object name will be prefix + index). 
  /// @param basket_locs std::vector<geometry_msgs::Point> location of the objects.
  void
  addCubes(std::string name_prefix, std::vector<geometry_msgs::Point> cube_locs);
  void
  addCubes(std::string name_prefix, std::vector<geometry_msgs::PointStamped> cube_locs);

  /// @brief function to add constraints. 
  void
  set_constraint ();


  /// @brief function to pick up an object and move it somewhere else. 
  /// @param obj_name std::string name of object to pick up.
  /// @param obj_loc geometry_msgs::Point the location of .
  /// @param goal_loc geometry_msgs::Point dimensions of the object.
  void
  pick_place_cube(std::string obj_name, geometry_msgs::Point obj_loc, geometry_msgs::Point goal_loc);


  /** \brief Point Cloud CallBack function.
  * 
  * \input[in] cloud_input a PointCloud2 sensor_msgs const pointer
  */
  void
  cloudCallBack (const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg);

  // ===============================================================================
  // Moveit functions
  // ===============================================================================

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


  /** \brief Apply Voxel Grid filtering.
  * 
  * \input[in] in_cloud_ptr the input PointCloud2 pointer
  * \input[out] out_cloud_ptr the output PointCloud2 pointer
  */
  void
  applyVX (PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr);





  /* ----- class member variables ----- */

  bool debug_;

  /** \brief RGB data. */
  uint32_t rgba;

  /** Config for pcl filter */
  double g_vg_leaf_sz;
  int g_k_nn;

  ros::NodeHandle nh_;
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;
  
  /** \brief Values used in the code */
  std::string base_frame_ = "panda_link0";
  double gripper_open_ = 80e-3;
  double gripper_closed_ = 0.0;
  std::string basket_name = "basket", cube_name = "cube", plane_name = "plane";

  /** \brief MoveIt interface to move groups to seperate the arm and the gripper,
    * these are defined in urdf. */
  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};

  /** \brief MoveIt interface to interact with the moveit planning scene 
    * (eg collision objects). */
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  moveit_msgs::Constraints constraints;
  moveit_msgs::JointConstraint joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7;
  
  /** --------------PCL Variables -------------- */

  /** \brief The input point cloud frame id. */
  std::string g_input_pc_frame_id_;
  
  /** \brief Voxel Grid filter. */
  pcl::VoxelGrid<PointT> g_vx;

  /** \brief Point Cloud (input). */
  PointCPtr g_cloud_ptr, g_cloud_filtered, g_cloud_filtered2;

  /** \brief Point Cloud (input). */
  pcl::PCLPointCloud2 g_pcl_pc;
    
  /** \brief KDTree for nearest neighborhood search. */
  pcl::search::KdTree<PointT>::Ptr g_tree_ptr;
};

#endif // end of include guard for CW1_CLASS_H_

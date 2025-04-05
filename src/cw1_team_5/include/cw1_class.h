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
#include <unordered_map>

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

// rosmoveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// include services from the spawner package - we will be responding to these
#include "cw1_world_spawner/Task1Service.h"
#include "cw1_world_spawner/Task2Service.h"
#include "cw1_world_spawner/Task3Service.h"

// pcl_ros transform and its implementation
// Apparently we need the implementation to run pcl ros with our point type
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

struct DetectedObject
{
  std::string type, color, name;
  geometry_msgs::Point position;
  float w,l,h,r,g,b;
};

// Condition function 
bool enforceColorSimilarity(const PointT& a, const PointT& b, float squared_dist);

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
  task1_callback(cw1_world_spawner::Task1Service::Request &request,
    cw1_world_spawner::Task1Service::Response &response);
  bool 
  task2_callback(cw1_world_spawner::Task2Service::Request &request,
    cw1_world_spawner::Task2Service::Response &response);
  bool 
  task3_callback(cw1_world_spawner::Task3Service::Request &request,
    cw1_world_spawner::Task3Service::Response &response);

  // ===============================================================================
  // Helper Functions
  // ===============================================================================
  
  /// @brief reinitialize all relavent variables
  void
  reset_task();

  /// @brief function to add the ground plane into motion planning for collision avoidence.
  /// @param object_name std::string name of the plane added
  void 
  add_plane(const std::string& object_name);

  /// @brief function to add the a collision representing all possible positions objects can spawn in
  /// @param object_name std::string name of the object added
  void 
  add_clarance(const std::string& object_name);

  /// @brief function to add the baskets into motion planning for collision avoidence. 
  /// @param name std::string prefix for the objects added. 
  /// @param basket_pos geometry_msgs::Point location of the object.
  void
  add_basket(const std::string& name, const geometry_msgs::Point& basket_pos);
  void
  add_baskets(const std::string& name_prefix, const std::vector<geometry_msgs::Point>& basket_locs);
  void 
  add_baskets(const std::string& name_prefix, const std::vector<geometry_msgs::PointStamped>& basket_locs);

  /// @brief function to add the cubes into motion planning for collision avoidence. 
  /// @param name std::string prefix for the objects added. 
  /// @param cube_loc geometry_msgs::Point location of the object.
  void 
  add_cube(const std::string& name, const geometry_msgs::Point& cube_loc);
  void
  add_cubes(const std::string& name_prefix, const std::vector<geometry_msgs::Point>& cube_locs);
  void
  add_cubes(const std::string& name_prefix, const std::vector<geometry_msgs::PointStamped>& cube_locs);

  /// @brief function to add constraints. 
  void
  setConstraint();

  /// @brief function to pick up an object and move it somewhere else. 
  /// @param obj_name std::string name of object to pick up.
  /// @param obj_loc geometry_msgs::Point the location of .
  /// @param goal_loc geometry_msgs::Point dimensions of the object.
  void
  pick_place_cube(const std::string& obj_name, 
                  const geometry_msgs::Point& obj_loc, 
                  const geometry_msgs::Point& goal_loc);

  /// @brief function to detect objects discovered by the point cloud. 
  /// @param in_cloud_ptr PointCPtr Input pointcloud
  /// @param detected_objects std::vector<DetectedObject> Output list of objects
  void 
  detectObjects(PointCPtr& in_cloud_ptr, std::vector<DetectedObject>& detected_objects);

  /// @brief Given cluster representing object, find its position 
  /// @param in_cloud_ptr PointCPtr Input pointcloud representing object
  /// @param transformed_cloud PointCPtr Output pointcloud In target_frame
  void
  convert_ptcld_to_world(PointCPtr& in_cloud_ptr, PointCPtr& transformed_cloud);

  /// @brief A function that will only exit after new pointcloud is recieved
  void
  wait_for_new_point_cloud();

  /** \brief Point Cloud CallBack function.
  * 
  * \input[in] cloud_input a PointCloud2 sensor_msgs const pointer
  */
  void
  cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg);

  // ===============================================================================
  // Moveit functions
  // ===============================================================================

  /// @brief Movelt function to move last link (but not end effector) to a given pose.
  /// @param target_pose geometry_msgs::Pose target pose to approach
  /// @return true
  bool 
  moveArm(geometry_msgs::Pose& target_pose);

  /// @brief Movelt function to open/close gripper.
  /// @param width float the width between two fingers.
  /// @return true
  bool 
  moveGripper(float width);

  /// @brief Movelt function to remove object in motion planning.
  /// @param object_name std::string name of object to be removed.
  void
  remove_collision_object(const std::string& object_name);

  /// @brief Movelt function to add object into motion planning for collision avoidence. 
  /// @param object_name std::string name of object to add.
  /// @param centre geometry_msgs::Point centre position of the object.
  /// @param dimensions geometry_msgs::Vector3 dimensions of the object.
  /// @param orientation geometrymsgs::Quaternion orientation of the object.
  void
  add_collision_object(const std::string& object_name, 
                     const geometry_msgs::Point& centre, 
                     const geometry_msgs::Vector3& dimensions, 
                     const geometry_msgs::Quaternion& orientation);


  /// \brief remove all collision objects
  void
  remove_all_collisions();


  /** \brief Apply Voxel Grid filtering.
  * 
  * \input[in] in_cloud_ptr the input PointCloud2 pointer
  * \input[out] out_cloud_ptr the output PointCloud2 pointer
  */
  void
  apply_voxel_filter(PointCPtr& in_cloud_ptr, PointCPtr& out_cloud_ptr);

  /* ----- class member variables ----- */

  bool debug_flag_;

  /** \brief RGB data. */
  uint32_t rgba_;

  /** Config for pcl filter */
  double vg_leaf_size_;
  int k_nn_;

  ros::NodeHandle nh_;
  ros::ServiceServer task1_service_;
  ros::ServiceServer task2_service_;
  ros::ServiceServer task3_service_;
  
  /** \brief Values used in the code */
  std::string base_frame_ = "panda_link0";
  double gripper_open_ = 80e-3;
  double gripper_closed_ = 0.0;
  std::string basket_name_ = "basket", cube_name_ = "cube", plane_name_ = "plane";

  /** \brief MoveIt interface to move groups to seperate the arm and the gripper,
    * these are defined in urdf. */
  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};

  /** \brief MoveIt interface to interact with the moveit planning scene 
    * (eg collision objects). */
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  moveit_msgs::Constraints constraints_;
  moveit_msgs::JointConstraint joint_constraints_[7];
  
  /** --------------PCL Variables -------------- */

  /** \brief The input point cloud frame id. */
  std::string input_pc_frame_id_;

  /** \brief The input point cloud time stamp. */
  std::uint64_t input_pc_time_;
  
  /** \brief Voxel Grid filter. */
  pcl::VoxelGrid<PointT> voxel_grid_;

  /** \brief Point Cloud (input). */
  PointCPtr cloud_ptr_, cloud_filtered_, cloud_filtered2_, cloud_world_, cloud_world_tmp_;
  
  // If the point cloud is dirty
  bool cloud_dirty_flag_;

  /** \brief Point Cloud (input). */
  pcl::PCLPointCloud2 pcl_pc_;
    
  /** \brief KDTree for nearest neighborhood search. */
  pcl::search::KdTree<PointT>::Ptr tree_ptr_;

  /** \brief Pass through filter for pointcloud simplification. */
  pcl::PassThrough<PointT> pass_filter_;

  /** -------------Additional ROS topics---------- */
  tf::TransformListener tf_listener_;

  /** --------------Task 3 valirbales ------------ */
  /** \brief ROS geometry message point. */
  // Used for temporary storage. Would this be optimized to here by C++ anyways?
  geometry_msgs::PointStamped cylinder_point_msg_;

  /** \brief RGB of red. */
  double red_color_[3] = {0.8, 0.1, 0.1};

  /** \brief RGB of purple. */
  double purple_color_[3] = {0.8, 0.1, 0.8};

  /** \brief RGB of blue. */
  double blue_color_[3] = {0.1, 0.1, 0.8};
};

#endif // end of include guard for CW1_CLASS_H_
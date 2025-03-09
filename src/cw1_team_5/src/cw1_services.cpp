#include "cw1_class.h"

// #include "add_or_remove_functions.cpp"
// #include "cw1_tools.cpp"

cw1::cw1(ros::NodeHandle nh) 
    : cloud_ptr_(new PointC),
      cloud_filtered_(new PointC),
      cloud_filtered2_(new PointC),
      cloud_world_(new PointC),
      cloud_world_tmp_(new PointC),
      tree_ptr_(new pcl::search::KdTree<PointT>()),
      debug_flag_(false),
      cloud_dirty_flag_(false) {
  
  nh_ = nh;

  task1_service_ = nh_.advertiseService("/task1_start", &cw1::task1_callback, this);
  task2_service_ = nh_.advertiseService("/task2_start", &cw1::task2_callback, this);
  task3_service_ = nh_.advertiseService("/task3_start", &cw1::task3_callback, this);

  vg_leaf_size_ = 0.01;
  k_nn_ = 50;

  ROS_INFO("CW1 class initialized.");
}

///////////////////////////////////////////////////////////////////////////////

bool cw1::task1_callback(cw1_world_spawner::Task1Service::Request &request,
                        cw1_world_spawner::Task1Service::Response &response) {
  /* function which should solve task 1 */

  ROS_INFO("The coursework solving callback for task 1 has been triggered");
  reset_task();
  // TODO: use the addCollisionObject function
  // Set the orientation and adjust translation

  // To provide leeway, we will inflate all objects by 0.01 in each diection (that matters)

  // create cup object
  // add the collision in the RViz
  // We examine from model.sdf.xmacro that the tiles span from:
  // 0.2 to 0.7 in the x direction, and -0.45 to 0.45 in the y direction
  // Thickness in the z direction is 0.01
  add_plane(plane_name_);  
  add_cube(cube_name_, request.object_loc.pose.position);
  add_basket(basket_name_, request.goal_loc.point);

  // geometry_msgs::Vector3 cube_dim;
  // cube_dim.x = 0.06;
  // cube_dim.y = 0.06;
  // cube_dim.z = 0.06;

  // geometry_msgs::Quaternion cube_ori;
  // cube_ori.w = 1;
  // cube_ori.x = 0;
  // cube_ori.y = 0;
  // cube_ori.z = 0;

  // // add the cube
  // addCollisionObject(cube_name, request.object_loc.pose.position,cube_dim,cube_ori);

  // // Define basket location
  // geometry_msgs::Vector3 basket_dim;
  // basket_dim.x = 0.12;
  // basket_dim.y = 0.12;
  // basket_dim.z = 0.12;

  // geometry_msgs::Quaternion basket_ori;
  // basket_ori.w = 1;
  // basket_ori.x = 0;
  // basket_ori.y = 0;
  // basket_ori.z = 0;

  // // add the basket
  // addCollisionObject(basket_name, request.goal_loc.point, basket_dim, basket_ori);

  set_constraint();

  // We specify waypoints for the arm trajectory
  geometry_msgs::Pose init_pos, target_pos, lift_pos, goal_pos;

  // We specify the location the arm need to reach to pick up the object
  target_pos.orientation.x = 0.9239;
  target_pos.orientation.y = -0.3827;
  target_pos.orientation.z = 0.0;
  target_pos.orientation.w = 0.0;
  target_pos.position.x = request.object_loc.pose.position.x;
  target_pos.position.y = request.object_loc.pose.position.y;
  target_pos.position.z = request.object_loc.pose.position.z + 0.125; //0.125 is the tested distance for the end effector to grab the cube firmly

  // We specify an intermediate state for the planner because it i stupid and could't find a clear path to goal
  lift_pos = target_pos;
  lift_pos.position.z = 0.18;

  // We define the pose above the goal for he arm to move to
  goal_pos = target_pos;
  goal_pos.position.x = request.goal_loc.point.x;
  goal_pos.position.y = request.goal_loc.point.y;
  goal_pos.position.z = 0.3;  // 0.1(cup height) + 0.125(height from edn effector to bottom of cube) + 0.075(leeway)

  pick_place_cube(cube_name_, target_pos.position, goal_pos.position);
  return true;

  // Open gripper
  bool opgrip_success = move_gripper(1.0);
  // Move into object position
  bool mvarm_success = move_arm(target_pos);
  // Close gripper
  remove_collision_object(cube_name_);
  bool clgrip_success = move_gripper(0.0);
  
  // Move to lifted position
  bool mvarm_success2 = move_arm(lift_pos);
  // Move to goal position
  bool mvarm_success3 = move_arm(goal_pos);
  // Open gripper
  bool opgrip_success2 = move_gripper(1.0);
  // Determine success
  //response.success = opgrip_success && mvarm_success && clgrip_success && mvarm_success2 && opgrip_success2;

  remove_collision_object(basket_name_);
  remove_collision_object(plane_name_);
  
  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool cw1::task2_callback(cw1_world_spawner::Task2Service::Request &request,
                        cw1_world_spawner::Task2Service::Response &response) {
  /* function which should solve task 2 */

  ROS_INFO("The coursework solving callback for task 2 has been triggered");
  reset_task();

  add_plane(plane_name_);
  add_baskets(basket_name_, request.basket_locs);

  // OK so we first try to go over the whole thing, and see if things work out fine. We also display the data from /c200 to see what's going on
  int numb_targets = request.basket_locs.size();
  geometry_msgs::Pose target, target_obs_pose;
  target.orientation.x = 0.9239;
  target.orientation.y = -0.3827;
  target.orientation.z = 0.0;
  target.orientation.w = 0.0;

  // initialize the output
  std::vector<std::string> output;
  output.resize(numb_targets);

  bool mvarm_success;
  // loop the location of the baskets

  for (int i = 0; i < numb_targets; i++) {
    target.position = request.basket_locs[i].point;
    ROS_INFO("Position (x, y, z): (%f, %f, %f)",
             target.position.x,
             target.position.y,
             target.position.z);
    target_obs_pose = target;
    target_obs_pose.position = target.position;
    target_obs_pose.position.z = 0.5;
    ROS_INFO("Moving arm");
    mvarm_success = move_arm(target_obs_pose);

    // Here we insert a list of 
    std::vector<DetectedObject> detected_objects;
    ROS_INFO("Detecting Object");

    // The whole object detction section
    wait_for_new_point_cloud();
    convert_ptcld_to_world(cloud_filtered_, cloud_filtered2_);
    detect_objects(cloud_filtered2_, detected_objects);

    // Find closest color
    double min_dist_sq = std::numeric_limits<double>::max();
    std::string closest_color = "";
    
    std::cout << "At location " << i << std::endl;
    // Loop over all detected objects
    for (const auto &obj : detected_objects) {
        std::cout << "Object " << std::endl;
        std::cout << "w:" << obj.w << " l:" << obj.l << "h:" << obj.h << std::endl;
        std::cout << "r:" << obj.r << " g:" << obj.g << "b:" << obj.b << std::endl;

        // Skip if color == "other"
        if (obj.color == "other") {
            continue;
        }

        // Compute squared distance to target.position
        double dx = target.position.x - obj.position.x;
        double dy = target.position.y - obj.position.y;
        double dist_sq = dx * dx + dy * dy;

        // Keep track of the closest so far
        // Also we only accept distance that is less than 0.05 apart from were we think the basket should be
        if (dist_sq < min_dist_sq && dist_sq < 0.05 * 0.05) {
            min_dist_sq = dist_sq;
            closest_color = obj.color;
        }
    }

    // After the loop, check if we found any valid object
    if (closest_color.empty()) {
        // No valid "non-other" objects were found
        output[i] = "none";
    } else {
        // We have the color of the closest object
        output[i] = closest_color;
    }
  }

  response.basket_colours = output;
  for (const auto &s : output) {
    std::cout << s << std::endl;
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool cw1::task3_callback(cw1_world_spawner::Task3Service::Request &request,
                        cw1_world_spawner::Task3Service::Response &response) {
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");
  reset_task();
  add_plane(plane_name_);
  add_clarance("Clearance");
  set_constraint();

  // Step 1: mapping the scene
  // We will take visualizations every 0.2 m
  geometry_msgs::Pose target;
  target.orientation.x = 0.9239;
  target.orientation.y = -0.3827;
  target.orientation.z = 0.0;
  target.orientation.w = 0.0;

  // Create a map from color to basket positions for quick lookup
  std::unordered_map<std::string, geometry_msgs::Point> basket_map;

  // Is basket map populated
  bool basket_map_populated = false;
  // Are cubes possibly in scene?
  bool cube_possible = true;

  // The Max and Min value of the plane
  double max_x = 0.7;
  double min_x = 0.2;
  double max_y = 0.45;
  double min_y = -0.45;

  // The x and y interval the robot need to check to form complete map
  double y_inc = 0.3;
  double x_inc = 0.25;

  // The amount of distance we think the cobe might move if the pick action fails
  double tol = 0.05;

  // We keep trying the task while there still may be cubes in scene
  while(cube_possible) {
    cloud_world_.reset(new PointC);
    // Scanning
    for(double y = std::min(max_y, min_y+y_inc/2); y <= std::max(min_y, max_y-y_inc/2)+0.001; y+=y_inc) {
      for(double x = std::min(max_x, min_x+x_inc/2); x <= std::max(min_x, max_x-x_inc/2)+0.001; x+=x_inc) {
        target.position.x = x;
        target.position.y = y;
        target.position.z = 0.6;
        ROS_INFO("Scanning at position: (%f, %f, %f)",
                target.position.x,
                target.position.y,
                target.position.z);
        move_arm(target);
    
        // Scanning
        wait_for_new_point_cloud();
        convert_ptcld_to_world(cloud_filtered_, cloud_filtered2_);
        // Scan again for another point cloud
        wait_for_new_point_cloud();
        convert_ptcld_to_world(cloud_filtered_, cloud_filtered2_);
      }
    }
    // Scaning phase complete, removing clearance
    remove_collision_object("Clearance");

    // Step 2: detect all the objects in the scene
    std::vector<DetectedObject> detected_objects;
    ROS_INFO("Detecting Object");
    detect_objects(cloud_world_, detected_objects);

    // Step 3: find out which baskets we have, and pace cubes with the right color into the right basket
    // Populate basket map when doing the first complete scene scan
    if(!basket_map_populated) {
      int i=0;
      for (auto& obj : detected_objects) {
        ROS_INFO("Object at position: (%s, %s, %f, %f, %f)", obj.type.c_str(), obj.color.c_str(),
                obj.position.x,
                obj.position.y,
                obj.position.z);
        if (obj.type == "cube" && obj.color != "other") {
          obj.name = "cube" + std::to_string(i);
          add_cube(obj.name, obj.position);
        }
        if (obj.type == "basket" && obj.color != "other") {
          obj.name = "basket" + std::to_string(i);
          add_basket(obj.name, obj.position);
          basket_map[obj.color] = obj.position;
        }
        i++;
      }
      basket_map_populated = true;
    }

    // place cubes into matching color baskets
    // We also record the max and min xy values of the cubes picked
    // This gives a range of where cubes might be after potentially failed executions

    // Set Max min x y to unreasonable values so it will always both be overwritten if cube exists
    // Set cube possible to false
    max_x = -10;
    max_y = -10;
    min_x = 10;
    min_y = 10;
    cube_possible = false;
    
    for (auto& cube : detected_objects) {
      if (cube.type == "cube" && cube.color != "other") {
        auto basket_it = basket_map.find(cube.color);
        if (basket_it != basket_map.end()) {
          // Also if we didn't pick up something from within a basket
          if(std::pow(cube.position.x - basket_it->second.x, 2) 
             + std::pow(cube.position.y - basket_it->second.y, 2)
             + std::pow(cube.position.z - basket_it->second.z, 2) > std::pow(0.15, 2)) {
            max_x = std::max(cube.position.x+tol, max_x);
            min_x = std::min(cube.position.x-tol, min_x);
            max_y = std::max(cube.position.y+tol, max_y);
            min_y = std::min(cube.position.y-tol, min_y);
            pick_place_cube(cube.name, cube.position, basket_it->second);

            // If we tried to pick a cube, we might fail.
            cube_possible = true;
          }
        }
      }
    }
    
    if(cube_possible) {
      ROS_INFO(" ---------------------------------------------- ");
      ROS_INFO("Double ckecking for unsuccessful cube picks");
      ROS_INFO("Ckecking between range: x = [%f, %f], y = [%f, %f]", min_x, max_x, min_y, max_y);
      ROS_INFO(" ---------------------------------------------- ");
    }
    else {
      ROS_INFO(" ---------------------------------------------- ");
      ROS_INFO("Task Complete, no cubes left in the scene");
      ROS_INFO(" ---------------------------------------------- ");
    }
  }

  // Now we scan the area between max and min x and y if we picked a cube before to make sure nothing went wrong
  
  return true;
}
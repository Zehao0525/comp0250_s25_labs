
# cw1_team_5 README

This ROS package provides a solution to the coursework tasks involving object detection, manipulation, and scene mapping using MoveIt!, PCL, and custom collision object management. The package can be launched with a preconfigured launch file, and individual tasks are activated via ROS services.

---

## License

This project is licensed under the MIT License.  
See the [LICENSE.txt](LICENSE) file for details.

---

## Authors

- **Zehao Wang**
- **Jiaqi Yao** 

---

## Prerequisites

- ROS (tested on [ROS Noetic](http://wiki.ros.org/noetic))
- MoveIt!
- PCL (Point Cloud Library)
- A working installation of `catkin_tools` or `catkin_make`

---

## Building the Package

1. **Clone the repository** Clone the base comp0250 repository:
   ```bash
   cd your_desired_directory
   git clone https://github.com/surgical-vision/comp0250_s25_labs.git --recurse-submodules
   ```
2. **Copy team coursework folder:** copy in folder into the src file
   ```bash
   cd comp0250_s25_labs/src
   ```
   **paste in cw1_team_5, then run:** 
   ```bash
   cd ..
   ```

3. **Build the workspace:**
   ```bash
   catkin build
   ```

4. **Source the workspace:**
   ```bash
   source devel/setup.bash
   ```

---

## Running the Package

### Launching the Full Solution

To bring up the entire solution along with the simulated environment and planning scene, run:
```bash
roslaunch cw1_team_5 run_solution.launch
```

### Activating Individual Tasks

Each task can be started via a ROS service call.
- **Task 1 (Pick and Place Cube):**
  ```bash
  rosservice call /task 1
  ```
- **Task 2 (Object Detection and Colour Recognition):**
  ```bash
  rosservice call /task 2
  ```
- **Task 3 (Scene Mapping and Object Placement):**
  ```bash
  rosservice call /task 3
  ```

---

## Algorithms description
### Task 1
A simple algorithm that:
1. moves the arm to 30 cm above the destination, 
2. opens gripper
3. move to the cube location, 
4. closes gripper, 
5. up agian, 
6. move to 30 cm above the basket
7. opens gripper to drop the cube. 

We made modifications to the moveArm function from Lab 2 so it will not stop until it found a successful trajectory, and successfully executing it (execution only happens after successful planning)

### Task 2
The actuator will be moved to directly above each possible baskets, then a pointcloud will be scanned and processed as follows:
1. Transform: Point cloud will be converted to world coordinate
2. Filter: Points with z<0.015 will be ignored (to remove the plane)
3. (This pointcloud will also be added to a global pointcloud)
4. Cluster: Point cloud will be clustered by their Eucledian distance and color, using a Kd tree aided clustering algorithm
5. Detection: Based on the rgb values and size of the point cloud clusters the objects will be classified as "red", "blue", "purple" and "other", as well as "cube", "basket" and "other". This object will then be turned into an "DetectedObject" datastructure with its coordinates, color and type. 
6. Record Color: We take the color of the object with type "basket", closest to the potential "basket" coordinate, and within certian position error bound, as the color of our target. If no such target exists we mark this spot as None.

### Task 3
We do the following:
1. We first designate an **area** where objects can appear, it is initialized to the dimensions of the whole plane
2. Using the functions from task 2, we move the actuator over this area to collect point clouds (which is automatically added to the global point cloud). 
3. We then perform one pass of the object detection to find all the cubes and baskets.
4. We build the mapping of basket location and color if not already
5. We pick each cube to the basket with the right color (if basket exists)
- *Since moveit has a chance on failing trajectory execution (even if the planning was successful), our tries to pick up the cbe can fail.*
6. Therefore If we interected with a cube, we generate a new **area** which cubes might be in, and we go step 2 (5 will be skipped), to make sure that we have picked up all the cubes.
7. If we have not interacted with a cube, the scene must be clean, and we complete our task.

#### Note on particular design choices and possible tweeks
Our particular implementation takes a relativly safe strategy, we spent considerable time scanning the scene. We can easily increase task execution speed at the cost of some robustness, and ease of task modification (i.e. if we change the dimensions of the objects/baskets of interst). 
- We have decided that for step 2., we will scan positions with intervals 0.3m in y direction, 0.25m in x direction (one pointcloud taken every such interval). This is relativly slow, however it guarentees the quality of the pointcloud taken. We can possibly improve the speed of our solution if we increase the height of the actuator when taking the pointcloud and take less pointclouds in general. 
- Pointclouds are taken in stationary positions because the camera pointcloud and the TF pointcloud are Asynchronous (or rather we have no synchronisation guarentees).  If we had synchronized TF and point cloud data, we could scan the scene continuously and merge point clouds dynamically. Alternatively, we could assume synchronization, but this would introduce sensor noise, potentially compromising data accuracy.

## Potential Errors:
- For Task 1, since there is a cance that moveit can fail trajectory execution, it has a tiny chance of knocking the cube out of its inital location before we try close the gripper on it, calusing the task to fail (though we have not encountered this in our tests)
- For Task 2 and 3, there is a small chance that the objects may be misclassified due to noise. We have not encountered any such instances during our tests.
- For Task 3, Although we have a mechanism to revisit areas where the cubes where positioned after each pick-and-place attempt, we are basing this action on the assumption that the failed trajectory will only slightly change the cube's location. If the failed trajectory execution where to move the cube too much (more than 15-20 cm in a direction where there are no other cubes), our revisit may not register this cube and terminate execution. *[We have not encountered this in our tests, in fact it was hard to even recreate the "failed trajectory execution" event. We has not been able to throughly test this particular failure recovery mechanism]*

## Task Breakdown and Team Contributions

### Task 1: Pick and Place Cube
- **Description:** Implementing the pick and place pipeline for a cube object, including setting up the collision objects and planning the trajectory.
- **Estimated Total Time:** ~4 hours
- **Team Contributions:**  
  - Jiaqi Yao: 100% (Implemented the cube grabing algorithm)

### Task 2: Object Detection and Colour Recognition
- **Description:** Clustering point cloud data to detect objects and identify their colours, with logic to determine the closest valid object.
- **Estimated Total Time:** ~8 hours
- **Team Contributions:**  
  - Zehao Wang: ~80% (Implemented pcl scene scanning and world coordinate transfrom and object detection)
  - Jiaqi Yao: ~20% (Helped debug the code and optimized motion)

### Task 3: Scene Mapping and Automated Object Placement
- **Description:** Scanning the scene from multiple viewpoints, building a world model, detecting objects, and automating cube placement into corresponding baskets.
- **Estimated Total Time:** ~8 hours
- **Team Contributions:**  
  - Zehao Wang: ~50% (Combined methods in Task 1 and 2 to complete task 3)
  - Jiaqi Yao: ~50% (refined algorithm and tackeled failure cases)

*Note:* The estimated times above are approximate and based on collaborative work where contributions were nearly equal among all team members.

---

## Additional Information

- **Debugging & Logging:** The package uses ROS logging (e.g., `ROS_INFO`, `ROS_WARN`, and `ROS_ERROR`) for real-time debugging and feedback.
---

This README is meant to serve as a guide to build, run, and understand the contributions behind the coursework package. For further details or troubleshooting, please refer to the inline comments within the source code or contact one of the team members.


---

## Code structure

```
————folder
  |
  |—— include
  |    |—— cw1_class.h      # Header file containing class definitions and function declarations
  |    |—— cw1_class.h.bak  # Header file backup
  |
  |—— src
  |    |—— cw1_node.cpp               # Main node entry point
  |    |—— cw1_services.cpp           # Task service callback implementations
  |    |—— cw1_tools.cpp              # Utility tools and helper functions
  |    |—— add_or_remove_functions.cpp # Collision object management functions
  |    |—— cw1_class.cpp.bak            # backup file


```

### Key Function Modules

#### Core Service Callbacks (cw1_services.cpp)
- `task1_callback` - Implements Task 1: Pick and place a single cube
- `task2_callback` - Implements Task 2: Basket color detection and recognition
- `task3_callback` - Implements Task 3: Scene mapping and multi-object automated sorting

#### Collision Object Management (add_or_remove_functions.cpp)
- `add_plane` - Add ground plane
- `add_clarance` - Add spatial clearance area
- `add_basket/add_baskets` - Add single/multiple baskets to planning scene
- `add_cube/add_cubes` - Add single/multiple cubes to planning scene
- `add_collision_object` - Generic collision object addition function
- `remove_collision_object` - Remove specific object from scene
- `remove_all_collisions` - Clear all collision objects

#### Motion Planning and Control (cw1_tools.cpp)
- `move_arm` - Plan and move arm to target pose
- `move_gripper` - Control gripper opening and closing
- `set_constraint` - Set joint constraints for the robotic arm
- `pick_place_cube` - Advanced pick and place functionality, integrating multiple steps

#### Point Cloud Processing and Analysis (cw1_tools.cpp)
- `apply_voxel_filter` - Apply voxel grid filtering to reduce point cloud density
- `cloud_callback` - Process subscribed camera point cloud data
- `convert_ptcld_to_world` - Transform point cloud to world coordinate system
- `detect_objects` - Point cloud clustering and object recognition (color, type determination)
- `wait_for_new_point_cloud` - Synchronous waiting for new point cloud data
- `enforce_color_similarity` - Color similarity judgment function for clustering

#### Other Helper Functions
- `reset_task` - Reset task state and related variables

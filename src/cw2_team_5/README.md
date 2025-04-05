
# cw2_team_5 README

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
   **paste in cw2_team_5, then run:** 
   ```bash
   cd ..
   ```

3. **Build the workspace:**
   ```bash
   [source ros noetic]
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
roslaunch cw2_team_5 run_solution.launch
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
### Shape Matcing
We first explain the algorithm used to identify the shape of the objects of interest:  
**Input:** A pointcloud that is Voxelized and have filter Z >= 0.04.
- First we flatten the pointcloud
- Then we generate a reference pointcloud of the flattened respective shape that is voxelized (so a point every 0.002 m appart in a ordered formation)
- Then for each shape (Cross with cell size 0.02,0.03,0.04, Nought with cell size 0.02,0.03,0.04), we do the following:
    - Then we perform principle component analysis to find the principle axis. If everything is mathematically correct, this axis can be: 
        - For nought: The diagnal of the square shape (45 degrees rotation with respect to principle component)
        - For cross: The diagnal of one of the rectangles that forms the cross (+- arctan(1/5) $\approx$ 11.3 degrees rotation)
    - We rotate our reference pointcloud by that degree, and produce a match score against the pointcloud for each position, and record the maximum
        - If any point in the cloud B is less than 0.03m(Voxel grid granularity) from point i in cloudA, point i is a match
        - The score is (number of matches for both pointcloud) / (size of both pointclouds)
    - If the matching score is too high (perfect match) or too low (errornouse detection), we stop and return. Otherwise we perform matches every 0.5 degrees within 11 degrees of: the principle component of the cross, the principle component + 45 degrees for nought. This should account for any nise present
    - We return the degree and matching score of the highest scored result.
- We return the shape, and gle, and score with the overall highest score. 


### Task 1
A simple algorithm that:
1. moves the arm to 41 cm above the destination, 
2. scan a pointcloud of the object, ad apply Vxel and Z value pass through filter
3. Perform shape matching
4. Generate collision object matching the detected shape
5. Grasp object and transprot to basket

We made modifications to the moveArm function from Lab 2 so it will not stop until it found a successful trajectory, and successfully executing it (execution only happens after successful planning)

### Task 2
We scan each given shape and then the mystry object:
1. Move the arm above the objetc 1, and perfom shape matching
2. Move the arm above the object 2, and perfom shape matching 
2. Move the arm above the Mystry object, and perfom shape matching. Determine which shaoe this matches and return result

### Task 3
We do the following:
1. We first designate an **area** where objects can appear, it is initialized to the dimensions of the whole plane
2. We move the actuator accorss the hole plane to get a scan of the platform.
3. We apply Voxel and Z value pass through filters.
3. We use a clustering algorithm based on euclidian distance and color to find close together points with the same color
4. We proces each cluster: First determining if it is an obstacle or basket by their color, then use shape matching to match object shape. In the process we will keep track of the number of crosses and nougts encountered, as well as the cross and nought with the highest matching score.
5. We pick and place the majority shape item with the highest matching score using the same method from task 1.

#### Note on particular design choices and possible tweeks
Our particular implementation takes a relativly safe strategy, we spent considerable time scanning the scene.
- We have decided that for step 2., we will scan positions with intervals 0.3m in y direction, 0.25m in x direction (one pointcloud taken every such interval). This is relativly slow, however it guarentees the quality of the pointcloud taken. Speed increase at the cost of noise is possible.  
- Pointclouds are taken in stationary positions because the camera pointcloud and the TF pointcloud are Asynchronous (or rather we have no synchronisation guarentees).  If we had synchronized TF and point cloud data, we could scan the scene continuously and merge point clouds dynamically. Alternatively, we could assume synchronization, but this would introduce sensor noise, potentially compromising data accuracy. 

## Potential Errors:
- For Task 1, The collision of the picked object is deleted from the scene. Additionally the RRT planner often plan some really crazy paths. This means there is a chance for the arm to plan a path that will cause the picked object to collide with other objects and knocking the gripped object out of the gripper. 
- For Task 2 and 3, there is a small chance that the objects may be misclassified due to noise. This is highly unlikly and we have not encountered any such instances during our tests.
- For Task 3, the same error for both task 1 and 2 can occure. Additionally, although the threshold for a shape to be considered a cross or nought is generouse, there is still a chance that some shape are ignored due to noise in the pointcloud scanning process. 

## Task Breakdown and Team Contributions

### Task 1: Pick and Place Cube
- **Description:** Implementing the pick and place pipeline for a cube object, including setting up the collision objects and planning the trajectory.
- **Estimated Total Time:** ~6 hours
- **Team Contributions:**  
  - Jiaqi Yao: 70% (Implemented the algorithm for T1_ANY_ORIENTATION = False, and partial solution for T1_ANY_ORIENTATION = True)
  - Zehao Wang: 30% (Optimized the algorithm for T1_ANY_ORIENTATION = True to achieve higher accuracy)

### Task 2: Object Detection and Colour Recognition
- **Description:** Clustering point cloud data to detect objects and identify their colours, with logic to determine the closest valid object.
- **Estimated Total Time:** ~8 hours
- **Team Contributions:**  
  - Jiaqi Yao: ~80% (Implemented the algorithm for T2_ANY_ORIENTATION = False, and partial solution for T2_ANY_ORIENTATION = True)
  - Zehao Wang: ~20% (Optimized the algorithm for T2_ANY_ORIENTATION = True to significantly reduce computation time)

### Task 3: Scene Mapping and Automated Object Placement
- **Description:** Scanning the scene from multiple viewpoints, building a world model, detecting objects, and automating cube placement into corresponding baskets.
- **Estimated Total Time:** ~16 hours
- **Team Contributions:**  
  - Jiaqi Yao: ~40% (Implemented the algorithm for T3_ANY_ORIENTATION = False)
  - Zehao Wang: ~60% (Implemented the algorithm for T3_ANY_ORIENTATION = True and optimized the algorithm's workflow)

### Code styleization and README:
- **Dsicription**: Retroactivly commenting and renaming functions and variables according to conventions, as well as writing the readme
- **Estimated Total Time:** ~ 1.5 hours
- **team Contributions:**
  - Zehao Wang: ~100%

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
  |    |—— cw2_class.h      # Header file containing class definitions and function declarations
  |    |—— data_structure.h
  |    |—— detect_object.h
  |    |—— external.h
  |    |—— shape_generator.h
  |
  |—— src
  |    |—— cw2_node.cpp                     # Main node entry point
  |    |—— cw2_class.cpp                    # Main task files
  |    |—— detect_object.cpp                # Utility tools for object detection and matching
  |    |—— gen_rviz_obj.cpp                 # RViz Collision object management functions
  |    |—— move_and_pick.cpp                # Actuator movement functions
  |    |—— PCL_test.cpp                     # testing using the saved PCD files
  |    |—— pcl_tools.cpp                    # Utility tools for pointclod processing
  |    |—— shape_generator_test.cpp         # RViz Collision object management functions
  |    |—— shape_generator.cpp              # Generating pointcloud for object matching


```

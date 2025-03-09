
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
   git clone https://github.com/surgical-vision/comp0250_s25_labs.git
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
- **Estimated Total Time:** ~4 hours
- **Team Contributions:**  
  - Zehao Wang: ~50% (Combined methods in Task 1 and 2 to complete task 3)
  - Jiaqi Yao: ~50% (refined algorithm and tackeled failure cases)

*Note:* The estimated times above are approximate and based on collaborative work where contributions were nearly equal among all team members.

---

## Additional Information

- **Debugging & Logging:** The package uses ROS logging (e.g., `ROS_INFO`, `ROS_WARN`, and `ROS_ERROR`) for real-time debugging and feedback.
---

This README is meant to serve as a guide to build, run, and understand the contributions behind the coursework package. For further details or troubleshooting, please refer to the inline comments within the source code or contact one of the team members.
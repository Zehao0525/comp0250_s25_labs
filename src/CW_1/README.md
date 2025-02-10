Authors: Zijie Chen (zijie-chen.20@ucl.ac.uk), Xuantong Lu (xuantong.lu.18@ucl.ac.uk), Wan Cai (wan.cai.17@ucl.ac.uk).

Video link: https://www.youtube.com/watch?v=6YOBiV6amBA&t=29s

Description: This package forms the base ROS workspace for the coursework of module COMP0129 (https://github.com/COMP0129-UCL/comp0129_s23_robot.git): Robotic Sensing, Manipulation and Interaction.

## Pre-Requisites
```bash
> sudo apt install ros-noetic-franka-ros ros-noetic-libfranka
```
Gazebo physics simluator is also needed (http://gazebosim.org/). This can be installed and then run with:
```bash
> curl -sSL http://get.gazebosim.org | sh
> gazebo
```

## Installation
1. Download this folder into your **your_catkin_workspace/src/** (replace **your_catkin_workspace** to the name of your folder).

2. Build this project through command:  

```bash
> cd your_catkin_workspace
> catkin build
```

## Run Panda robot Gazebo and rviz
```bash
> source devel/setup.bash
> roslaunch cw1_team_4 run_solution.launch
```

## Visualizing Task 1, 2 & 3
* This cousework contains three tasks including grasping, color detection and a planning task contains these two. For visualizing each task, run following command after run the Panda robot Gazebo and rviz:

```bash
> rosservice call /task x
```
**x** is the index of task, which is **1, 2 or 3**. Replace **x** with your choice.

* A Point Cloud viewer has been implemented and will pop up when do `rosservice call /task x`. It provide the robot's perspectives. If nothing can be seen from the window, try scroll using mouse.    

## Algorithms description

* Task 1: Pick and Place at given positions.

  Step 1: add collision objects to the Moveit motion planning.  

  Step 2: move to the approach pose just above the cube and open the gripper.

  Step 3: move to the grasp pose which is the centroid of cubes (given).

  Step 4: move to the release_pose which is at some distance above the basket (given).

  Step 5: open the grippers and drop cube on the basket. 

* Task 2: Basket Colour identification.

  Step 1: move to the first given position (given)

  Step 2: use the subscribed point cloud to classify the colour (Voxel Grid and Pass through filtered, plane segmented).

  Step 3: repeat Step  2 until all given position has been checked.

* Task 3: Planning and Execution. 

  Step 1: move to a scan pose to get the overview of all objects on the working desk.

  Step 2: using a Euclidean Cluster to find cubes and baskets. 

  Step 3: based on point cloud of each sub-clusters, return colour, category (basket/cube) and position.

  Step 4: move to one cube, similar as task 1 to place it on basket in corresponding colour.

## Potential Error

* This may pop up on command in some cases. To solve it, we have to re-`roslaunch` the project.

```bash
[cw1_solution_node-19] process has died [pid 90014, exit code -6, cmd /home/wancai/comp0129_s23_robot/devel/lib/cw1_team_4/cw1_solution_node __name:=cw1_solution_node __log:=/home/wancai/.ros/log/59bb9b7a-b913-11ed-963c-db5ae5b7568a/cw1_solution_node-19.log].
log file: /home/wancai/.ros/log/59bb9b7a-b913-11ed-963c-db5ae5b7568a/cw1_solution_node-19*.log
```

* Also, task 3 has some situations where noise is regarded as cube. Those noise may present a position that robot can not reach and thus lead to the failure of motion planning. In this case, we can only re-`rosservice call /task 3` to start a new task. However, after optimising the hyperparameter of PCL segmentation algorithms, this problems has been less likely to happen (approximately 1 out of 20 tests). 

  *<u>(Task 3 is challenging and we found that our method may be not robust enough to solve all situations. So please, test it serval times to see its general performance.)</u>*

* Euclidean Clustering is not stable as the hyperparameters have not been turned/optimised using any optimisation algorithm. Consequently, in some cases, the returned centroid position of cube may have error, which lead to a unreliable motion planning and crash happened. 

## Performance/Accuracy

From our experiments which run three tasks repeatedly (20 times), the accuracy of each algorithms:

* Task 1: 100%
* Task 2: 95%
* Task 3: 90%

## License

LICENSE:  
Copyright (c) 2019-2021 Zijie Chen

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


DISCLAIMER:

THIS INFORMATION AND/OR SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS INFORMATION AND/OR
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Copyright (C) 2019-2023 Zijie Chen except where specified

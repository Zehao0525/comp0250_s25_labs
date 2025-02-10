#!/usr/bin/python3

"""
This code contains the world spawner. This file accomplishes two main goals:

  1. Create and manage the objects in the gazebo world using the World() class
  2. Spawn and monitor the completion of coursework tasks using the Task() class

The coursework contains three tasks, and each of them are defined here. There
are three classes derived from the Task() base class, Task1(), Task2(), and
Task3().

Each of these TaskX() classes defines what objects will spawn for a given task,
as well as sending the service request for taskX_start() - this is the request
that you will receive and respond to (by solving the task).

Currently, tasks may spawn with random box positions and colours. If you wish to
change this, you can edit the task parameters:

  1. Task parameters defined as globals (IN UPPER CASE) at the top of this file.
     Edit these to adjust the tasks, this can help you during testing.

  2. Each TaskX() class has a method 'spawn_task_objects' which sets up the task,
     you can edit this function as well to change tasks during testing.

IMPORTANT! You will NOT submit your version of this file. When we mark your work,
we will use a clean version of this file.

During our tests, we may adjust the tasks to look at edge cases. The following
parameters may change:
  - T2_N_BASKETS
  - T2_BASKET_COLOUR_UNIQUE
  - T3_N_BOXES
  - T3_N_BASKETS
  - T3_BASKET_COLOUR_UNIQUE

No other parameters will change.

Best of luck!
"""

import rospy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from coursework_world_spawner_lib.coursework_world_spawner import *
from cw1_world_spawner.srv import TaskSetup, TaskSetupResponse
from cw1_world_spawner.srv import Task1Service, Task2Service, Task3Service
import numpy as np

# # hint: an easy way to 'disable' the randomness in the task spawning is:
# myseed = 0
# np.random.seed(myseed) # choose any int as your seed

# ----- key coursework task parameters ----- #

# task 1 parameters                 
T1_BOX_X_LIMS = [0.3, 0.5]            # xrange a box can spawn
T1_BOX_Y_LIMS = [-0.10, 0.11]         # yrange a box can spawn

# task 2 parameters
T2_N_BASKETS = 3                      # number of baskets to spawn
T2_BASKET_COLOUR_UNIQUE = False       # can there only be one basket of each colour
T2_BASKET_NOISE = 50e-3               # possible noise on (x, y) for basket location

# task 3 parameters
T3_N_BOXES = 4                        # number of boxes spawn
T3_N_BASKETS = 3                      # number of baskets to spawn
T3_BASKET_COLOUR_UNIQUE = True        # can there only be one basket of each colour
T3_BOX_X_LIMS = [0.35, 0.66]          # xrange a box can spawn
T3_BOX_Y_LIMS = [-0.10, 0.11]         # yrange a box can spawn
T3_BASKET_NOISE = 50e-3               # possible noise on (x, y) for basket location

# possible goal basket locations (x, y)
BASKET_LOCATIONS = [(0.59, -0.34), 
                    (0.59,  0.34),
                    (0.31, -0.34),
                    (0.31,  0.34)]

# possible spawned box colours, do not edit as rgb for models set in sdf file
BOX_COLORS = {'purple': [0.8, 0.1, 0.8],
              'red':    [0.8, 0.1, 0.1], 
              'blue':   [0.1, 0.1, 0.8]}

# possible goal basket colours, do not edit as rgb for models set in sdf file
BASKET_COLORS = {'purple': [0.8, 0.1, 0.8],
                 'red':    [0.8, 0.1, 0.1], 
                 'blue':   [0.1, 0.1, 0.8]}

# ----- Class definitions for base world and task ----- #

class World(object):

  # lengths in metres for world objects, resultant from sdf model files, don't change
  tile_side_length = 100e-3
  tile_thickness = 20e-3
  basket_side_length = 120e-3
  robot_safety_radius = 280e-3

  def __init__(self):
    """
    World managing class for the coursework
    """
    world_spawner.despawn_all(exceptions="object-golf-tile")
    self.spawn_tiles()

  def spawn_tiles (self):
    """
    Spawn all of the green tiles around the panda robot
    """

    # spawn in all of the tiles, xyz already input in model.sdf so use (0, 0, 0)
    model = Model(model_name = "all_tiles",
                  instance_name = 'object-all-golf-tiles',
                  model_type = 'sdf',
                  position = [0, 0, 0]
            )
    world_spawner.spawn(model)

    return

  def spawn_goal_basket(self, goal_loc=None, colour=None, name=None):
    """
    Spawn a new goal basket, returns where it spawned as a geometry_msgs/Point.
    If colour is set to 'empty', returns only the spawn point without spawning
    """

    if goal_loc is None: raise RuntimeError("no goal location given to spawn_goal_basket()")
    if colour is None: colour = np.random.choice(list(BASKET_COLORS.keys()))
    if name is None: name = "object-goal-basket"

    # determine the point to spawn in the basket
    goal_pt = Point(goal_loc[0], goal_loc[1], world.tile_thickness)

    # create and spawn the model, unless told colour is "empty"
    if colour != "empty":
      model = Model(model_name = f"basket_{colour}",
                    instance_name = name,
                    model_type = 'sdf',
                    position = [goal_pt.x, goal_pt.y, goal_pt.z]
              )
      world_spawner.spawn(model)

    return goal_pt

  def reset(self):
    world_spawner.despawn_all()
    return

class Task(object):

  def __init__(self, mode='coursework', validation_scenario=0):
    """
    Task base class with methods to spawn objects and reset
    """

    self.models = {}

    if mode == 'coursework':
      self.spawn_task_objects()
      self.begin_task()
    else:
      self.spawn_test_objects(validation_scenario)
      self.begin_test(validation_scenario)

    return 

  def spawn_box_object(self, name='boxobject1', 
                       xlims = [0.3,0.5], ylims = [-0.12,0.12], zlims = [0.1,0.101],
                       color='blue'):
    """
    Spawn a box of a given colour in a random position given some limits
    """
    # determine the colour of the box to spawn
    if color=='blue': model_name = 'box_blue'
    elif color=='red': model_name = 'box_red'
    elif color=='purple': model_name = 'box_purple'
    else: model_name = 'box'
    rospy.logdebug("Spawning " + model_name)

    # create and spawn the model
    model = Model(model_name = model_name,
                  instance_name = name,
                  model_type = 'sdf',
                  position = random_position_in_area(xlims, ylims, zlims))
    world_spawner.spawn(model)

    # save the spawned model in the dictionary
    self.models[name] = model

    return

  def spawn_random_goal_baskets(self, num=1, save_empty=False, 
                                unique_colours=True, noise=0.0):
    """
    Spawn goals with randomised colours
    """

    if num < 1: raise RuntimeError("spawn_random_goal_baskets(...) given num < 1")
    if num > len(BASKET_LOCATIONS): 
      raise RuntimeError("spawn_random_goal_baskets(...) given num greater than len(BASKET_LOCATIONS)")

    # wipe and reset basket information
    self.basket_points = []
    self.basket_colours = []

    # what are the possible colours to choose from
    if unique_colours:
      goal_colours = list(BASKET_COLORS.keys())
    else:
      goal_colours = list(BASKET_COLORS.keys()) * 10
    
    # how many baskets have we been asked for
    if num < len(goal_colours):
      # shuffle the colours and trim to the correct size
      goal_colours = list(np.random.permutation(goal_colours)[:num])
    elif num > len(goal_colours):
      raise RuntimeError("spawn_random_goal_baskets(...) has num > len(goal_colours) but unique_colours=True")
    
    # do we save the position of empty basket locations
    if save_empty:
      num_empties = len(BASKET_LOCATIONS) - len(goal_colours)
      if num_empties > 0: 
        goal_colours += ["empty"] * num_empties

    # randomly shuffle the colour and goal location orders
    goal_colours = np.random.permutation(goal_colours)
    goal_locs = np.random.permutation(BASKET_LOCATIONS)
    print("goal_basket spawned at:",goal_locs)
    for i, colour in enumerate(goal_colours):
      
      basket_loc = goal_locs[i] + noise * (np.random.random() * 2 - 1)
      basket_point = world.spawn_goal_basket(goal_loc=basket_loc, colour=colour,
                                             name=f"object-goal-basket-{i}")
      self.basket_points.append(basket_point)
      self.basket_colours.append(colour)

  def prepare_for_task_request(self, service_name, timeout=60):
    """
    Search for the required task solving service to be advertised
    """
    rospy.logdebug(f"Attempting to connect to {service_name} Service...")
    try:
      rospy.wait_for_service(service_name, timeout=timeout)
    except rospy.ROSException as e:
      rospy.logwarn(f"Error in {service_name} request:", e)
      rospy.logdebug(f"{service_name} Request failed - not advertised")
      return False
    return True

  def reset_task(self):
    """
    Remove any objects from the world, so baskets and cubes
    """
    world_spawner.despawn_all(keyword='object', exceptions="tile")
    return

  def get_position_from_point(self,pt):
    return np.asarray([pt.x, pt.y, pt.z])

  def get_position_from_point_stamped(self,ptst):
    pt = ptst.point
    return np.asarray([pt.x, pt.y, pt.z])

  def get_position_from_pose(self,pose):
    pos_np = self.get_position_from_point(pose.position)
    return pos_np

  def get_euclidean_distance(self,a,b):
    return np.sqrt(np.sum(np.power(a - b,2)))

  def spawn_task_objects():
    raise NotImplementedError
  
  def begin_task(self):
    raise NotImplementedError
  
  def spawn_test_objects(self, validation_scenario):
    raise NotImplementedError
  
  def begin_test(self, validation_scenario):
    raise NotImplementedError
  
# ----- Class definitions for the coursework tasks ----- #

class Task1(Task):

  # name of service which is requested to solve the task
  service_to_request = "/task1_start"

  def __init__(self, mode='coursework', validation_scenario=0):
    """
    Task class, initialise either in 'coursework' mode (normal, no validation
    scenario needed), or initialise in 'validation' mode and pass in a chosen
    validation scenario number.
    """
    rospy.loginfo('================Starting Task1==============')
    Task.__init__(self, mode, validation_scenario)

  def spawn_task_objects(self):
    """
    Spawns the objects for the task, feel free to edit
    """
    
    print()
    print("========================")
    print("Spawning Object")
    self.reset_task() # remove any objects currently spawned

    # spawn new objects for this task
    self.spawn_random_goal_baskets(num=1)
    self.spawn_box_object(name='boxobject1', xlims=T1_BOX_X_LIMS, ylims=T1_BOX_Y_LIMS)
    print("========================")
    print()

    return

  def send_task1_request(self, pose):
    """
    Sends out a service request that this task get solved
    """

    rospy.logdebug("Task1 Service connected. Sending request...")
    task1srv = rospy.ServiceProxy(self.service_to_request, Task1Service)
    pose_st = PoseStamped()
    pose_st.pose = pose
    pose_st.header.frame_id = "panda_link0"
    pose_st.header.stamp = rospy.Time.now()
    point_st = PointStamped()
    point_st.point = self.basket_points[0]
    point_st.header.frame_id = "panda_link0"
    point_st.header.stamp = rospy.Time.now()
    _ = task1srv(pose_st, point_st)

    return True

  def begin_task(self):
    """
    Start the task, check a service is available to solve the task
    """

    success = self.prepare_for_task_request(self.service_to_request)
    rospy.sleep(rospy.Duration(1))
    init_pose = self.models['boxobject1'].get_model_state().pose
    if success: resp = self.send_task1_request(init_pose)
    else: rospy.logerr("Task Request failed - not advertised")

    return

class Task2(Task):

  # name of service which is requested to solve the task
  service_to_request = "/task2_start"

  def __init__(self, mode='coursework', validation_scenario=0):
    """
    Task class, initialise either in 'coursework' mode (normal, no validation
    scenario needed), or initialise in 'validation' mode and pass in a chosen
    validation scenario number.
    """
    rospy.loginfo('================Starting Task2==============')
    Task.__init__(self,mode, validation_scenario)

  def spawn_task_objects(self):
    """
    Spawns the objects for the task, feel free to edit
    """

    self.reset_task() # remove any objects currently spawned

    # spawn new objects for this task
    self.spawn_random_goal_baskets(num=T2_N_BASKETS, unique_colours=T2_BASKET_COLOUR_UNIQUE,
                                   noise=T2_BASKET_NOISE, save_empty=True)
                                  
    return

  def send_task2_request(self):
    """
    Sends out a service request that this task get solved
    """

    rospy.logdebug("Task2 Service connected. Sending request...")
    task2srv = rospy.ServiceProxy(self.service_to_request, Task2Service)

    # get the possible basket locations as points
    basket_locs = []
    for i in range(len(BASKET_LOCATIONS)):
      point_st = PointStamped()
      point_st.point = self.basket_points[i]
      point_st.header.frame_id = "panda_link0"
      point_st.header.stamp = rospy.Time.now()
      basket_locs.append(point_st)

    resp = task2srv(basket_locs)

    return resp

  def begin_task(self):
    """
    Start the task, check a service is available to solve the task and then call
    that service
    """

    success = self.prepare_for_task_request(self.service_to_request)
    if success: resp = self.send_task2_request()
    else: rospy.logerr("Task Request failed - not advertised")

    return

class Task3(Task):

  # name of service which is requested to solve the task
  service_to_request = "/task3_start"

  def __init__(self, mode='coursework'):
    """
    Task class, initialise either in 'coursework' mode (normal, no validation
    scenario needed), or initialise in 'validation' mode and pass in a chosen
    validation scenario number.
    """
    rospy.loginfo('================Starting Task3==============')
    Task.__init__(self, mode)

  def spawn_task_objects(self):
    """
    Spawns the objects for the task, feel free to edit
    """

    self.reset_task() # remove any objects currently spawned

    # spawn new baskets for this task
    self.spawn_random_goal_baskets(num=T3_N_BASKETS, unique_colours=T3_BASKET_COLOUR_UNIQUE,
                                   noise=T3_BASKET_NOISE, save_empty=True)

    # create a list of all possible box spawning positions
    self.box_locs = []
    ys = np.arange(T3_BOX_Y_LIMS[0], T3_BOX_Y_LIMS[1], step=world.tile_side_length)
    xs = np.arange(T3_BOX_X_LIMS[0], T3_BOX_X_LIMS[1], step=world.tile_side_length)
    for i, x_i in enumerate(xs):
      for j, y_j in enumerate(ys):
        if (np.abs(x_i) < world.robot_safety_radius and 
            np.abs(y_j) < world.robot_safety_radius): 
          continue
        self.box_locs.append([x_i, y_j])
    
    # spawn new objects randomly within limits
    np.random.shuffle(self.box_locs)
    self.box_locs = self.box_locs[:T3_N_BOXES]
    for i,loc in enumerate(self.box_locs):
      box_colour = list(BOX_COLORS.keys())[np.random.randint(0, len(BOX_COLORS.keys()))]
      mname = 'boxobject%02d'%i
      self.models[mname] = self.spawn_box_object(name=mname, color=box_colour,
                             xlims=[loc[0],loc[0]], ylims=[loc[1],loc[1]])

    return

  def send_task3_request(self):
    """
    Sends out a service request that this task get solved
    """

    # Send request
    rospy.logdebug("Task3 Service connected. Sending request...")
    task3srv = rospy.ServiceProxy(self.service_to_request, Task3Service)
    resp = task3srv()

    return resp
  
  def begin_task(self):
    """
    Start the task, check a service is available to solve the task and then call
    that service
    """

    # Prepare and send request
    success = self.prepare_for_task_request(self.service_to_request)
    if success: resp = self.send_task3_request()
    else: rospy.logerr("Task Request failed - not advertised")

    return

# ----- Running the coursework node ----- #

def handle_task_request(req):
  """
  Helper function to handle requesting tasks
  """

  # Callback for selecting which task to start
  if req.task_index == 1:
    Task1(mode="coursework")

  elif req.task_index == 2:
    Task2(mode="coursework")

  elif req.task_index == 3:
    Task3(mode="coursework")

  else: rospy.logwarn("Unrecognized task requested")

  return TaskSetupResponse()

# globals for interacting with the world
world_spawner = WorldSpawner()
world = World()

if __name__ == "__main__":

  # create the world and run the coursework /task service
  rospy.init_node('coursework1_wrapper')

  # create the /task service callback
  rospy.Service('/task', TaskSetup, handle_task_request)
  rospy.loginfo("Ready to initiate task.")
  rospy.loginfo("Use rosservice call /task <INDEX> to start a task")
  rospy.spin()

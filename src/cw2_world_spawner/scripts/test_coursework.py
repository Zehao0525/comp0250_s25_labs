#!/usr/bin/python3

# THIS FILE MUST NEVER BE RELEASED TO STUDENTS

import rospy
import rospkg
from coursework_world_spawner_lib.coursework_world_spawner import *
from cw1_world_spawner.srv import TaskSetup, TaskSetupResponse
import numpy as np
import os
import datetime
import pdb
from geometry_msgs.msg import Point, PointStamped, PoseStamped

from world_spawner import Task1, Task2, Task3, World

# # hint: an easy way to 'disable' the randomness in the task spawning is:
# myseed = 0
# np.random.seed(myseed) # choose any int as your seed

# ----- key coursework task parameters ----- #

# task 1 parameters                 
T1_SHAPE_X_LIMS = [0.40, 0.55]           # xrange a shape can spawn
T1_SHAPE_Y_LIMS = [-0.40, 0.40]          # yrange a shape can spawn
T1_ANY_ORIENTATION = False               # do we allow any rotation of a shape
T1_GROUND_PLANE_NOISE = 0e-3             # do we add noise on the z height of the green tiles
T1_USE_MULTIPLE_SIZES = False            # do we spawn objects with varying sizes

# task 2 parameters
T2_SHAPE_X_LIMS = [0.40, 0.55]           # xrange a shape can spawn
T2_SHAPE_Y_LIMS = [-0.40, 0.40]          # yrange a shape can spawn
T2_N_REF_SHAPES = 2                      # number of baskets to spawn
T2_OBJECT_REF_POINTS = [(-0.43, -0.4), 
                        (-0.43,  0.4)]
T2_ANY_ORIENTATION = False               # do we allow any rotation of a shape
T2_GROUND_PLANE_NOISE = 0e-3             # do we add noise on the z height of the green tiles
T2_USE_MULTIPLE_SIZES = False            # do we spawn objects with varying sizes

# task 3 parameters
T3_MAX_SHAPES = 7                        # maximum number of spawned shapes
T3_SHAPE_X_LIMS = [-0.6, 0.7]            # xrange a shape can spawn
T3_SHAPE_Y_LIMS = [-0.55, 0.55]          # yrange a shape can spawn
T3_N_OBSTACLES = 2
T3_ANY_ORIENTATION = False               # do we allow any rotation of a shape
T3_GROUND_PLANE_NOISE = 0e-3             # do we add noise on the z height of the green tiles
T3_USE_MULTIPLE_SIZES = False            # do we spawn objects with varying sizes
T3_COMMON_SHAPES_DICT = {}
# possible goal basket locations (x, y)
BASKET_LOCATIONS = [(-0.41, -0.36), 
                    (-0.41,  0.36)]

# define the variety of spawned objects, these values are defined in .sdf files
POSSIBLE_SHAPES = ["nought", "cross"]
POSSIBLE_SIZES = ["40", "30", "20"]
POSSIBLE_COLOURS = {'purple': [0.8, 0.1, 0.8],
                    'red':    [0.8, 0.1, 0.1], 
                    'blue':   [0.1, 0.1, 0.8]}
POSSIBLE_OBSTACLES = [
  ["obstacle_1", 80e-3],
  ["obstacle_2", 50e-3],
  ["obstacle_3", 60e-3],
  ["obstacle_4", 100e-3],
]

# used to keep track of task models
TASK_MODELS_DICT =  {}

# ----- variables used for marking ----- #

# time in seconds in each task before timeout
T1_TEST_TIME = 60
T2_TEST_TIME = 120
T3_TEST_TIME = 360

# marking variables
T3_MIN_TASK_DURATION = 120
T2_TEST_NCENTROIDS_MAX_MARK = 50.
T2_TEST_ESTIMATION_MAX_MARK = 50.
T2_TEST_PICK_PLACE_MAX_MARK = 0.
T3_TEST_SPEED_MAX_MARK = 50
T3_TEST_PICK_PLACE_MAX_MARK = 50
TEST_REPORT_STR = """"""
# validation method, not for students
def task1_spawn_test_objects(self, validation_scenario):
  # get required globals
  global TEST_REPORT_STR, T1_GROUND_PLANE_NOISE, T1_ANY_ORIENTATION, T1_USE_MULTIPLE_SIZES, TASK_MODELS_DICT

  # set seed
  myseed = 5
  np.random.seed(myseed) # choose any int as your seed

  # reset task models
  TASK_MODELS_DICT =  {}


  # set ground height
  T1_GROUND_PLANE_NOISE = 1e-3 
  tile_height = np.random.random() * (T1_GROUND_PLANE_NOISE)
  self.reset_task(respawn_tiles=True, tile_height=tile_height)
    
  TEST_REPORT_STR += "\nTesting for task 1\n"

  # scenario 0: easy pick from right in front of the robot
  if validation_scenario == 0:
    T1_ANY_ORIENTATION = False
    T1_USE_MULTIPLE_SIZES = False

    rospy.loginfo("Creating validation scenario 0")
    TEST_REPORT_STR += "Creating validation scenario 0\n"

    # spawn a random object
    colour_options = list(POSSIBLE_COLOURS.keys())
    rand_colour = np.random.choice(colour_options)
    rand_object = POSSIBLE_SHAPES[0] # nought
    if T1_USE_MULTIPLE_SIZES:
      rand_size = np.random.choice(POSSIBLE_SIZES)
    else: rand_size = POSSIBLE_SIZES[0]
    random_object = rand_object + "_" + rand_colour + "_" + rand_size + "mm"
    rot_lims = [0, 2*np.math.pi] if T1_ANY_ORIENTATION else [0, 0]
    model = self.spawn_model(name=random_object, rotationlims=rot_lims, xlims=T1_SHAPE_X_LIMS,
                     ylims=T1_SHAPE_Y_LIMS)
    
    # add model instance to dict
    TASK_MODELS_DICT["object"] = model
    # save the name of the object
    self.object_type = rand_object

    
    # spawn a goal basket
    random_goal = BASKET_LOCATIONS[np.random.randint(0, len(BASKET_LOCATIONS))]
    model = self.spawn_model(name="basket", point=random_goal)
    # add model instance to dict
    TASK_MODELS_DICT["basket"] = model

  # scenario 1: easy pick from right in front of the robot, object rotated slightly
  elif validation_scenario == 1:
    rospy.loginfo("Creating validation scenario 1")
    TEST_REPORT_STR += "Creating validation scenario 1\n"

    T1_ANY_ORIENTATION = True
    T1_USE_MULTIPLE_SIZES = False

    # spawn a random object
    colour_options = list(POSSIBLE_COLOURS.keys())
    rand_colour = np.random.choice(colour_options)
    rand_object = POSSIBLE_SHAPES[1] # cross
    if T1_USE_MULTIPLE_SIZES:
      rand_size = np.random.choice(POSSIBLE_SIZES)
    else: rand_size = POSSIBLE_SIZES[0]
    random_object = rand_object + "_" + rand_colour + "_" + rand_size + "mm"
    rot_lims = [np.math.pi/6.0, np.math.pi/4.0] if T1_ANY_ORIENTATION else [0, 0]
    model = self.spawn_model(name=random_object, rotationlims=rot_lims, xlims=T1_SHAPE_X_LIMS,
                     ylims=T1_SHAPE_Y_LIMS)

    # add model instance to dict
    TASK_MODELS_DICT["object"] = model
    # save the name of the object
    self.object_type = rand_object

    
    # spawn a goal basket
    random_goal = BASKET_LOCATIONS[np.random.randint(0, len(BASKET_LOCATIONS))]
    model = self.spawn_model(name="basket", point=random_goal)
    # add model instance to dict
    TASK_MODELS_DICT["basket"] = model

                                                    
  else: raise RuntimeError("validation scenario must equal 0 or 1")

  return

# validation method, not for students
def task1_begin_test(self, validation_scenario):

  global TEST_REPORT_STR, TASK_MODELS_DICT

  t1_grade = 0
  ##### START TASK TIMER
  # rospy.logdebug("Starting Task1 timer.")
  # self.trial_timeout = False
  # rospy.Timer(rospy.Duration(T1_TEST_TIME), self.stop_callback, oneshot=True)
  ##### SEND TASK REQUEST

  rospy.logwarn(f"Evaluating Task 1 Validation Scenario {validation_scenario}\n")

  rospy.sleep(rospy.Duration(1))
  # init object location
  init_pose = TASK_MODELS_DICT['object'].get_model_state().pose
  init_pos_np = self.get_position_from_pose(init_pose)

  # basket location
  basket_pose = TASK_MODELS_DICT['basket'].get_model_state().pose
  goal_pos_np = self.get_position_from_pose(basket_pose)

  success = self.prepare_for_task_request(self.service_to_request)

  if success == False:
    rospy.loginfo("[RESULT - %d/100] Task1 - Fail, no service found!"%t1_grade)
    TEST_REPORT_STR += "[RESULT - %d/100] Task1 - Fail, no service found!\n"%t1_grade
    return False

  resp = self.send_task1_request(init_pose.position, basket_pose.position)

  # final object location
  final_pose = TASK_MODELS_DICT['object'].get_model_state().pose
  final_pos_np = self.get_position_from_pose(final_pose)
  dx,dy,dz = np.abs(goal_pos_np - final_pos_np)


  if dx < World.basket_side_length/2. and dy < World.basket_side_length/2.:
    t1_grade = 100
    rospy.loginfo("[RESULT - %d/100] Task1 - Successful pick and place!"%t1_grade)
    TEST_REPORT_STR += "[RESULT - %d/100] Task1 - Successful pick and place!\n"%t1_grade
    
    return True
  else: 
    d_move  = np.sqrt(np.sum(np.power(init_pos_np[:2] - final_pos_np[:2],2)))
    d_goal  = np.sqrt(np.sum(np.power(goal_pos_np[:2] - final_pos_np[:2],2)))
    d_total = np.sqrt(np.sum(np.power(goal_pos_np[:2] - init_pos_np[:2],2))) + 1e-6 # avoids division by zero

    if d_goal < d_total:
      t1_grade = int(70*(d_goal/d_total)) + 10 # 10 free marks for moving the shape + up to 70 more for getting close
    else:
      t1_grade = 0
    rospy.loginfo("[RESULT - %d/100] Task1 - Fail, timeout! Moved %0.3f, distance to goal %0.3f"%(t1_grade, d_move, d_goal))
    TEST_REPORT_STR += "[RESULT - %d/100] Task1 - Fail, timeout! Moved %0.3f, distance to goal %0.3f\n"%(t1_grade, d_move, d_goal)
    
    return False



# validation method, not for students
def task2_spawn_test_objects(self, validation_scenario):
  # get required globals
  global TEST_REPORT_STR, T2_GROUND_PLANE_NOISE, T2_ANY_ORIENTATION, T2_USE_MULTIPLE_SIZES, TASK_MODELS_DICT


  # reset task models
  TASK_MODELS_DICT =  {}


  TEST_REPORT_STR += "\nTesting for task 2\n"

  # scenario 0: easy mystery shape location: default setup
  if validation_scenario == 0:
    T2_ANY_ORIENTATION = False
    T2_USE_MULTIPLE_SIZES = False

    # set seed
    myseed = 5
    np.random.seed(myseed) # choose any int as your seed

    T2_GROUND_PLANE_NOISE = 0e-3
    tile_height = np.random.random() * (T2_GROUND_PLANE_NOISE)
    self.reset_task(respawn_tiles=True, tile_height=tile_height)

    rospy.loginfo("Creating validation scenario 0")
    TEST_REPORT_STR += "Creating validation scenario 0\n"

    colour_options = list(POSSIBLE_COLOURS.keys())
    shape_options = POSSIBLE_SHAPES
    n_ref_shapes = T2_N_REF_SHAPES
    rot_lims = [0, 2*np.math.pi] if T2_ANY_ORIENTATION else [0, 0]

    # randomise shapes
    rand_shapes = np.random.permutation(shape_options)

    # check we have enough possible shapes to use as references
    if n_ref_shapes > len(shape_options):
      rospy.logwarn("T2_N_REF_SHAPES is greater than number of possible shapes")
      n_ref_shapes = len(shape_options) # local change only
    if n_ref_shapes > len(T2_OBJECT_REF_POINTS):
      rospy.logwarn("T2_N_REF_SHAPES is greater than number of reference positions")
      n_ref_shapes = len(T2_OBJECT_REF_POINTS) # local change only

    # clip our random shapes by the number of references
    rand_shapes = rand_shapes[:n_ref_shapes]

    # loop and spawn our references shapes
    for i in range(n_ref_shapes):

      rand_colour = np.random.choice(colour_options)
      rand_object = rand_shapes[i]
      if T2_USE_MULTIPLE_SIZES:
        rand_size = np.random.choice(POSSIBLE_SIZES)
      else: rand_size = POSSIBLE_SIZES[0]
      random_ref = rand_object + "_" + rand_colour + "_" + rand_size + "mm"
      model = self.spawn_model(name=random_ref, point=T2_OBJECT_REF_POINTS[i], rotationlims=rot_lims)
      TASK_MODELS_DICT["ref-"+random_ref] = model

    # spawn the mystery object
    rand_colour = np.random.choice(colour_options)
    rand_object = np.random.choice(rand_shapes)
    if T2_USE_MULTIPLE_SIZES:
      rand_size = np.random.choice(POSSIBLE_SIZES)
    else: rand_size = POSSIBLE_SIZES[0]
    random_object = rand_object + "_" + rand_colour + "_" + rand_size + "mm"
    model = self.spawn_model(name=random_object, rotationlims=rot_lims, xlims=T2_SHAPE_X_LIMS,
                      ylims=T2_SHAPE_Y_LIMS)
    TASK_MODELS_DICT["obj-"+random_object] = model

  # mystery shape has rotation, ground has more noise
  elif validation_scenario == 1:
    T2_ANY_ORIENTATION = True
    T2_USE_MULTIPLE_SIZES = False

    # set seed
    myseed = 10
    np.random.seed(myseed) # choose any int as your seed

    T2_GROUND_PLANE_NOISE = 40e-3
    tile_height = np.random.random() * (T2_GROUND_PLANE_NOISE)
    self.reset_task(respawn_tiles=True, tile_height=tile_height)

    rospy.loginfo("Creating validation scenario 1")
    TEST_REPORT_STR += "Creating validation scenario 1\n"

    colour_options = list(POSSIBLE_COLOURS.keys())
    shape_options = POSSIBLE_SHAPES
    n_ref_shapes = T2_N_REF_SHAPES
    rot_lims = [0, 2*np.math.pi] if T2_ANY_ORIENTATION else [0, 0]

    # randomise shapes
    rand_shapes = np.random.permutation(shape_options)

    # check we have enough possible shapes to use as references
    if n_ref_shapes > len(shape_options):
      rospy.logwarn("T2_N_REF_SHAPES is greater than number of possible shapes")
      n_ref_shapes = len(shape_options) # local change only
    if n_ref_shapes > len(T2_OBJECT_REF_POINTS):
      rospy.logwarn("T2_N_REF_SHAPES is greater than number of reference positions")
      n_ref_shapes = len(T2_OBJECT_REF_POINTS) # local change only

    # clip our random shapes by the number of references
    rand_shapes = rand_shapes[:n_ref_shapes]

    # loop and spawn our references shapes
    for i in range(n_ref_shapes):

      rand_colour = np.random.choice(colour_options)
      rand_object = rand_shapes[i]
      if T2_USE_MULTIPLE_SIZES:
        rand_size = np.random.choice(POSSIBLE_SIZES)
      else: rand_size = POSSIBLE_SIZES[0]
      random_ref = rand_object + "_" + rand_colour + "_" + rand_size + "mm"
      model = self.spawn_model(name=random_ref, point=T2_OBJECT_REF_POINTS[i], rotationlims=rot_lims)
      TASK_MODELS_DICT["ref-"+random_ref] = model

    # spawn the mystery object
    rand_colour = np.random.choice(colour_options)
    rand_object = np.random.choice(rand_shapes)
    if T2_USE_MULTIPLE_SIZES:
      rand_size = np.random.choice(POSSIBLE_SIZES)
    else: rand_size = POSSIBLE_SIZES[0]
    random_object = rand_object + "_" + rand_colour + "_" + rand_size + "mm"
    model = self.spawn_model(name=random_object, rotationlims=rot_lims, xlims=T2_SHAPE_X_LIMS,
                      ylims=T2_SHAPE_Y_LIMS)
    TASK_MODELS_DICT["obj-"+random_object] = model
  
  # same as scenario 21 but different seed
  elif validation_scenario == 2:
    T2_ANY_ORIENTATION = False
    T2_USE_MULTIPLE_SIZES = False

    # set seed
    myseed = 7
    np.random.seed(myseed) # choose any int as your seed

    T2_GROUND_PLANE_NOISE = 40e-3
    tile_height = np.random.random() * (T2_GROUND_PLANE_NOISE)
    self.reset_task(respawn_tiles=True, tile_height=tile_height)

    rospy.loginfo("Creating validation scenario 2")
    TEST_REPORT_STR += "Creating validation scenario 2\n"

    colour_options = list(POSSIBLE_COLOURS.keys())
    shape_options = POSSIBLE_SHAPES
    n_ref_shapes = T2_N_REF_SHAPES
    rot_lims = [0, 2*np.math.pi] if T2_ANY_ORIENTATION else [0, 0]

    # randomise shapes
    rand_shapes = np.random.permutation(shape_options)

    # check we have enough possible shapes to use as references
    if n_ref_shapes > len(shape_options):
      rospy.logwarn("T2_N_REF_SHAPES is greater than number of possible shapes")
      n_ref_shapes = len(shape_options) # local change only
    if n_ref_shapes > len(T2_OBJECT_REF_POINTS):
      rospy.logwarn("T2_N_REF_SHAPES is greater than number of reference positions")
      n_ref_shapes = len(T2_OBJECT_REF_POINTS) # local change only

    # clip our random shapes by the number of references
    rand_shapes = rand_shapes[:n_ref_shapes]

    # loop and spawn our references shapes
    for i in range(n_ref_shapes):

      rand_colour = np.random.choice(colour_options)
      rand_object = rand_shapes[i]
      if T2_USE_MULTIPLE_SIZES:
        rand_size = np.random.choice(POSSIBLE_SIZES)
      else: rand_size = POSSIBLE_SIZES[0]
      random_ref = rand_object + "_" + rand_colour + "_" + rand_size + "mm"
      model = self.spawn_model(name=random_ref, point=T2_OBJECT_REF_POINTS[i], rotationlims=rot_lims)
      TASK_MODELS_DICT["ref-"+random_ref] = model

    # spawn the mystery object
    rand_colour = np.random.choice(colour_options)
    rand_object = np.random.choice(rand_shapes)
    if T2_USE_MULTIPLE_SIZES:
      rand_size = np.random.choice(POSSIBLE_SIZES)
    else: rand_size = POSSIBLE_SIZES[0]
    random_object = rand_object + "_" + rand_colour + "_" + rand_size + "mm"
    model = self.spawn_model(name=random_object, rotationlims=rot_lims, xlims=T2_SHAPE_X_LIMS,
                      ylims=T2_SHAPE_Y_LIMS)
    TASK_MODELS_DICT["obj-"+random_object] = model


  return  

# validation method, not for students
def task2_begin_test(self, validation_scenario):

  global TEST_REPORT_STR, TASK_MODELS_DICT

  t2_grade = 0
  ##### START TASK TIMER
  # rospy.logdebug("Starting Task2 timer.")
  # self.trial_timeout = False
  # rospy.Timer(rospy.Duration(T2_TEST_TIME), self.stop_callback, oneshot=True)
  rospy.logwarn(f"Evaluating Task 2 Validation Scenario {validation_scenario}\n")
  ##### SEND TASK REQUEST
  success = self.prepare_for_task_request(self.service_to_request)
  if success == False:
    rospy.loginfo("[RESULT - %d/100] Task2 - Fail, no service found!" % t2_grade )
    TEST_REPORT_STR += "[RESULT - %d/100] Task2 - Fail, no service found!\n" % t2_grade
    return t2_grade

  
  # find correct answer
  
  names = [name.split("-")[1].split("_")[0] for name in TASK_MODELS_DICT.keys()]
  print(names)
  if names[0] == names[2]:
    correct_num = 1
  elif names[1] == names[2]:
    correct_num = 2
  else:
    rospy.logerr("Test Failed, mystery shape different from references! Please restart test")
    raise RuntimeError("Test failed, mystery shape different from references! Please restart test")
    

  # get reference points
  ref_points = []
  for i in range(len(self.models) - 1):
    point_st = PointStamped()
    point_st.point = self.models[i].get_model_state().pose.position
    point_st.header.frame_id = "panda_link0"
    point_st.header.stamp = rospy.Time.now()
    ref_points.append(point_st)
  
  mystery_point = PointStamped()
  mystery_point.point = self.models[-1].get_model_state().pose.position
  mystery_point.header.frame_id = "panda_link0"
  mystery_point.header.stamp = rospy.Time.now()

  resp = self.send_task2_request(ref_points, mystery_point)
  # get returned response val
  returned_num =  resp.mystery_object_num

  t2_grade = 0 
  if returned_num == correct_num:
    t2_grade = 100
    rospy.loginfo("[RESULT - %d/100] Task2 - Success"% ( 
                    int(t2_grade)))
    TEST_REPORT_STR += "[RESULT - %d/100] Task2 - Success\n"% ( 
                    int(t2_grade))
    
    return True
  else:
    rospy.loginfo("[RESULT - %d/100] Task2 - Failed to identify mystery shape"% ( 
                    int(t2_grade)))
    TEST_REPORT_STR += "[RESULT - %d/100] Task2 - Failed to identify mystery shape\n"% ( 
                    int(t2_grade))
    return False
  

# validation method, not for students
def task3_spawn_test_objects(self, validation_scenario):
  
  # get required globals
  global TEST_REPORT_STR, T3_GROUND_PLANE_NOISE, T3_ANY_ORIENTATION, T3_USE_MULTIPLE_SIZES, TASK_MODELS_DICT, T3_N_OBSTACLES, T3_MAX_SHAPES, POSSIBLE_SIZES, T3_COMMON_SHAPES_DICT

  # reset task models
  TASK_MODELS_DICT =  {}
  T3_COMMON_SHAPES_DICT = {}
  # task 3 parameters
                          # maximum number of spawned shapes
  T3_SHAPE_X_LIMS = [-0.6, 0.7]            # xrange a shape can spawn
  T3_SHAPE_Y_LIMS = [-0.55, 0.55]          # yrange a shape can spawn

  T3_ANY_ORIENTATION = False               # do we allow any rotation of a shape
  T3_USE_MULTIPLE_SIZES = False 
  POSSIBLE_SIZES = ["40", "30", "20"] 

  TEST_REPORT_STR += "\nTesting for task 3\n"
  if validation_scenario == 0:

    T3_ANY_ORIENTATION = False
    T3_USE_MULTIPLE_SIZES = False
    T3_N_OBSTACLES = 0
    T3_MAX_SHAPES = 7


    # set seed
    myseed = 9
    np.random.seed(myseed) # choose any int as your seed


    T3_GROUND_PLANE_NOISE = 0e-3
    tile_height = np.random.random() * (T3_GROUND_PLANE_NOISE)
    self.reset_task(respawn_tiles=True, tile_height=tile_height)

    rospy.loginfo("Creating validation scenario 0")
    TEST_REPORT_STR += "Creating validation scenario 0\n"

    # spawn task objects
    colour_options = list(POSSIBLE_COLOURS.keys())
    rot_lims = [0, 2*np.math.pi] if T3_ANY_ORIENTATION else [0, 0]
    n_objects = np.random.random_integers(T3_MAX_SHAPES // 2, T3_MAX_SHAPES)

    # spawn a goal basket
    random_goal = BASKET_LOCATIONS[np.random.random_integers(0, len(BASKET_LOCATIONS)) - 1]
    self.spawn_model(name="basket", point=random_goal)

    # record where the panda and basket are so we don't spawn objects on them
    panda = [(0, 0), World.robot_safety_radius * np.math.sqrt(2)]
    basket = [
      (self.models[0].get_model_state().pose.position.x, 
      self.models[0].get_model_state().pose.position.y),
      0.5 * World.basket_side_length * np.math.sqrt(2)
    ]
    self.spawned_points = [panda, basket]

    # spawn a number of obstacles
    noughts = 0
    crosses = 0
    for i in range(T3_N_OBSTACLES):

      random_obstacle = POSSIBLE_OBSTACLES[np.random.random_integers(0, len(POSSIBLE_OBSTACLES)) - 1]
      obstacle_size = 200e-3
      obstacle_name = random_obstacle[0]
      obstacle_spawn_height = random_obstacle[1] + self.ground_level + 10e-3
      spawn_point = self.find_empty_point(obstacle_size, T3_SHAPE_X_LIMS, 
                                          T3_SHAPE_Y_LIMS, self.spawned_points)
      if spawn_point is not None:
        self.spawn_model(name=obstacle_name, point=spawn_point, rotationlims=rot_lims,
                         spawn_height=obstacle_spawn_height)
        self.spawned_points.append([spawn_point, obstacle_size])
      else: break

    # spawn a number of random objects in empty gaps
    for i in range(n_objects):

      # spawn a random object
      rand_colour = np.random.choice(colour_options)
      rand_object = np.random.choice(POSSIBLE_SHAPES)
      if rand_object == "nought":
        noughts+=1
      else:
        crosses+=1
      if T3_USE_MULTIPLE_SIZES:
        rand_size = np.random.choice(POSSIBLE_SIZES)
      else: rand_size = POSSIBLE_SIZES[0]
      random_object = rand_object + "_" + rand_colour + "_" + rand_size + "mm"
      random_size = int(rand_size) * 5 * 1e-3
      spawn_point = self.find_empty_point(random_size, T3_SHAPE_X_LIMS, 
                                          T3_SHAPE_Y_LIMS, self.spawned_points)
      if spawn_point is not None:
        shape_model = self.spawn_model(name=random_object, point=spawn_point, rotationlims=rot_lims)
        TASK_MODELS_DICT[rand_object+'_'+str(i)] = shape_model
        self.spawned_points.append([spawn_point, random_size])
      else: break
  elif validation_scenario == 1: # 2 obstacles, 6 objects

    T3_ANY_ORIENTATION = True
    T3_USE_MULTIPLE_SIZES = True
    T3_N_OBSTACLES = 2
    T3_MAX_SHAPES = 6


    # set seed
    myseed = 2
    np.random.seed(myseed) # choose any int as your seed

    T3_GROUND_PLANE_NOISE = 0e-3
    tile_height = np.random.random() * (T3_GROUND_PLANE_NOISE)
    self.reset_task(respawn_tiles=True, tile_height=tile_height)

    rospy.loginfo("Creating validation scenario 1")
    TEST_REPORT_STR += "Creating validation scenario 1\n"

    # spawn task objects
    colour_options = list(POSSIBLE_COLOURS.keys())
    rot_lims = [0, 2*np.math.pi] if T3_ANY_ORIENTATION else [0, 0]
    n_objects = np.random.random_integers(T3_MAX_SHAPES // 2, T3_MAX_SHAPES)

    # spawn a goal basket
    random_goal = BASKET_LOCATIONS[np.random.random_integers(0, len(BASKET_LOCATIONS)) - 1]
    self.spawn_model(name="basket", point=random_goal)

    # record where the panda and basket are so we don't spawn objects on them
    panda = [(0, 0), World.robot_safety_radius * np.math.sqrt(2)]
    basket = [
      (self.models[0].get_model_state().pose.position.x, 
      self.models[0].get_model_state().pose.position.y),
      0.5 * World.basket_side_length * np.math.sqrt(2)
    ]
    self.spawned_points = [panda, basket]

    # spawn a number of obstacles
    noughts = 0
    crosses = 0
    for i in range(T3_N_OBSTACLES):

      random_obstacle = POSSIBLE_OBSTACLES[np.random.random_integers(0, len(POSSIBLE_OBSTACLES)) - 1]
      obstacle_size = 200e-3
      obstacle_name = random_obstacle[0]
      obstacle_spawn_height = random_obstacle[1] + self.ground_level + 10e-3
      spawn_point = self.find_empty_point(obstacle_size, T3_SHAPE_X_LIMS, 
                                          T3_SHAPE_Y_LIMS, self.spawned_points)
      if spawn_point is not None:
        self.spawn_model(name=obstacle_name, point=spawn_point, rotationlims=rot_lims,
                         spawn_height=obstacle_spawn_height)
        self.spawned_points.append([spawn_point, obstacle_size])
      else: break

    # spawn a number of random objects in empty gaps
    for i in range(n_objects):

      # spawn a random object
      rand_colour = np.random.choice(colour_options)
      rand_object = np.random.choice(POSSIBLE_SHAPES)
      if rand_object == "nought":
        noughts+=1
      else:
        crosses+=1
      if T3_USE_MULTIPLE_SIZES:
        rand_size = np.random.choice(POSSIBLE_SIZES)
      else: rand_size = POSSIBLE_SIZES[0]
      random_object = rand_object + "_" + rand_colour + "_" + rand_size + "mm"
      random_size = int(rand_size) * 5 * 1e-3
      spawn_point = self.find_empty_point(random_size, T3_SHAPE_X_LIMS, 
                                          T3_SHAPE_Y_LIMS, self.spawned_points)
      if spawn_point is not None:
        shape_model = self.spawn_model(name=random_object, point=spawn_point, rotationlims=rot_lims)
        TASK_MODELS_DICT[rand_object+'_'+str(i)] = shape_model
        self.spawned_points.append([spawn_point, random_size])
      else: break
  elif validation_scenario == 2:

    T3_ANY_ORIENTATION = True
    T3_USE_MULTIPLE_SIZES = True
    T3_N_OBSTACLES = 3
    T3_MAX_SHAPES = 7
    POSSIBLE_SIZES = ["20"]

    # set seed
    myseed = 4
    np.random.seed(myseed) # choose any int as your seed


    T3_GROUND_PLANE_NOISE = 0e-3
    tile_height = np.random.random() * (T3_GROUND_PLANE_NOISE)
    self.reset_task(respawn_tiles=True, tile_height=tile_height)

    rospy.loginfo("Creating validation scenario 2")
    TEST_REPORT_STR += "Creating validation scenario 2\n"

    # spawn task objects
    colour_options = list(POSSIBLE_COLOURS.keys())
    rot_lims = [0, 2*np.math.pi] if T3_ANY_ORIENTATION else [0, 0]
    n_objects = np.random.random_integers(T3_MAX_SHAPES // 2, T3_MAX_SHAPES)

    # spawn a goal basket
    random_goal = BASKET_LOCATIONS[np.random.random_integers(0, len(BASKET_LOCATIONS)) - 1]
    self.spawn_model(name="basket", point=random_goal)

    # record where the panda and basket are so we don't spawn objects on them
    panda = [(0, 0), World.robot_safety_radius * np.math.sqrt(2)]
    basket = [
      (self.models[0].get_model_state().pose.position.x, 
      self.models[0].get_model_state().pose.position.y),
      0.5 * World.basket_side_length * np.math.sqrt(2)
    ]
    self.spawned_points = [panda, basket]

    # spawn a number of obstacles
    noughts = 0
    crosses = 0
    for i in range(T3_N_OBSTACLES):

      random_obstacle = POSSIBLE_OBSTACLES[np.random.random_integers(0, len(POSSIBLE_OBSTACLES)) - 1]
      obstacle_size = 200e-3
      obstacle_name = random_obstacle[0]
      obstacle_spawn_height = random_obstacle[1] + self.ground_level + 10e-3
      spawn_point = self.find_empty_point(obstacle_size, T3_SHAPE_X_LIMS, 
                                          T3_SHAPE_Y_LIMS, self.spawned_points)
      if spawn_point is not None:
        self.spawn_model(name=obstacle_name, point=spawn_point, rotationlims=rot_lims,
                         spawn_height=obstacle_spawn_height)
        self.spawned_points.append([spawn_point, obstacle_size])
      else: break

    # spawn a number of random objects in empty gaps
    for i in range(n_objects):

      # spawn a random object
      rand_colour = np.random.choice(colour_options)
      rand_object = np.random.choice(POSSIBLE_SHAPES)
      if rand_object == "nought":
        noughts+=1
      else:
        crosses+=1
      if T3_USE_MULTIPLE_SIZES:
        rand_size = np.random.choice(POSSIBLE_SIZES)
      else: rand_size = POSSIBLE_SIZES[0]
      random_object = rand_object + "_" + rand_colour + "_" + rand_size + "mm"
      random_size = int(rand_size) * 5 * 1e-3
      spawn_point = self.find_empty_point(random_size, T3_SHAPE_X_LIMS, 
                                          T3_SHAPE_Y_LIMS, self.spawned_points)
      if spawn_point is not None:
        shape_model = self.spawn_model(name=random_object, point=spawn_point, rotationlims=rot_lims)
        TASK_MODELS_DICT[rand_object+'_'+str(i)] = shape_model
        self.spawned_points.append([spawn_point, random_size])
      else: break
  elif validation_scenario == 3:

    T3_ANY_ORIENTATION = True
    T3_USE_MULTIPLE_SIZES = True
    T3_N_OBSTACLES = 0
    T3_MAX_SHAPES = 7
    POSSIBLE_SIZES = ["20"]   
    T3_SHAPE_X_LIMS = [0.30, 0.7]           # xrange a shape can spawn
    T3_SHAPE_Y_LIMS = [-0.55, 0.55]          # yrange a shape can spawn


    # set seed #1
    myseed = 1
    np.random.seed(myseed) # choose any int as your seed

    T3_GROUND_PLANE_NOISE = 0e-3
    tile_height = np.random.random() * (T3_GROUND_PLANE_NOISE)
    self.reset_task(respawn_tiles=True, tile_height=tile_height)

    rospy.loginfo("Creating validation scenario 3")
    TEST_REPORT_STR += "Creating validation scenario 3\n"

    # spawn task objects
    colour_options = list(POSSIBLE_COLOURS.keys())
    rot_lims = [0, 2*np.math.pi] if T3_ANY_ORIENTATION else [0, 0]
    n_objects = np.random.random_integers(T3_MAX_SHAPES // 2, T3_MAX_SHAPES)

    # spawn a goal basket
    random_goal = BASKET_LOCATIONS[np.random.random_integers(0, len(BASKET_LOCATIONS)) - 1]
    self.spawn_model(name="basket", point=random_goal)

    # record where the panda and basket are so we don't spawn objects on them
    panda = [(0, 0), World.robot_safety_radius * np.math.sqrt(2)]
    basket = [
      (self.models[0].get_model_state().pose.position.x, 
      self.models[0].get_model_state().pose.position.y),
      0.5 * World.basket_side_length * np.math.sqrt(2)
    ]
    self.spawned_points = [panda, basket]

    # spawn a number of obstacles
    noughts = 0
    crosses = 0
    for i in range(T3_N_OBSTACLES):

      random_obstacle = POSSIBLE_OBSTACLES[np.random.random_integers(0, len(POSSIBLE_OBSTACLES)) - 1]
      obstacle_size = 200e-3
      obstacle_name = random_obstacle[0]
      obstacle_spawn_height = random_obstacle[1] + self.ground_level + 10e-3
      spawn_point = self.find_empty_point(obstacle_size, T3_SHAPE_X_LIMS, 
                                          T3_SHAPE_Y_LIMS, self.spawned_points)
      if spawn_point is not None:
        self.spawn_model(name=obstacle_name, point=spawn_point, rotationlims=rot_lims,
                         spawn_height=obstacle_spawn_height)
        self.spawned_points.append([spawn_point, obstacle_size])
      else: break

    # spawn a number of random objects in empty gaps
    for i in range(n_objects):

      # spawn a random object
      rand_colour = np.random.choice(colour_options)
      rand_object = np.random.choice(POSSIBLE_SHAPES)
      if rand_object == "nought":
        noughts+=1
      else:
        crosses+=1
      if T3_USE_MULTIPLE_SIZES:
        rand_size = np.random.choice(POSSIBLE_SIZES)
      else: rand_size = POSSIBLE_SIZES[0]
      random_object = rand_object + "_" + rand_colour + "_" + rand_size + "mm"
      random_size = int(rand_size) * 5 * 1e-3
      spawn_point = self.find_empty_point(random_size, T3_SHAPE_X_LIMS, 
                                          T3_SHAPE_Y_LIMS, self.spawned_points)
      if spawn_point is not None:
        shape_model = self.spawn_model(name=random_object, point=spawn_point, rotationlims=rot_lims)
        TASK_MODELS_DICT[rand_object+'_'+str(i)] = shape_model
        self.spawned_points.append([spawn_point, random_size])
      else: break

  T3_COMMON_SHAPES_DICT["nought"]=noughts
  T3_COMMON_SHAPES_DICT["cross"]=crosses

  return 

# validation method, not for students
def task3_begin_test(self, validation_scenario):

  # get required globals
  global TEST_REPORT_STR, TASK_MODELS_DICT, T3_COMMON_SHAPES_DICT

  t3_grade = 0
  ##### START TASK TIMER
  # rospy.logdebug("Starting Task2 timer.")
  # self.trial_timeout = False
  # rospy.Timer(rospy.Duration(T2_TEST_TIME), self.stop_callback, oneshot=True)
  rospy.logwarn(f"Evaluating Task 3 Validation Scenario {validation_scenario}\n")
  ##### SEND TASK REQUEST
  success = self.prepare_for_task_request(self.service_to_request)
  if success == False:
    rospy.loginfo("[RESULT - %d/100] Task2 - Fail, no service found!" % t3_grade )
    TEST_REPORT_STR += "[RESULT - %d/100] Task2 - Fail, no service found!\n" % t3_grade
    return 
  
  resp = self.send_task3_request()

  result_common_shapes = resp.num_most_common_shape
  result_total_num_shapes = resp.total_num_shapes

  correct_total_shapes = len(TASK_MODELS_DICT.keys())
  correct_num_common_shapes = max(T3_COMMON_SHAPES_DICT["cross"], T3_COMMON_SHAPES_DICT["nought"])
  rospy.loginfo("Correct Values %d total_shapes and %d num most common shape"%(correct_total_shapes, correct_num_common_shapes))
  rospy.loginfo("Service returned %d total_shapes and %d num most common shape"%(result_total_num_shapes, result_common_shapes))

  # check if returned integers are are correct
  if correct_total_shapes == result_total_num_shapes:
    t3_grade = 100
    rospy.loginfo("[RESULT - %d/100] Task3 - Total shapes is correct "% ( 
                    int(t3_grade)))
    TEST_REPORT_STR += "[RESULT - %d/100] Task3 - Success\n"% ( 
                    int(t3_grade))
  else:
    rospy.loginfo("[RESULT - %d/100] Task3 - Incorrect number of total shapes"% ( 
                    int(t3_grade)))
    TEST_REPORT_STR += "[RESULT - %d/100] Task3 - Incorrect number of total shapes\n"% ( 
                    int(t3_grade))

  if correct_num_common_shapes == result_common_shapes:
    t3_grade = 100
    rospy.loginfo("[RESULT - %d/100] Task3 - Num common shapes is correct "% ( 
                    int(t3_grade)))
    TEST_REPORT_STR += "[RESULT - %d/100] Task3 - Success\n"% ( 
                    int(t3_grade))
  else:
    rospy.loginfo("[RESULT - %d/100] Task3 - Incorrect number of common shapes"% ( 
                    int(t3_grade)))
    TEST_REPORT_STR += "[RESULT - %d/100] Task3 - Incorrect number of common shapes\n"% ( 
                    int(t3_grade))
  
  # create list of correct (common) shapes
  if T3_COMMON_SHAPES_DICT["cross"] > T3_COMMON_SHAPES_DICT["nought"]:
    correct_shapes = ["cross"]
  elif T3_COMMON_SHAPES_DICT["cross"] < T3_COMMON_SHAPES_DICT["nought"]:
    correct_shapes = ["nought"]
  else:
    correct_shapes = ["cross", "nought"]

  # get basket position
  basket_pos = self.spawned_points[1]
  goal_pos_np = np.array([basket_pos[0][0], basket_pos[0][1], basket_pos[1]])
  # award marks for placing any shape

  # rest task 3 grade
  t3_grade = 0
  for key, shape_model in TASK_MODELS_DICT.items():
    name = key.split('_')[0]

    final_pose = shape_model.get_model_state().pose
    final_pos_np = self.get_position_from_pose(final_pose)
    dx,dy,dz = np.abs(goal_pos_np - final_pos_np)

    if dx < World.basket_side_length/2. and dy < World.basket_side_length/2.:
      if name in correct_shapes:  # check if placed shape is the correct one
        t3_grade = 100
        rospy.loginfo("[RESULT - %d/100] Task3 - Successful pick and place - Correct shape!"%t3_grade)
        TEST_REPORT_STR += "[RESULT - %d/100] Task3 - Successful pick and place - Correct!\n"%t3_grade
      else: # otherwise add marks for placing any shape
        t3_grade = 30
        rospy.loginfo("[RESULT - %d/100] Task3 - Successful pick and place - Incorrect shape!"%t3_grade)
        TEST_REPORT_STR += "[RESULT - %d/100] Task3 - Successful pick and place - Incorrect shape!\n"%t3_grade


      return True
    else:
      continue
    
  if t3_grade == 0:
    rospy.loginfo("[RESULT - %d/100] Task3 - No shape in basket, pick and place failed!"%t3_grade)
    TEST_REPORT_STR += "[RESULT - %d/100] Task3 - No shape in basket, pick and place failed!\n"%t3_grade
      
    return False

# apply these functions to the base class
Task1.spawn_test_objects = task1_spawn_test_objects
Task1.begin_test = task1_begin_test
Task2.spawn_test_objects = task2_spawn_test_objects
Task2.begin_test = task2_begin_test
Task3.spawn_test_objects = task3_spawn_test_objects
Task3.begin_test = task3_begin_test

def run_task1_validation(validation_scenario=None):
  # wipe the test report string
  global TEST_REPORT_STR
  if validation_scenario == None:
    TEST_REPORT_STR= """Test report for task 1:\n\n"""
    rospy.loginfo("Running task 1 validation battery")
    Task1(mode='validation', validation_scenario=0)
    Task1(mode='validation', validation_scenario=1)
    TEST_REPORT_STR += "\nEnd of test"
    print(TEST_REPORT_STR)
  elif validation_scenario == 0:
    TEST_REPORT_STR= """Test report for task 1:\n\n"""
    rospy.loginfo("Running task 1 validation 0 only")
    Task1(mode='validation', validation_scenario=0)
    TEST_REPORT_STR += "\nEnd of test"
    print(TEST_REPORT_STR) 
  elif validation_scenario == 1:
    TEST_REPORT_STR= """Test report for task 1:\n\n"""
    rospy.loginfo("Running task 1 validation 1 only")
    Task1(mode='validation', validation_scenario=1)
    TEST_REPORT_STR += "\nEnd of test"
    print(TEST_REPORT_STR)     

def run_task2_validation(validation_scenario=None):
  # wipe the test report string
  global TEST_REPORT_STR
  if validation_scenario == None:
    TEST_REPORT_STR= """Test report for task 2:\n\n"""
    rospy.loginfo("Running task 2 validation battery")
    Task2(mode='validation', validation_scenario=0)
    Task2(mode='validation', validation_scenario=1)
    Task2(mode='validation', validation_scenario=2)
    
    TEST_REPORT_STR += "\nEnd of test"
    print(TEST_REPORT_STR)
  elif validation_scenario == 0:
    TEST_REPORT_STR= """Test report for task 2:\n\n"""
    rospy.loginfo("Running task 2 validation 0 only")
    Task2(mode='validation', validation_scenario=0)
    TEST_REPORT_STR += "\nEnd of test"
    print(TEST_REPORT_STR) 
  elif validation_scenario == 1:
    TEST_REPORT_STR= """Test report for task 2:\n\n"""
    rospy.loginfo("Running task 2 validation 1 only")
    Task2(mode='validation', validation_scenario=1)
    TEST_REPORT_STR += "\nEnd of test"
    print(TEST_REPORT_STR)
  elif validation_scenario == 2:
    TEST_REPORT_STR= """Test report for task 2:\n\n"""
    rospy.loginfo("Running task 2 validation 2 only")
    Task2(mode='validation', validation_scenario=2)
    TEST_REPORT_STR += "\nEnd of test"
    print(TEST_REPORT_STR)

def run_task3_validation(validation_scenario=None):
  # wipe the test report string
  global TEST_REPORT_STR
  if validation_scenario == None:
    TEST_REPORT_STR= """Test report for task 3:\n\n"""
    rospy.loginfo("Running task 3 validation battery")
    Task3(mode='validation', validation_scenario=0)
    Task3(mode='validation', validation_scenario=1)
    Task3(mode='validation', validation_scenario=2)
    Task3(mode='validation', validation_scenario=3)   
    TEST_REPORT_STR += "\nEnd of test"
    print(TEST_REPORT_STR)
  elif validation_scenario == 0:
    TEST_REPORT_STR= """Test report for task 3:\n\n"""
    rospy.loginfo("Running task 3 validation 0 only")
    Task3(mode='validation', validation_scenario=0)
    TEST_REPORT_STR += "\nEnd of test"
    print(TEST_REPORT_STR) 
  elif validation_scenario == 1:
    TEST_REPORT_STR= """Test report for task 3:\n\n"""
    rospy.loginfo("Running task 3 validation 1 only")
    Task3(mode='validation', validation_scenario=1)
    TEST_REPORT_STR += "\nEnd of test"
    print(TEST_REPORT_STR)
  elif validation_scenario == 2:
    TEST_REPORT_STR= """Test report for task 3:\n\n"""
    rospy.loginfo("Running task 3 validation 2 only")
    Task3(mode='validation', validation_scenario=2)
    TEST_REPORT_STR += "\nEnd of test"
    print(TEST_REPORT_STR)
  elif validation_scenario == 3:
    TEST_REPORT_STR= """Test report for task 3:\n\n"""
    rospy.loginfo("Running task 3 validation 3 only")
    Task3(mode='validation', validation_scenario=3)
    TEST_REPORT_STR += "\nEnd of test"
    print(TEST_REPORT_STR)

def run_full_validation():
  rospy.loginfo("Running full coursework validation!")
  # wipe the test report string
  global TEST_REPORT_STR
  TEST_REPORT_STR= """Test report for all three tasks:\n\n"""
  # run tasks
  rospy.loginfo("Running task 1 validation battery")
  Task1(mode='validation', validation_scenario=0)
  Task1(mode='validation', validation_scenario=1)
  rospy.loginfo("Running task 2 validation battery")
  Task2(mode='validation', validation_scenario=0)
  Task2(mode='validation', validation_scenario=1)
  Task2(mode='validation', validation_scenario=2)
  rospy.loginfo("Running task 3 validation battery")
  Task3(mode='validation', validation_scenario=0)
  Task3(mode='validation', validation_scenario=1)
  Task3(mode='validation', validation_scenario=2)
  Task3(mode='validation', validation_scenario=3) 
  # print and save test report string
  TEST_REPORT_STR += "\nEnd of test"
  print(TEST_REPORT_STR)

def save_test_report():
  global TEST_REPORT_STR

  rospack = rospkg.RosPack()
  path_to_pkg = rospack.get_path("cw2_world_spawner")

  folders = path_to_pkg.split("/")
  # trim src/cw1_world_spawner
  folders = folders[:-2]
  savepath = "/"
  for f in folders:
    savepath += f + "/"

  team = [x for x in os.listdir(savepath + "/src/") if x.startswith("cw2_team_")]

  timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
  filename = savepath + team[0] + "_test_report_"+timestamp +".txt"

  with open(filename, 'w') as openfile:
        openfile.write(TEST_REPORT_STR)
        rospy.loginfo("Saved test report at: {}".format(filename))

def handle_task_request(req):

  rospy.loginfo("started handle_task_request")

  # Callback for selecting which task to start
  if req.task_index == 1:
    Task1(mode="coursework")
  elif req.task_index == 2:
    Task2(mode="coursework")
  elif req.task_index == 3:
    Task3(mode="coursework")
  else:
    rospy.logwarn("Unrecognized task requested")

  rospy.loginfo("finished handle_task_request")

  return TaskSetupResponse()

def handle_test_request(req):

  rospy.loginfo("enter handle_task_request:")
  rospy.loginfo("\nFull validation  all tasks- 10 \n Individual task X - X = {1,2,3} \n Specific scenario, e.g., XX0 for scenario 0 of task X ")


  # Callback for selecting which task to start
  # if req.task_index == 1:
  #   Task1(mode="validation")
  # elif req.task_index == 2:
  #   Task2(mode="validation")
  # elif req.task_index == 3:
  #   Task3(mode="validation")

  # testing methods, not for students
  if req.task_index == 0:
    run_full_validation()
  
  # validation for task 1
  elif req.task_index == 1:
    run_task1_validation()
  elif req.task_index == 10:
    run_task1_validation(0)
  elif req.task_index == 11:
    run_task1_validation(1)

  # validation for task 2
  elif req.task_index == 2:
    run_task2_validation()
  elif req.task_index == 20:
    run_task2_validation(0)
  elif req.task_index == 21:
    run_task2_validation(1)
  elif req.task_index == 22:
    run_task2_validation(2)

  # validation for task 3
  elif req.task_index == 3:
    run_task3_validation()
  elif req.task_index == 30:
    run_task3_validation(0)
  elif req.task_index == 31:
    run_task3_validation(1)
  elif req.task_index == 32:
    run_task3_validation(2)  
  elif req.task_index == 33:
    run_task3_validation(3) 
  # end not for students

  else:
    rospy.logwarn("Unrecognized task requested")

  rospy.loginfo("at end of handle_task_request")
  save_test_report()
  return TaskSetupResponse()

if __name__ == "__main__":

  # create the world and run the coursework /task service
  rospy.init_node('coursework3_test_wrapper')

  # create the /task service callback
  rospy.Service('/task', TaskSetup, handle_task_request)
  rospy.loginfo("Ready to initiate task.")
  rospy.loginfo("Use rosservice call /task <INDEX> to start a task")
  
  # START TEST SERVICE, not for students
  rospy.Service('/test', TaskSetup, handle_test_request)
  rospy.loginfo("Use rosservice call /test <INDEX> to start a test")

  # find the path to the models folder and print a helpful warning
  import rospkg, os
  rospack = rospkg.RosPack()
  rospath = rospack.get_path("cw2_world_spawner")
  pkg_models_path = rospath + "/models"
  export_cmd = f"export GAZEBO_MODEL_PATH={pkg_models_path}"
  rospy.logwarn(f"If Gazebo freezes when you call a task you need to run in your terminal the following: {export_cmd}")

  rospy.logwarn("Don't forget you are running test_coursework.py")

  rospy.spin()

# THIS FILE MUST NEVER BE RELEASED TO STUDENTS
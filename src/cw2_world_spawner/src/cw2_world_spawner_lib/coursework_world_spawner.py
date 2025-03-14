import rospy, rospkg
import gazebo_ros
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState, GetWorldProperties

from geometry_msgs.msg import Pose, PoseStamped

import os
from glob import glob
import numpy as np
"""
Spawn online model

rosrun gazebo_ros spawn_model -database coke_can -sdf -model coke_can -y 1.2 -x -0.0

Move existing model 

rosservice call /gazebo/set_model_state '{model_state: 
  { model_name: coke_can, 
    pose: { 
      position: { x: 0.3, y: 0.2 ,z: 0 }, 
      orientation: {x: 0, y: 0.491983115673, z: 0, w: 0.870604813099 } 
    }, 
    twist: { 
      linear: {x: 0.0 , y: 0 ,z: 0 } , 
      angular: { x: 0.0 , y: 0 , z: 0.0 } 
    } ,
    reference_frame: world } 
  }'

"""

"""
spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
spawn_model_client(
    model_name='ground_plane',
    model_xml=open('/usr/share/gazebo-9/models/ground_plane/model.sdf', 'r').read(),
    robot_namespace='/foo',
    initial_pose=Pose(),
    reference_frame='world'
)
"""
def quaternion_to_euler(x, y, z, w):

  import math
  t0 = +2.0 * (w * x + y * z)
  t1 = +1.0 - 2.0 * (x * x + y * y)
  X = math.degrees(math.atan2(t0, t1))

  t2 = +2.0 * (w * y - z * x)
  t2 = +1.0 if t2 > +1.0 else t2
  t2 = -1.0 if t2 < -1.0 else t2
  Y = math.degrees(math.asin(t2))

  t3 = +2.0 * (w * z + x * y)
  t4 = +1.0 - 2.0 * (y * y + z * z)
  Z = math.degrees(math.atan2(t3, t4))

  return X, Y, Z

def euler_to_quaternion(yaw, pitch, roll):

  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

  return np.array([qx, qy, qz, qw])

def vectors_to_ros_pose(pos, q):
  pose = Pose()
  pose.position.x = pos[0]
  pose.position.y = pos[1]
  pose.position.z = pos[2]
  pose.orientation.x = q[0]
  pose.orientation.y = q[1]
  pose.orientation.z = q[2]
  pose.orientation.w = q[3]
  return pose

def random_position_in_area(xlims, ylims, zlims):
  x = np.random.uniform(xlims[0], xlims[1])
  y = np.random.uniform(ylims[0], ylims[1])
  z = np.random.uniform(zlims[0], zlims[1])
  return np.asarray([x,y,z])

def random_orientation(roll_lims, pitch_lims, yaw_lims):
  roll  = np.random.uniform(roll_lims[0], roll_lims[1])
  pitch = np.random.uniform(pitch_lims[0], pitch_lims[1])
  yaw   = np.random.uniform(yaw_lims[0], yaw_lims[1])
  q = euler_to_quaternion(roll, pitch, yaw)
  return q

# def random_pose_in_area(xlims, ylims, zlims,
#                         roll_lims, pitch_lims, yaw_lims):
#   xyz = random_position_in_area(xlims, ylims, zlims)
#   q = random_orientation(roll_lims, pitch_lims, yaw_lims)
#   pose = vectors_to_ros_pose(xyz, q)
#   return pose

def get_random_upright_pose(xlims, ylims, zlims):
  xyz = random_position_in_area(xlims, ylims, zlims)
  pose = vectors_to_ros_pose(xyz, [0,0,0,1])
  return


class Model(object):
  def __init__(self, model_name, instance_name, model_type, 
               position, orientation=[0,0,0,1], scale=None):
    self.mdict = self.create_model_dict(model_name, instance_name, 
                                   model_type, position, 
                                   orientation, scale)
    return

  def create_model_dict(self, model_name, instance_name, model_type, 
                        position, orientation=[0,0,0,1], scale=None):
    if model_type == 'sdf':
      pose = vectors_to_ros_pose(position, orientation)
      mdict = dict(mtype = model_type, model_name = model_name, 
                  instance_name=instance_name, pose=pose)
      pass
    elif model_type == 'primitive':
      pass
    else: 
      mdict = None
      rospy.logerror('Wrong model type. Should be [model|primitive]')
    return mdict

  def get_model_state(self, relname="world"):
    get_model_state_srv = rospy.ServiceProxy('/gazebo/get_model_state',
                                              GetModelState)
    resp = get_model_state_srv(self.mdict["instance_name"], relname)
    return resp

  def despawn(self):
    delete_model = rospy.ServiceProxy('/gazebo/delete_model', 
                                              DeleteModel)
    delete_model(self.mdict["instance_name"])

    
class WorldSpawner(object):
  def __init__(self, extra_model_dirs = []):
    self.init_model_dirs(extra_model_dirs)
    self.init_model_names()
    return

  def init_model_dirs(self, extra_model_dirs):
    self.models_dirs = []
    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    rospath = rospack.get_path("cw2_world_spawner")
    pkg_models_path = rospath+"/models/"
      
    self.models_dirs += [ pkg_models_path ]
    self.models_dirs += extra_model_dirs

    rospy.loginfo("Model libraries used by cw2_world_spawner: ")
    for mdir in self.models_dirs:
      rospy.loginfo(mdir)
    return

  def init_model_names(self):
    self.model_names = set()
    for mdir in self.models_dirs:
      subfolders = os.listdir(mdir)
      self.model_names |= set(subfolders)
    return

  def find_model(self, model_name):
    if model_name not in self.model_names:
      rospy.logwarn("Requested model_name %s is not found"%(model_name))
      return None

    # Search for model name in all model directories
    for mdir in self.models_dirs :
      glob_query  = mdir + "/"+model_name+"/*.sdf"
      model_paths = glob(glob_query)
      if len(model_paths)>0:
        break 
    else:  # For/else clause means that the loop has finished without breaking
      rospy.logwarn("No .sdf found for model %s"%(model_name))
      return None

    path_to_model = model_paths[0]

    return path_to_model

  def spawn(self, model):
    model_dict = model.mdict
    if model_dict['mtype'] == 'sdf':
      model_name = model_dict['model_name']
      instance_name = model_dict['instance_name']
      pose = model_dict['pose']
      return self.spawn_model(model_name, instance_name, pose)
    elif model_dict['mtype'] == 'primitive':
      pass
      return False
    else:
      rospy.logerr("Unsupported model spawn type %s"%model_dict['mtype'])
      return False

  def spawn_model(self, model_name, instance_name, pose):
    path_to_model =  self.find_model(model_name)
    if path_to_model is None:
      return False

    rospy.logdebug("Spawning model of type %s"%model_name)
    sdff = open(path_to_model).read()

    rospy.wait_for_service('gazebo/spawn_sdf_model')
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client(
        model_name=instance_name,
        model_xml=sdff,
        robot_namespace='/WorldSpawner',
        initial_pose=pose,
        reference_frame='world'
    )
    return True

  def get_model_state_by_name(self, name, relname="world"):
    get_model_state_srv = rospy.ServiceProxy('/gazebo/get_model_state',
                                              GetModelState)
    resp = get_model_state_srv(name, relname)
    return resp

  def despawn_by_name(self, instance_name):
    delete_model = rospy.ServiceProxy('/gazebo/delete_model', 
                                              DeleteModel)
    resp = delete_model(instance_name)
    return resp
  
  def get_world_properties(self):
    world_props = rospy.ServiceProxy('/gazebo/get_world_properties', 
                                      GetWorldProperties)
    resp = world_props()
    return resp

  def despawn_all(self, keyword='object', exceptions='exception'):
    props = self.get_world_properties().model_names
    [self.despawn_by_name(name) for name in props if keyword in name and exceptions not in name] 
    return

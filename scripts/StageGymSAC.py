import numpy as np
import gymnasium as gym
from gymnasium import spaces
from numpy.core.fromnumeric import shape
import rospy
import rospkg
from rl_dovs.srv import *
from std_srvs.srv import Empty
from typing import Any, Dict, Optional, Union

from collections import OrderedDict
import tensorflow_datasets as tfds
import tensorflow as tf


# class DiscreteDOVS(spaces.Discrete):
#   def __init__(self, n: int, n_extra: int) -> None:
#       self.goal_actions = []
#       self.n_extra = n_extra
#       super().__init__(n)

class StageGym(gym.Env):
  """
  Custom Environment that follows gym interface.
  """

  def __init__(self, observation_len = 808):
    super(StageGym, self).__init__()
    self.action_space = spaces.Box(low=0.0, high=1.0, shape=(2,), dtype=np.float32)
    self.observation_space = spaces.Box(low=-float('inf'), high=float('inf'), shape=(observation_len,), dtype=np.float32)
    print("ROS node init, waiting for service to be advertised")

  def reset(self, seed=None):
    """
    Important: the observation must be a numpy array
    :return: (np.array) 
    """
    rospy.wait_for_service('/gym_reset')
    try:
        gym_reset = rospy.ServiceProxy('/gym_reset', SAC_gym_reset)
        resp = gym_reset(False)
        return np.array(resp.state), {}
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

  def step(self, action): 
    rospy.wait_for_service('/gym_step')
    try:
        gym_step = rospy.ServiceProxy('/gym_step', SAC_gym)
        resp = gym_step(action[0], action[1])
        return np.array(resp.state), resp.reward, resp.done, False, {}
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


  def render(self, mode='console'):
    print("RENDER ENV->This should never be called")
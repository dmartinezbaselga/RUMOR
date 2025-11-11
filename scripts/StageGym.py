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

  def __init__(self, dovs_shape, vector_shape, n_actions, observation_len = 808):
  # def __init__(self, total_size, n_actions) -> None:
    super(StageGym, self).__init__()

    # # Size of the 1D-grid
    # self.grid_size = grid_size
    # # Initialize the agent at the right of the grid
    # self.agent_pos = grid_size - 1

    # Define action and observation space
    # They must be gym.spaces objects
    # Example when using discrete actions, we have two: left and right
    # self.action_space = DiscreteDOVS(n_actions, 3)
    self.action_space = spaces.Box(low=0.0, high=1.0, shape=(2,), dtype=np.float32)
    # The observation will be the coordinate of the agent
    # this can be described both by Discrete and Box space
    self.observation_space = spaces.Box(low=-float('inf'), high=float('inf'), shape=(observation_len,), dtype=np.float32)
    # self.observation_space = spaces.Dict(
    #   spaces={
    #     "vec": spaces.Box(low=-float('inf'), high=float('inf'), shape=vector_shape, dtype=np.float32),
    #     "img": spaces.Box(low=0, high=1, shape=dovs_shape, dtype=np.float32)
    #   }
    # )    
    # self.observation_space = spaces.Tuple(
    #   spaces=
    #     [spaces.Box(low=-float('inf'), high=float('inf'), shape=vector_shape, dtype=np.float32),
    #     spaces.Box(low=0, high=1, shape=dovs_shape, dtype=np.float32)]
    # )
    # self.dovs_shape = dovs_shape
    # self.vector_shape = vector_shape
    self.n_actions = n_actions
    # rospy.init_node('StageGym')
    print("ROS node init, waiting for service to be advertised")
    # print("HEEEEEEEEEEEEEEEEY")
    # rospy.wait_for_service('/gym_step')
    print("Service is advertised")
    # gym_reset = rospy.ServiceProxy('/gym_reset', DQN_gym_reset)
    # resp = gym_reset(False)
    # # state = tfds.features.FeaturesDict({"vec": resp.state[-self.vector_shape[0]:], "img": np.array(resp.state[:-self.vector_shape[0]]).reshape(self.dovs_shape)})
    # state = np.array([np.array(resp.state[-self.vector_shape[0]:]), np.array(resp.state[:-self.vector_shape[0]]).reshape(self.dovs_shape)])
    # print("HOLAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA", np.shape(state))

  def reset(self, seed=None):
    """
    Important: the observation must be a numpy array
    :return: (np.array) 
    """
    # print("Waiting for service gym_rest")
    rospy.wait_for_service('/gym_reset')
    try:
        # print("Calling service")
        gym_reset = rospy.ServiceProxy('/gym_reset', DQN_gym_reset)
        resp = gym_reset(False)
        if resp.goal_actions is None:
          resp.goal_actions = []
        # print("Goal actions after if: ", resp.goal_actions)     
        forbidden_actions = np.zeros(self.n_actions, dtype=bool)
        forbidden_actions[-3:] = True
        for i in resp.goal_actions:
          forbidden_actions[i] = False
        self.initial_invalid_actions = {'invalid_actions': np.array(forbidden_actions, dtype=bool)}
        # state = tfds.features.FeaturesDict({"vec": resp.state[-self.vector_shape[0]:], "img": np.array(resp.state[:-self.vector_shape[0]]).reshape(self.dovs_shape)})
        # state = tf.tuple([np.array(resp.state[-self.vector_shape[0]:]), np.array(resp.state[:-self.vector_shape[0]]).reshape(self.dovs_shape)])
        # print("HOLAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA", np.shape(state))
        # self.goal_actions = resp.goal_actions
        # self.action_space.goal_actions = resp.goal_actions
        # desired_goal_data = resp.reward_data.copy()
        # desired_goal_data[:-1] = 0
        # state = OrderedDict(
        #   [
        #     ("vec", resp.state[-self.vector_shape[0]:]),
        #     ("img", np.array(resp.state[:-self.vector_shape[0]]).reshape(self.dovs_shape))
        #   ]
        # )  
        # state = [np.array(resp.state[-self.vector_shape[0]:]), np.array(resp.state[:-self.vector_shape[0]]).reshape(self.dovs_shape)]   
        # np.reshape(resp.state[-self.vector_shape[0]:], (1,7))
        # np.reshape(resp.state[:-self.vector_shape[0]], (1,) + self.dovs_shape)
        # state = [[np.reshape(resp.state[-self.vector_shape[0]:], (1,1,7)), np.reshape(resp.state[:-self.vector_shape[0]], (1,) + self.dovs_shape)]]  
        # print("Antes de return")
        # return np.reshape(resp.state, (1, -1))
        # print("SHAPE WE ARE USING: ", np.shape(np.concatenate((np.array(resp.state), np.array(forbidden_actions), [0.05]))))
        # print("Reset: ", np.array(resp.state).shape)
        return np.array(resp.state), {}
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    # Initialize the agent at the right of the grid
    # self.agent_pos = self.grid_size - 1
    # here we convert to float32 to make it more general (in case we want to use continuous actions)
    # return np.array([self.agent_pos]).astype(np.float32)
    # return -1

  def step(self, action):
    # if action == self.LEFT:
    #   self.agent_pos -= 1
    # elif action == self.RIGHT:
    #   self.agent_pos += 1
    # else:
    #   raise ValueError("Received invalid action={} which is not part of the action space".format(action))

    # # Account for the boundaries of the grid
    # self.agent_pos = np.clip(self.agent_pos, 0, self.grid_size)

    # # Are we at the left of the grid?
    # done = bool(self.agent_pos == 0)

    # # Null reward everywhere except when reaching the goal (left of the grid)
    # reward = 1 if self.agent_pos == 0 else 0

    # # Optionally we can pass additional info, we are not using that for now
    # info = {}
    # print("STEP ENV->This should never be called")
    rospy.wait_for_service('/gym_step')
    try:
        # if 
        gym_step = rospy.ServiceProxy('/gym_step', DQN_gym)
        action = 0
        resp = gym_step(action)
        # state = tf.tuple([np.array(resp.state[-self.vector_shape[0]:]), np.array(resp.state[:-self.vector_shape[0]]).reshape(self.dovs_shape)])
        # print("HOLAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA", np.shape(state))
        # desired_goal_data = resp.reward_data.copy()
        # desired_goal_data[:-1] = 0
        # state = OrderedDict(
        #   [
        #     ("vec", resp.state[-self.vector_shape[0]:]),
        #     ("img", np.array(resp.state[:-self.vector_shape[0]]).reshape(self.dovs_shape))
        #   ]
        # )   
        # state = np.concatenate([np.array(resp.state[-self.vector_shape[0]:]), np.array(resp.state[:-self.vector_shape[0]]).reshape(self.dovs_shape)])     
        # self.goal_actions = resp.goal_actions
        # self.action_space.goal_actions = resp.goal_actions
        # print("DONE RECEIVED: ", resp.done)
        # state = [[np.reshape(resp.state[-self.vector_shape[0]:], (1,1,7)), np.reshape(resp.state[:-self.vector_shape[0]], (1,) + self.dovs_shape)]]  
        # print(resp)
        # print("Vec state: ", resp.state[-8:])
        # print("Goal actions before if: ", resp.goal_actions)
        if resp.goal_actions is None:
          resp.goal_actions = []
        # print("Goal actions after if: ", resp.goal_actions)     
        forbidden_actions = np.zeros(self.n_actions, dtype=bool)
        forbidden_actions[-3:] = True
        for i in resp.goal_actions:
          forbidden_actions[i] = False
        # print(forbidden_actions)
        # print("Step: ", np.array(resp.state).shape)
        return np.array(resp.state), resp.reward, resp.done, False, {}
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    # return np.array([self.agent_pos]).astype(np.float32), reward, done, info
    # return 0

  def render(self, mode='console'):
    # if mode != 'console':
    #   raise NotImplementedError()
    # # agent is represented as a cross, rest as a dot
    # print("." * self.agent_pos, end="")
    # print("x", end="")
    # print("." * (self.grid_size - self.agent_pos))
    print("RENDER ENV->This should never be called")

  def compute_reward(self, achieved_goal: object, desired_goal: object, _info: Optional[Dict[str, Any]]) -> float:
      if (abs(achieved_goal[0]-desired_goal[0]) < 0.2 and abs(achieved_goal[1]-desired_goal[1] < achieved_goal[2])):
        return 1
      elif achieved_goal[3]:
        return -3
      else:
        return -1
  def close(self):
    pass
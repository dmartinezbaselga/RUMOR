from gym.spaces import space
import numpy as np
import gym
from gym import spaces
from numpy.core.fromnumeric import shape
import rospy
import rospkg
from rl_dovs.srv import *
from std_srvs.srv import Empty
from typing import Any, Dict, Optional, Union

from collections import OrderedDict


class DiscreteDOVS(spaces.Discrete):
  def __init__(self, n: int, n_extra: int) -> None:
      self.goal_actions = []
      self.n_extra = n_extra
      super().__init__(n)


class StageGym(gym.GoalEnv):
  """
  Custom Environment that follows gym interface.
  """

  def __init__(self, dovs_shape, vector_shape, n_actions):
    super(StageGym, self).__init__()

    # # Size of the 1D-grid
    # self.grid_size = grid_size
    # # Initialize the agent at the right of the grid
    # self.agent_pos = grid_size - 1

    # Define action and observation space
    # They must be gym.spaces objects
    # Example when using discrete actions, we have two: left and right
    # self.action_space = DiscreteDOVS(n_actions, 3)
    self.action_space = spaces.Discrete(n_actions)
    # The observation will be the coordinate of the agent
    # this can be described both by Discrete and Box space
    # self.observation_space = spaces.Box(low=-float('inf'), high=float('inf'), shape=dovs_shape, dtype=np.float32)
    self.observation_space = spaces.Dict(
        spaces={
            "observation": spaces.Dict(
              spaces={
                "vec": spaces.Box(low=-float('inf'), high=float('inf'), shape=vector_shape, dtype=np.float32),
                "img": spaces.Box(low=0, high=1, shape=dovs_shape, dtype=np.float32)
              }
            ),
            "achieved_goal": spaces.Box(
              low=-float('inf'), high=float('inf'), shape=[4], dtype=np.float32
            ),
            "desired_goal": spaces.Box(
              low=-float('inf'), high=float('inf'), shape=[4], dtype=np.float32
            )            
        }
    )
    self.dovs_shape = dovs_shape
    self.vector_shape = vector_shape
    self.n_actions = n_actions
    rospy.init_node('StageGym')
    print("ROS node init, waiting for service to be advertised")
    rospy.wait_for_service('/gym_step')
    print("Service is advertised")

  def reset(self):
    """
    Important: the observation must be a numpy array
    :return: (np.array) 
    """
    print("Waiting for service gym_rest")
    rospy.wait_for_service('/gym_reset')
    try:
        print("Calling service")
        gym_reset = rospy.ServiceProxy('/gym_reset', DQN_gym_reset)
        resp = gym_reset(False)
        # state = {"vec": resp.state[-self.vector_shape[0]:], "img": np.array(resp.state[:-self.vector_shape[0]]).reshape(self.dovs_shape)}
        # self.goal_actions = resp.goal_actions
        # self.action_space.goal_actions = resp.goal_actions
        desired_goal_data = resp.reward_data.copy()
        desired_goal_data[:-1] = 0
        state = OrderedDict(
          [
            ("observation", OrderedDict(
              [
                ("vec", resp.state[-self.vector_shape[0]:]),
                ("img", np.array(resp.state[:-self.vector_shape[0]]).reshape(self.dovs_shape))
              ]
            )),
            ("achieved_goal", resp.reward_data), 
            ("desired_goal", desired_goal_data)
          ]
        )  
        return state
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
        resp = gym_step(action)
        state = {"vec": resp.state[-self.vector_shape[0]:], "img": np.array(resp.state[:-self.vector_shape[0]]).reshape(self.dovs_shape)}
        desired_goal_data = resp.reward_data.copy()
        desired_goal_data[:-1] = 0
        state = OrderedDict(
          [
            ("observation", OrderedDict(
              [
                ("vec", resp.state[-self.vector_shape[0]:]),
                ("img", np.array(resp.state[:-self.vector_shape[0]]).reshape(self.dovs_shape))
              ]
            )),
            ("achieved_goal", resp.reward_data), 
            ("desired_goal", desired_goal_data)
          ]
        )        
        # self.goal_actions = resp.goal_actions
        # self.action_space.goal_actions = resp.goal_actions
        # print("DONE RECEIVED: ", resp.done)
        return state, resp.reward, resp.done, {"goal_actions": resp.goal_actions}
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
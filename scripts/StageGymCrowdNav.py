from time import sleep
from gym.spaces import space
from keyring import set_keyring
import numpy as np
import gym
from gym import spaces
from numpy.core.fromnumeric import shape
import rospy
import rospkg
from rl_dovs.srv import *
from std_srvs.srv import Empty
from typing import Any, Dict, Optional, Tuple, Union
import os
from crowd_sim.envs.utils.state import ObservableState
from crowd_sim.envs.utils.utils import point_to_segment_dist
from crowd_sim.envs.utils.action import ActionRot, ActionVW, ActionXY

class InnerRobot:
  def __init__(self):
      pass
  
  def update_inner_robot(self, px, py, gx, gy, vx, vy, theta, radius=None, v_pref=None):
    self.px = px
    self.py = py
    self.gx = gx
    self.gy = gy
    self.vx = vx
    self.vy = vy
    self.theta = theta
    self.radius = radius
    self.v_pref = v_pref

class StageGymCrowdNav(gym.Env):
  """
  Custom Environment that follows gym interface.
  """

  def __init__(self):
    # self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
    # self.observation_space = self.encoder.observation_space
    self.time_step = 0.2
    super(StageGymCrowdNav, self).__init__()
    print("ROS node init, waiting for service to be advertised")
    rospy.wait_for_service('/gym_step')
    print("Service is advertised")
    self.robot_state = InnerRobot()

  def set_robot(self, robot):
    self.robot = robot

  def reset(self, phase = None):
    """
    Important: the observation must be a numpy array
    :return: (np.array) 
    """
    if self.robot is None:
        raise AttributeError('robot has to be set!')
    rospy.wait_for_service('/gym_reset')
    try:
        gym_reset = rospy.ServiceProxy('/gym_reset', DQN_gym_reset_crowdnav)
        resp = gym_reset(False)  
        self.robot.set(resp.robot_state[0], resp.robot_state[1], resp.robot_state[2], resp.robot_state[3], resp.robot_state[4], 
                        resp.robot_state[5], resp.robot_state[6], resp.robot_state[7], resp.robot_state[8])     
        self.robot_state.update_inner_robot(resp.robot_state[0], resp.robot_state[1], resp.robot_state[2], resp.robot_state[3], resp.robot_state[4], 
                        resp.robot_state[5], resp.robot_state[6], resp.robot_state[7], resp.robot_state[8])     
        obs = [ObservableState(x.array[0], x.array[1], x.array[2], x.array[3], x.array[4]) for x in resp.observable_state]
        # Trick to handle few observations in crowdnav code
        while len(obs) < 15:
            obs.append(ObservableState(30,30,0,0,0))
        self.previous_obs = obs[:15]
        return obs[:15]
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

  def step(self, action):
    rospy.wait_for_service('/gym_step')
    try:
        gym_step = rospy.ServiceProxy('/gym_step', DQN_gym_crowdnav)
        # if isinstance(action, Tuple):
        #     action = action[0]
        # print(action)
        if action == 0:
            action=ActionRot(0,0)
        if isinstance(action[0], ActionRot):
            action = action[0]
            action_send = np.zeros(3)
            action_send[2] = action.r/0.2
            # action_send[0] = np.cos(self.robot_state.theta) * action.v * 0.2
            action_send[0] = action.v
            # action_send[1] = np.sin(self.robot_state.theta) * action.v * 0.2
            action_send[1] = 0
            action = action_send
        elif isinstance(action, ActionRot):
            action_send = np.zeros(3)
            action_send[2] = action.r/0.2
            # action_send[0] = np.cos(self.robot_state.theta) * action.v * 0.2
            action_send[0] = action.v
            # action_send[1] = np.sin(self.robot_state.theta) * action.v * 0.2
            action_send[1] = 0
            action = action_send
        elif isinstance(action, ActionVW):
            action = np.array([action[0], 0., action[1]])
        elif isinstance(action[0], ActionVW):
            action = np.array([action[0][0], 0., action[0][1]])
        elif isinstance(action[0], ActionXY):
            action = np.array([action[0][0], action[0][1], 0.])
        else:
            # print("ACTION: ", action)
            action = np.array([action[0], action[1], 0.])  # no rotation
        resp = gym_step(action[0], action[1], action[2])
        self.robot.set(resp.robot_state[0], resp.robot_state[1], resp.robot_state[2], resp.robot_state[3], resp.robot_state[4], 
                        resp.robot_state[5], resp.robot_state[6], resp.robot_state[7], resp.robot_state[8])     
        self.robot_state.update_inner_robot(resp.robot_state[0], resp.robot_state[1], resp.robot_state[2], resp.robot_state[3], resp.robot_state[4], 
                        resp.robot_state[5], resp.robot_state[6], resp.robot_state[7], resp.robot_state[8])
        obs = [ObservableState(x.array[0], x.array[1], x.array[2], x.array[3], x.array[4]) for x in resp.observable_state]
        # Trick to handle few observations in crowdnav code
        while len(obs) < 15:
            obs.append(ObservableState(30,30,0,0,0))
        self.previous_obs = obs[:15]
        return obs[:15], resp.reward, resp.done, {}
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
  
  def onestep_lookahead(self, action):
    # collision detection
    dmin = float('inf')
    collision = False
    for i, human in enumerate(self.previous_obs):
        px = human.px - self.robot.px
        py = human.py - self.robot.py
        if self.robot.kinematics == 'holonomic':
            vx = human.vx - action.vx
            vy = human.vy - action.vy
        else:
            vx = human.vx - action.v * np.cos(action.r + self.robot.theta)
            vy = human.vy - action.v * np.sin(action.r + self.robot.theta)
        ex = px + vx * self.time_step
        ey = py + vy * self.time_step
        # closest distance between boundaries of two agents
        closest_dist = point_to_segment_dist(px, py, ex, ey, 0, 0) - human.radius - self.robot.radius
        if closest_dist < 0:
            collision = True
            # logging.debug("Collision: distance between robot and p{} is {:.2E}".format(i, closest_dist))
            break
        elif closest_dist < dmin:
            dmin = closest_dist
    # check if reaching the goal
    end_position = np.array(self.robot.compute_position(action, self.time_step))
    reaching_goal = np.linalg.norm(end_position - np.array(self.robot.get_goal_position())) < self.robot.radius

    if collision:
        reward = -0.25
        done = True
    elif reaching_goal:
        reward = 1
        done = True
    elif dmin < 0.2:
        # only penalize agent for getting too close if it's visible
        # adjust the reward based on FPS
        reward = (dmin - 0.2) * 0.5 * self.time_step
        done = False
    else:
        reward = 0
        done = False
    ob = [ObservableState(x.px + x.vx*self.time_step, x.py + x.vy*self.time_step, x.vx, x.vy, x.radius, x.theta) for x in self.previous_obs]
    return ob, reward, done, {}

  def render(self, mode='console'):
    print("RENDER ENV->This should never be called")

  def close(self):
    pass
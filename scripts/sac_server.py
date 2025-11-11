from StageGymSAC import StageGym

from stable_baselines3 import SAC, PPO
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from sb3_contrib import RecurrentPPO

import numpy as np
import torch.nn as nn
import torch as th
from gymnasium import spaces
from datetime import datetime

import rospy
from rl_dovs.srv import *
import argparse
import matplotlib.pyplot as plt
from stable_baselines3.common.env_util import make_vec_env

from typing import Any, Dict, List, Mapping, Optional, Tuple, Union

from stable_baselines3.common.vec_env import VecFrameStack, DummyVecEnv
from stable_baselines3.common.vec_env.stacked_observations import StackedObservations
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CheckpointCallback, StopTrainingOnMaxEpisodes, CallbackList, BaseCallback

class CustomFeaturesExtractor(BaseFeaturesExtractor):
    """
    :param observation_space: (gym.Space)
    :param features_dim: (int) Number of features extracted.
        This corresponds to the number of unit for the last layer.
    """
    def __init__(self, observation_space: spaces.Box, features_dim: int, dovs_shape: list, vector_size: int, n_stack: int):
        super().__init__(observation_space, features_dim)
        self.features = self.FeatureNetwork(dovs_shape=dovs_shape, vector_size=vector_size, n_stack=n_stack)
        self.lstm = nn.LSTM(320, 256, batch_first=True)
        self.last = nn.Sequential(nn.Linear(256, features_dim), nn.ReLU())
        self.n_stack = n_stack
        # self._network = nn.Sequential(
        #     self.FeatureNetwork(dovs_shape=dovs_shape, vector_size=vector_size, n_stack=n_stack), # Output size = 320
        #     # nn.Linear(320, 256),
        #     # nn.ReLU(),
        #     # nn.Linear(256, features_dim),
        #     # nn.ReLU()
        #     nn.Linear(320, features_dim),
        #     nn.ReLU()
        # )

    def forward(self, observations: th.Tensor) -> th.Tensor:
        output = self.features(observations)
        output, _ = self.lstm(output)
        return self.last(output[:, -1, :])
    
    class FeatureNetwork(nn.Module):
        class ConvNetwork(nn.Module):
            def __init__(self, dovs_shape):
                super().__init__()
                self.network = nn.Sequential(
                        nn.Conv2d(1,8,5, padding="same", padding_mode="reflect"),
                        nn.ReLU(),
                        nn.Conv2d(8,8,4, padding="same", padding_mode="reflect"),
                        nn.ReLU(),
                        nn.Conv2d(8,4,3, padding="same", padding_mode="reflect"),
                        nn.ReLU(),
                        nn.Flatten(),
                        nn.Linear(dovs_shape[0]*dovs_shape[1]*4, 256),
                        nn.ReLU()
                    )

            def forward(self, input: th.Tensor) -> th.Tensor:
                return self.network(input)
        class LinearNetworkVector(nn.Module):
            def __init__(self):
                super().__init__()
                self.network = nn.Sequential(
                        nn.Linear(8, 64),
                        nn.ReLU()
                    )
            def forward(self, input: th.Tensor) -> th.Tensor:
                return self.network(input)
        def __init__(self, dovs_shape, vector_size, n_stack):
            super().__init__()
            self.vector_size = vector_size
            self.dovs_shape = dovs_shape
            self.conv_network = self.ConvNetwork(dovs_shape)
            self.linear_network_vector = self.LinearNetworkVector()
            self.n_stack = n_stack

        def forward(self, input: th.Tensor) -> th.Tensor:
            input = input.reshape((-1, self.n_stack, self.vector_size + self.dovs_shape[0]*self.dovs_shape[1]))
            vec = input[:, :, -self.vector_size:]
            img = input[:, :, :-self.vector_size]
            img = img.reshape((-1, 1, self.dovs_shape[0], self.dovs_shape[1]))
            output = self.conv_network(img)
            output = output.reshape((-1, self.n_stack, 256))
            # print("vec: ", vec)
            # plt.figure()
            # plt.imshow(img.cpu().squeeze(0).squeeze(0))
            # print(img)
            # plt.show()
            output_vec = self.linear_network_vector(vec)
            output = th.concat([output, output_vec], -1)
            return output

class ResumeStopTrainingOnMaxEpisodes(StopTrainingOnMaxEpisodes):
    def _on_step(self) -> bool:
        self.n_episodes = self.model._episode_num
        return super()._on_step()
    
class RecordSumReward(BaseCallback):
    def __init__(self, verbose: int = 0):
        super().__init__(verbose=verbose)
        self.reward_ep = 0.0

    def _on_step(self) -> bool:
        # Check that the `dones` local variable is defined
        assert "rewards" in self.locals, "`rewards` variable is not defined, please check your code next to `callback.on_step()`"
        self.reward_ep += np.sum(self.locals["rewards"]).item()
        if np.sum(self.locals["dones"]).item() > 0.0:
            self.logger.record("rewards", self.reward_ep)
            self.reward_ep = 0.0
        return True


rospy.init_node('rl_server')
writer_tensorboard = None

algorithm = 'dqn'
parser = argparse.ArgumentParser(description='Launch DQN of RUMOR')    
parser.add_argument('--no_training', action='store_true')
parser.add_argument('--single_agent', type=bool, default=False)
parser.add_argument('--restore_epsilon', type=bool, default=False)
parser.add_argument('--use_crowdnav_actions', type=bool, default=False)
parser.add_argument('--lidar_observation', type=bool, default=False)
parser.add_argument('--alpha', type=float, default=0.0003)
parser.add_argument('--read_weights', type=bool, default=False)
parser.add_argument('--weights_file', type=str, default="")
parser.add_argument('--n_episodes', type=int, default=200)
args = parser.parse_args()
no_training = args.no_training
single_agent = args.single_agent
read_weights = args.read_weights
weights_file = args.weights_file
# weights_file = "ppo_dovs_" + datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
n_episodes = args.n_episodes
env = DummyVecEnv([StageGym])
env = Monitor(env)
env = VecFrameStack(env, n_stack=8)
if no_training:
    #load number of episodes
    model = SAC.load("./logs/" + weights_file)
    #use weights file
    obs = env.reset()
    print("Ready to compute action")
    for _ in range(n_episodes):
        terminated = [False]
        while not terminated[0]:
            action, _states = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated = env.step(action)

else:
    policy_kwargs = dict(
        features_extractor_class=CustomFeaturesExtractor,
        features_extractor_kwargs=dict(features_dim=128, dovs_shape=[20, 40], vector_size=8, n_stack=8),
    )

    model = SAC("MlpPolicy", env, verbose=1, policy_kwargs=policy_kwargs, tensorboard_log="./logs/" + weights_file + "_tensorboard", train_freq=4)
    # model = SAC.load("./logs/sac_dovs_700000_steps", env=env)
    checkpoint_callback = CheckpointCallback(
    save_freq=100000,
    save_path="./logs/",
    name_prefix=weights_file,
    save_replay_buffer=False,
    save_vecnormalize=False,
    )
    reward_callback = RecordSumReward(1)
    callback_max_episodes = ResumeStopTrainingOnMaxEpisodes(max_episodes=n_episodes, verbose=1)
    callback = CallbackList([reward_callback, checkpoint_callback, callback_max_episodes])
    model.learn(total_timesteps=1000000, log_interval=10, progress_bar=True, callback=callback, reset_num_timesteps=False)
    model.save("./logs/" + weights_file + "_final")
# gym_reset = rospy.ServiceProxy('/gym_reset', DQN_gym_reset)
# resp = gym_reset(True)

# del model # remove to demonstrate saving and loading



gym_reset = rospy.ServiceProxy('/gym_reset', SAC_gym_reset)
resp = gym_reset(True)

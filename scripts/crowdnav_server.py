from ast import arg
import logging
import argparse
import configparser
import os
import torch
import numpy as np
import argparse
import gym
from crowd_nav.utils.explorer import Explorer
from crowd_nav.policy.policy_factory import policy_factory
from crowd_sim.envs.utils.robot import Robot
from crowd_sim.envs.policy.orca import ORCA
from crowd_nav.utils.trainer import VNRLTrainer, MPRLTrainer, TSRLTrainer, TD3RLTrainer
from crowd_nav.utils.memory import ReplayMemory
from crowd_nav.policy.reward_estimate import Reward_Estimator
import rospy
import rospkg
from StageGymCrowdNav import StageGymCrowdNav
from rl_dovs.srv import *
import importlib.util
from crowd_sim.envs.utils.info import *
import copy
import shutil
from tensorboardX import SummaryWriter

class ExplorerDOVS(Explorer):
    def __init__(self, env, robot, device, writer, use_noisy_net, memory=None, gamma=None, target_policy=None, intrinsic_reward=None):
        super().__init__(env, robot, device, writer, use_noisy_net, memory, gamma, target_policy, intrinsic_reward)
    
    def run_k_episodes(self, k, phase, update_memory=False, imitation_learning=False, episode=None, epoch=None,
                       print_failure=False):
        self.robot.policy.set_phase(phase)
        success_times = []
        collision_times = []
        timeout_times = []
        success = 0
        collision = 0
        timeout = 0
        discomfort = 0
        min_dist = []
        cumulative_rewards = []
        average_returns = []
        returns_list = []
        collision_cases = []
        timeout_cases = []
        discomfort_nums = []

        if self.robot.policy.name in ['model_predictive_rl', 'tree_search_rl']:
            if phase in ['test', 'val']: #and self.use_noisy_net:
                self.robot.policy.model[2].eval()
            else:
                self.robot.policy.model[2].train()

        for i in range(k):
            ob = self.env.reset(phase)
            done = False
            states = []
            actions = []
            rewards = []
            dones = []
            num_discoms =[]
            intrinsic_rewards = []
            while not done:
                num_discom = 0
                action, action_index = self.robot.act(ob)
                previous_ob = ob
                ob, reward, done, info = self.env.step(action)
                states.append(self.robot.policy.last_state)
                # if(self.robot.policy.name in ['TD3RL']):
                #     actions.append(torch.tensor((action.vx, action.vy)).float())
                # else:

                # for TD3rl, append the velocity and theta
                actions.append(action_index)
                # rewards.append(reward)
                # actually, final states of timeout cases is not terminal states
                if isinstance(info, Timeout):
                    dones.append(False)
                else:
                    dones.append(done)
                rewards.append(reward)
                if isinstance(info, Discomfort):
                    discomfort += 1
                    min_dist.append(info.min_dist)
                    num_discom = info.num
                num_discoms.append(num_discom)
            # add the terminal state
            states.append(self.robot.get_state(ob))

            if update_memory:
                # if isinstance(info, ReachGoal) or isinstance(info, Collision):
                    # only add positive(success) or negative(collision) experience in experience set   
                if self.intrinsic_reward_alg is not None:
                    embeddings = None
                    if self.intrinsic_reward_alg.name == "RE3":
                        embeddings = []
                        intrinsic_rewards = rewards
                        for i_intrinsic in range(len(rewards)):
                            if type(states[i_intrinsic]) is tuple:
                                embeddings.append(self.intrinsic_reward_alg.get_embeddings((states[i_intrinsic][0].unsqueeze(0), states[i_intrinsic][1].unsqueeze(0))).to(states[0][0].device))
                            else:
                                embeddings.append(self.intrinsic_reward_alg.get_embeddings(states[i_intrinsic].unsqueeze(0).to(states[0].device)))
                    else:
                        for i_intrinsic in range(len(rewards)):
                            intrinsic_rewards.append(rewards[i_intrinsic] + self.intrinsic_reward_alg.compute_intrinsic_reward(
                                tuple(torch.unsqueeze(substate, 0) for substate in states[i_intrinsic]),
                                tuple(torch.unsqueeze(substate, 0) for substate in states[i_intrinsic + 1]), 
                                actions[i_intrinsic]))                         
                    self.update_memory(states, actions, intrinsic_rewards, dones, imitation_learning, embeddings)
                else:
                    self.update_memory(states, actions, rewards, dones, imitation_learning)

        return self.statistics


class Arguments():
    def __init__(self, model_dir, il, env_config, gpu, policy, policy_config, parser_args) -> None:
        rospack = rospkg.RosPack()
        self.model_dir = rospack.get_path('rl_dovs') + '/scripts/' + model_dir
        self.output_dir = rospack.get_path('rl_dovs') + '/scripts/' + model_dir
        self.il = il
        self.env_config = env_config
        self.gpu = True
        self.policy = policy
        self.policy_config = policy_config
        self.debug = False
        self.overwrite = False
        self.config = self.output_dir + '/config.py' 
        self.resume = False
        self.no_training = parser_args.no_training
        self.n_episodes = parser_args.n_episodes
        if self.no_training:
            self.epsilon = 0.01
        else:
            self.epsilon = 1.0
        print("Config file ", self.policy_config)

def main():
    parser = argparse.ArgumentParser(description='Launch crowdnav server')    
    parser.add_argument('--no_training', action='store_true')
    parser.add_argument('--weights_file', type=str, default="")
    parser.add_argument('--n_episodes', type=int, default=10000)
    parser.add_argument('--policy', type=str)
    args = parser.parse_args()
    policy = args.policy
    print(args.no_training)
    if args.no_training: 
        weights_file = args.weights_file 
        
        args = Arguments("data/" + weights_file, False, "configs/env.config", True, policy, 'configs/policy.config', args)

        if args.model_dir is not None and args.policy != "orca" and args.policy != "vecmpc":
            env_config_file = os.path.join(args.model_dir, os.path.basename(args.env_config))
            policy_config_file = os.path.join(args.model_dir, os.path.basename(args.policy_config))
            if args.il:
                model_weights = os.path.join(args.model_dir, 'il_model.pth')
            else:
                if os.path.exists(os.path.join(args.model_dir, 'resumed_rl_model.pth')):
                    model_weights = os.path.join(args.model_dir, 'resumed_rl_model.pth')
                elif os.path.exists(os.path.join(args.model_dir, 'best_val.pth')):
                    model_weights = os.path.join(args.model_dir, 'best_val.pth')
                else:
                    exist = os.path.exists(os.path.join(args.output_dir, 'rl_model_0.pth'))
                    if exist:
                        i_name = 1
                        while exist:
                            exist = os.path.exists(os.path.join(args.output_dir, 'rl_model_' +  str(i_name) + '.pth'))
                            i_name = i_name+1
                        model_weights = os.path.join(args.output_dir, 'rl_model_' +  str(i_name-2) + '.pth')
                    else:
                        print("Error: Model weights file does not exist")
                        exit(1)
        else:
            env_config_file = args.env_config
            policy_config_file = args.env_config

        # configure logging and device
        logging.basicConfig(level=logging.INFO, format='%(asctime)s, %(levelname)s: %(message)s',
                            datefmt="%Y-%m-%d %H:%M:%S")
        # device = torch.device("cuda:0" if torch.cuda.is_available() and args.gpu else "cpu")
        device="cpu"
        logging.info('Using device: %s', device)

        # configure policy
        config_file = os.path.join(args.model_dir, 'config.py')
        spec = importlib.util.spec_from_file_location('config', config_file)
        config = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(config)

        policy_config = config.PolicyConfig(args.debug)
        policy = policy_factory[policy_config.name]()
        train_config = config.TrainConfig().train
        env_config = config.EnvConfig(args.debug)

        reward_estimator = Reward_Estimator()
        reward_estimator.configure(env_config)
        policy.reward_estimator = reward_estimator
        policy.exploration_alg = "epsilon_greedy"
        if policy_config.name == "model_predictive_rl":
            if args.planning_depth is not None:
                policy_config.model_predictive_rl.do_action_clip = True
                policy_config.model_predictive_rl.planning_depth = args.planning_depth
            if args.planning_width is not None:
                policy_config.model_predictive_rl.do_action_clip = True
                policy_config.model_predictive_rl.planning_width = args.planning_width
            if args.sparse_search:
                policy_config.model_predictive_rl.sparse_search = True

        policy.configure(policy_config, device)
        if policy.trainable:
            if args.model_dir is None:
                print('Trainable policy must be specified with a model weights directory')
                exit()
            policy.load_model(model_weights)
        # configure environment

        robot = Robot(env_config, 'robot')
        robot.time_step = 0.2
        robot.set_policy(policy)
        env = StageGymCrowdNav()
        env.set_robot(robot)

        policy.set_phase('test')
        policy.set_device(device)
        if not isinstance(robot.policy, ORCA):
            epsilon = args.epsilon
            robot.policy.set_epsilon(epsilon)
        # set safety space for ORCA in non-cooperative simulation
        if isinstance(robot.policy, ORCA):
            if robot.visible:
                robot.policy.safety_space = 0
            else:
                # because invisible case breaks the reciprocal assumption
                # adding some safety space improves ORCA performance. Tune this value based on your need.
                robot.policy.safety_space = 0
            logging.info('ORCA agent buffer: %f', robot.policy.safety_space)

        policy.set_env(env)
        n_episodes = args.n_episodes
        if robot.policy.name in ['tree_search_rl']:
            policy.model[2].eval()
        for i in range(n_episodes):
            ob = env.reset()
            done = False
            while not done:
                action = robot.act(ob)
                ob, _, done, info = env.step(action)
        gym_reset = rospy.ServiceProxy('/gym_reset', DQN_gym_reset_crowdnav)
        resp = gym_reset(True)
    else:
        # configure paths
        make_new_dir = True
        weights_file = args.weights_file
        args = Arguments("data/" + weights_file, False, "configs/env.config", True, policy, 'configs/policy.config', args)

        # if os.path.exists(args.output_dir):
        #     if args.overwrite:
        #         shutil.rmtree(args.output_dir)
        #     else:
        #         shutil.rmtree(args.output_dir)
        # if make_new_dir:
        #     os.makedirs(args.output_dir)
        #     shutil.copy(args.config, os.path.join(args.output_dir, 'config.py'))

        log_file = os.path.join(args.output_dir, 'output.log')
        in_weight_file = os.path.join(args.output_dir, 'in_model.pth')
        il_weight_file = os.path.join(args.output_dir, 'il_model.pth')
        rl_weight_file = os.path.join(args.output_dir, 'rl_model.pth')

        spec = importlib.util.spec_from_file_location('config', args.config)
        if spec is None:
            print('Config file not found in ', args.config)
            exit()
        config = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(config)

        # configure logging
        mode = 'a' if args.resume else 'w'
        file_handler = logging.FileHandler(log_file, mode=mode)
        stdout_handler = logging.StreamHandler(sys.stdout)
        level = logging.INFO if not args.debug else logging.DEBUG
        logging.basicConfig(level=level, handlers=[stdout_handler, file_handler],
                            format='%(asctime)s, %(levelname)s: %(message)s', datefmt="%Y-%m-%d %H:%M:%S")
        device = "cpu"
        # device = torch.device("cuda:0" if torch.cuda.is_available() and args.gpu else "cpu")
        logging.info('Using device: %s', device)
        writer = SummaryWriter(log_dir=args.output_dir)

        # configure policy
        policy_config = config.PolicyConfig()
        policy = policy_factory[policy_config.name]()
        if not policy.trainable:
            print('Policy has to be trainable')
            exit()
        train_config = config.TrainConfig(args.debug)

        policy.set_exploration_alg("epsilon_greedy")
        policy.configure(policy_config, device)
        policy.set_device(device)

        # configure environment
        env_config = config.EnvConfig(args.debug)
        env = StageGymCrowdNav()
        robot = Robot(env_config, 'robot')
        robot.time_step = env.time_step
        env.set_robot(robot)

        reward_estimator = Reward_Estimator()
        reward_estimator.configure(env_config)
        policy.reward_estimator = reward_estimator 

        # read training parameters
        # train_config = config.TrainConfig(args.debug)
        rl_learning_rate = train_config.train.rl_learning_rate
        train_batches = train_config.train.train_batches
        # train_episodes = train_config.train.train_episodes 
        train_episodes = args.n_episodes
        sample_episodes = train_config.train.sample_episodes
        target_update_interval = train_config.train.target_update_interval
        evaluation_interval = train_config.train.evaluation_interval
        capacity = train_config.train.capacity
        epsilon_start = train_config.train.epsilon_start
        epsilon_end = train_config.train.epsilon_end
        epsilon_decay = train_config.train.epsilon_decay
        checkpoint_interval = train_config.train.checkpoint_interval

        # configure trainer and explorer
        memory = ReplayMemory(capacity)
        model = policy.get_model()
        batch_size = train_config.trainer.batch_size
        optimizer = train_config.trainer.optimizer
        print(policy_config.name)
        intrinsic_reward_alg = None
        env.human_num = 15
        if policy_config.name == 'model_predictive_rl': 
            trainer = MPRLTrainer(model, policy.state_predictor, memory, device, policy, writer, batch_size, optimizer, env.human_num,
                                reduce_sp_update_frequency=train_config.train.reduce_sp_update_frequency,
                                freeze_state_predictor=train_config.train.freeze_state_predictor,
                                detach_state_predictor=train_config.train.detach_state_predictor,
                                share_graph_model=policy_config.model_predictive_rl.share_graph_model,
                                intrinsic_reward=intrinsic_reward_alg)
        elif policy_config.name == 'tree_search_rl':
            trainer = TSRLTrainer(model, policy.state_predictor, memory, device, policy, writer, batch_size, optimizer, env.human_num,
                                reduce_sp_update_frequency=train_config.train.reduce_sp_update_frequency,
                                freeze_state_predictor=train_config.train.freeze_state_predictor,
                                detach_state_predictor=train_config.train.detach_state_predictor,
                                share_graph_model=policy_config.model_predictive_rl.share_graph_model,
                                intrinsic_reward = intrinsic_reward_alg)

        elif policy_config.name == 'gat_predictive_rl':
            trainer = MPRLTrainer(model, policy.state_predictor, memory, device, policy, writer, batch_size, optimizer, env.human_num,
                                reduce_sp_update_frequency=train_config.train.reduce_sp_update_frequency,
                                freeze_state_predictor=train_config.train.freeze_state_predictor,
                                detach_state_predictor=train_config.train.detach_state_predictor,
                                share_graph_model=policy_config.model_predictive_rl.share_graph_model)
        elif policy_config.name == 'td3_rl':
            # # for continous action
            action_dim = 2
            max_action = np.array([0.7, 3.14])
            min_action = np.array([0.0, -3.14])
            policy.set_action(action_dim, max_action, min_action)
            trainer = TD3RLTrainer(policy.actor, policy.critic, policy.state_predictor, memory, device, policy, writer,
                                batch_size, optimizer, env.human_num, reduce_sp_update_frequency=train_config.train.reduce_sp_update_frequency,
                                freeze_state_predictor=train_config.train.freeze_state_predictor,
                                detach_state_predictor=train_config.train.detach_state_predictor,
                                share_graph_model=policy_config.model_predictive_rl.share_graph_model)
        else:
            trainer = VNRLTrainer(model, memory, device, policy, batch_size, optimizer, writer, intrinsic_reward=intrinsic_reward_alg)
        explorer = ExplorerDOVS(env, robot, device, writer, policy_config.use_noisy_net, memory, policy.gamma, target_policy=policy,
                            intrinsic_reward = intrinsic_reward_alg)
        policy.save_model(in_weight_file)
        # imitation learning
        # if args.resume:
        #     if not os.path.exists(rl_weight_file):
        #         logging.error('RL weights does not exist')
        #     policy.load_state_dict(torch.load(rl_weight_file))
        #     model = policy.get_model()
        #     rl_weight_file = os.path.join(args.output_dir, 'resumed_rl_model.pth')
        #     logging.info('Load reinforcement learning trained weights. Resume training')
        # elif os.path.exists(il_weight_file):
        #     policy.load_state_dict(torch.load(rl_weight_file))
        #     model = policy.get_model()
        #     logging.info('Load imitation learning trained weights.')
        # else:
        #     il_episodes = train_config.imitation_learning.il_episodes
        #     il_policy = train_config.imitation_learning.il_policy
        #     il_epochs = train_config.imitation_learning.il_epochs
        #     il_learning_rate = train_config.imitation_learning.il_learning_rate
        #     trainer.set_learning_rate(il_learning_rate)
        #     if robot.visible:
        #         safety_space = 0
        #     else:
        #         safety_space = train_config.imitation_learning.safety_space
        #     il_policy = policy_factory[il_policy]()
        #     il_policy.set_common_parameters(policy_config)
        #     il_policy.multiagent_training = policy.multiagent_training
        #     il_policy.safety_space = safety_space
        #     robot.set_policy(il_policy)
        #     explorer.run_k_episodes(il_episodes, 'train', update_memory=True, imitation_learning=True)
        #     trainer.optimize_epoch(il_epochs)
        #     policy.save_model(il_weight_file)
        #     logging.info('Finish imitation learning. Weights saved.')
        #     logging.info('Experience set size: %d/%d', len(memory), memory.capacity)
        exist = os.path.exists(os.path.join(args.output_dir, 'rl_model_0.pth'))
        if exist:
            i_name = 1
            while exist:
                exist = os.path.exists(os.path.join(args.output_dir, 'rl_model_' +  str(i_name) + '.pth'))
                i_name = i_name+1
            policy.load_state_dict(torch.load(os.path.join(args.output_dir, 'rl_model_' +  str(i_name-2) + '.pth')))
            model = policy.get_model()
        trainer.update_target_model(model)
        # reinforcement learning
        policy.set_env(env)
        robot.set_policy(policy)
        robot.print_info()
        trainer.set_rl_learning_rate(rl_learning_rate)
        # fill the memory pool with some RL experience
        # if args.resume:
        #     robot.policy.set_epsilon(epsilon_end)
        #     explorer.run_k_episodes(100, 'train', update_memory=True, episode=0)
        #     logging.info('Experience set size: %d/%d', len(memory), memory.capacity)
        episode = 0
        best_val_reward = -1
        best_val_return = -1
        best_val_model = None
        # evaluate the model after imitation learning

        # if episode % evaluation_interval == 0:
        #     logging.info('Evaluate the model instantly after imitation learning on the validation cases')
        #     explorer.run_k_episodes(env.case_size['val'], 'val', episode=episode)
        #     explorer.log('val', episode // evaluation_interval)

        #     if args.test_after_every_eval:
        #         explorer.run_k_episodes(env.case_size['test'], 'test', episode=episode, print_failure=True)
        #         explorer.log('test', episode // evaluation_interval)

        episode = 0
        fw = open(args.output_dir + '/data.txt', 'w')
        print("%f %f %f %f %f" % (0,0,0,0,0), file=fw)
        while episode < train_episodes:
            print("EPISODE: ", episode)
            if args.resume:
                epsilon = epsilon_end
            else:
                if episode < epsilon_decay:
                    epsilon = epsilon_start + (epsilon_end - epsilon_start) / epsilon_decay * episode
                else:
                    epsilon = epsilon_end
            robot.policy.set_epsilon(epsilon)

            # sample k episodes into memory and optimize over the generated memory
            explorer.run_k_episodes(sample_episodes, 'train', update_memory=True, episode=episode)
            print("after episode")
            # explorer.log('train', episode)
            trainer.optimize_batch(train_batches, episode)
            episode += 1

            if episode % target_update_interval == 0:
                trainer.update_target_model(model)
            # evaluate the model
            # if episode % evaluation_interval == 0:
            #     _, _, _, reward, average_return, _, _ = explorer.run_k_episodes(env.case_size['val'], 'val', episode=episode)
            #     explorer.log('val', episode // evaluation_interval)

            #     if episode % checkpoint_interval == 0 and average_return > best_val_return:
            #         best_val_return = average_return
            #         best_val_model = copy.deepcopy(policy.get_state_dict())

            if episode != 0 and episode % checkpoint_interval == 0:
                current_checkpoint = episode // checkpoint_interval - 1
                save_every_checkpoint_rl_weight_file = rl_weight_file.split('.')[0] + '_' + str(current_checkpoint) + '.pth'
                policy.save_model(save_every_checkpoint_rl_weight_file)

        # # test with the best val model
        fw.close()
        gym_reset = rospy.ServiceProxy('/gym_reset', DQN_gym_reset_crowdnav)
        resp = gym_reset(True)

if __name__ == '__main__':
    rospy.init_node('rl_server')
    main()
#include <rl_dovs/active_agent.h>
#include <stage_ros_dovs/teleport.h>
#include <stage_ros_dovs/step_by_step.h>
#include <std_srvs/Empty.h>
#include "rl_dovs/DQN_restart_parameters.h"
#include <rl_dovs/DQN_gym.h>
#include <rl_dovs/DQN_gym_reset.h>
#include <rl_dovs/DQN_gym_reset_lidar.h>
#include <rl_dovs/DQN_gym_lidar.h>
#include <rl_dovs/DQN_gym_reset_crowdnav.h>
#include <rl_dovs/DQN_gym_crowdnav.h>
#include <rl_dovs/DQN_action_rules.h>
#include <rl_dovs/SAC_gym.h>
#include <rl_dovs/SAC_gym_reset.h>
#include <rl_dovs/FloatArray.h>


#include <random>

#include <std_msgs/UInt8.h>

class ActiveAgentDQN: public ActiveAgentClass {
private:
    ros::Subscriber ack_sub, collision_sub, reset_position_sub, odom_sub, laser_sub;
    ros::Publisher end_msg;
    ros::ServiceClient client_teleport, client_reset;
    ros::ServiceServer server_gym, server_gym_reset, server_act_rules;
    std::vector<ros::Publisher> amcl_pub, new_vel_pub, reset_position_pub, shutdown_agent_pub;

    bool visualize;
    bool crowdnav = false;
    bool rllib = false;
    ros::Duration sleep_rate;
    ros::Time before_stop;
    std::vector<float> lidar_measurements;
    geometry_msgs::Twist vel_measurement, last_vel_published;
    geometry_msgs::Pose pos_measurement;
    // int n_episodes_getGoal = 0;
    bool use_crowdnav_actions = false;
    std::vector<Velocidad> crowdnavVelocities;
    bool lidar_observation = false;

public:
    ros::ServiceClient client_step_by_step;
    bool isFinished();
    // bool waitingForACK();
    // void ackCallback(const std_msgs::String& s);
    // void sendEndMessage();
    // void robotNearbyColl(const nav_msgs::Odometry& msg);
    void collisionCall(const std_msgs::UInt8& msg);
    void positionChange(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void setFinish(bool& flag, const end_flag type) override;
    void teleportAgents(const int n_actives, const int n_pasives, const bool visualize, const bool finish_training);
    void newIteration(const bool visualize, const bool finish_training);
    bool handle_gym(rl_dovs::DQN_gym::Request  &req, rl_dovs::DQN_gym::Response &res);
    bool handle_reset(rl_dovs::DQN_gym_reset::Request& req, rl_dovs::DQN_gym_reset::Response& res);
    bool handle_gym_sac(rl_dovs::SAC_gym::Request  &req, rl_dovs::SAC_gym::Response &res);
    bool handle_reset_sac(rl_dovs::SAC_gym_reset::Request& req, rl_dovs::SAC_gym_reset::Response& res);
    void act(int action);
    void act(double a1, double a2);
    void getGoalState(std::vector<float> goalState, double reward);
    bool select_action_rules(rl_dovs::DQN_action_rules::Request& req, rl_dovs::DQN_action_rules::Response& res);
    void lidarCallback(const sensor_msgs::LaserScan& msg);
    void odomCallback(const nav_msgs::Odometry& msg);
    void getStateLidar (std::vector<double>& robot_state);
    void observe_lidar(std::vector<double>& robot_state, double& reward, bool& done, bool from_reset = false);
    bool handle_reset_lidar(rl_dovs::DQN_gym_reset_lidar::Request& req, rl_dovs::DQN_gym_reset_lidar::Response& res);
    bool handle_gym_lidar(rl_dovs::DQN_gym_lidar::Request  &req, rl_dovs::DQN_gym_lidar::Response &res);
    void act_lidar(geometry_msgs::Twist& action);
    bool handle_reset_crowdnav(rl_dovs::DQN_gym_reset_crowdnav::Request& req, rl_dovs::DQN_gym_reset_crowdnav::Response& res);
    bool handle_gym_crowdnav(rl_dovs::DQN_gym_crowdnav::Request  &req, rl_dovs::DQN_gym_crowdnav::Response &res);
    void observe_crowdnav(std::vector<double>& robot_state, std::vector<rl_dovs::FloatArray>& observable_state, 
        double& reward, bool& done, bool from_reset = false);
    void getStateCrowdnav (std::vector<double>& robot_state);
    void changeTestingScenario(const int n_actives, const int n_pasives, const int episodes, const bool visualize);

    
    ActiveAgentDQN(int id, double v_x, double v_th, double x_goal, double y_goal, 
        double av, double aw, bool learning, bool deep_q_learning, bool graph, double time_step, int lookAhead, 
        int timeHorizon, bool accConst, int algorithm, bool video, bool debug, bool readqtable, 
        double epsilon, const string& fileQTable, bool manual, bool test, double alpha, double gamma,
        bool track_metric, string q_table_name, bool record_steps, bool no_training, int nActives, int nPasives, int n_episodes,
        double epsilon_discount, bool random_scenario, double original_x, double original_y, double original_theta, bool visualize,
        bool collaborative_scenario, bool lidar_state, bool crowdnav, bool curricular_obstacles, bool static_obstacles, bool random_n_obstacles, 
        bool cadrl, string crowdnav_policy, bool curricular_goal, bool global_scenario, bool rllib, string rllib_policy, bool orca_agents,
        bool use_crowdnav_actions, bool lidar_observation, bool sac) : 
            ActiveAgentClass (id, v_x, v_th, x_goal, y_goal, av, aw, learning, deep_q_learning, graph, time_step, lookAhead, 
            timeHorizon, accConst, algorithm, video, debug, readqtable, epsilon, ros::package::getPath("rl_dovs") + "/data/" + fileQTable, manual, test, 
            alpha, gamma, track_metric, q_table_name, record_steps, no_training, collaborative_scenario, global_scenario, orca_agents, 
            curricular_obstacles, curricular_goal, random_n_obstacles, static_obstacles, sac, nActives, nPasives) 
            {
                if (test){
                    episodes = -1;
                }
                if (test){
                    const auto p1 = std::chrono::system_clock::now();
                    if (lidar_state){
                        f_results_metrics_episode.open(ros::package::getPath("rl_dovs") + "/data/metrics/metrics_navrep_" + to_string(nActives)  + "-" + 
                                        to_string(nPasives) + "_" + to_string(id) + "_" + to_string(std::chrono::duration_cast<std::chrono::seconds>(
                                        p1.time_since_epoch()).count()) + ".txt", ios::app);
                    }
                    else if (cadrl){
                        f_results_metrics_episode.open(ros::package::getPath("rl_dovs") + "/data/metrics/metrics_cadrl_" + to_string(nActives)  + "-" + 
                                        to_string(nPasives) + "_" + to_string(id) + "_" + to_string(std::chrono::duration_cast<std::chrono::seconds>(
                                        p1.time_since_epoch()).count()) + ".txt", ios::app);
                    }
                    else if (crowdnav){
                        f_results_metrics_episode.open(ros::package::getPath("rl_dovs") + "/data/metrics/metrics_crowdnav_" + crowdnav_policy.substr(1) + "_" + to_string(nActives) + "-" + 
                                        to_string(nPasives) + "_" + to_string(id) + "_" + to_string(std::chrono::duration_cast<std::chrono::seconds>(
                                        p1.time_since_epoch()).count()) + ".txt", ios::app);
                    }
                    else if (rllib){
                        this->rllib = true;
                        f_results_metrics_episode.open(ros::package::getPath("rl_dovs") + "/data/metrics/metrics_rllib_" + rllib_policy + "_" + fileQTable + "_" + to_string(nActives) + "-" + 
                                        to_string(nPasives) + "_" + to_string(id) + "_" + to_string(std::chrono::duration_cast<std::chrono::seconds>(
                                        p1.time_since_epoch()).count()) + ".txt", ios::app);
                    }
                    else if (sac){
                        f_results_metrics_episode.open(ros::package::getPath("rl_dovs") + "/data/metrics/metrics_sac_" + fileQTable + "_" + to_string(nActives) + "-" + 
                                        to_string(nPasives) + "_" + to_string(id) + "_" + to_string(std::chrono::duration_cast<std::chrono::seconds>(
                                        p1.time_since_epoch()).count()) + ".txt", ios::app);
                    }
                    else {
                        f_results_metrics_episode.open(ros::package::getPath("rl_dovs") + "/data/metrics/metrics_dqn_" + to_string(nActives)  + "-" + 
                                        to_string(nPasives) + "_" + to_string(id) + "_" + to_string(std::chrono::duration_cast<std::chrono::seconds>(
                                        p1.time_since_epoch()).count()) + ".txt", ios::app);
                    }
                }
                else if (!no_training){
                    f_results_metrics_episode.open(ros::package::getPath("rl_dovs") + "/data/metrics/training_metrics.txt", ios::app);
                }
                this->lidar_observation = lidar_observation;
                sleep_rate = ros::Duration(time_step);
                this->visualize = visualize;
                this->multi = true;
                this->client_teleport = n.serviceClient<stage_ros_dovs::teleport>("/teleport_stage");
                this->client_reset = n.serviceClient<std_srvs::Empty>("/reset_positions");
                this->client_step_by_step = n.serviceClient<stage_ros_dovs::step_by_step>("/step_stage");
                this->finished = false;
                this->waiting_for_ack = false;
                // this->end_msg = n.advertise<std_msgs::String>("/shutdown", 1);
                if (id == 0){
                    // this->collision_sub = n.subscribe("near_robot_positions", 1, &ActiveAgentDQN::robotNearbyColl, this);
                    this->collision_sub = n.subscribe("collision", 1, &ActiveAgentDQN::collisionCall, this);
                }
                // this->position_sub = n.subscribe("amcl_pose", 1, &ActiveAgentDQN::positionChange, this);
                this->reset_position_sub = n.subscribe("reset_pos", 1, &ActiveAgentClass::resetPositionCallback, dynamic_cast<ActiveAgentClass*>(this));
                // this->ack_sub = n.subscribe("ack", 1, &ActiveAgentDQN::ackCallback, this);
                this->crowdnav = crowdnav;
                this->use_crowdnav_actions = use_crowdnav_actions;
                if (use_crowdnav_actions){ //54 actions
                    for (double v_c = 0.0; v_c <= 0.8; v_c+=0.14){
                        for (double w_c = -1.0; w_c <=1.0; w_c+=0.25){
                            crowdnavVelocities.push_back(Velocidad(v_c, w_c));
                        }
                    }
                }
                if (id == 0){
                    if (lidar_state){
                        server_gym = n.advertiseService("/gym_step", &ActiveAgentDQN::handle_gym_lidar, this);
                        server_gym_reset = n.advertiseService("/gym_reset", &ActiveAgentDQN::handle_reset_lidar, this);
                        this->odom_sub = n.subscribe("/robot_" + to_string(id) + "/odom", 1, &ActiveAgentDQN::odomCallback, this);
                        this->laser_sub = n.subscribe("/robot_" + to_string(id) + "/ranger_0/base_scan", 1, &ActiveAgentDQN::lidarCallback, this);
                        // position_sub.shutdown();
                    }
                    else if (crowdnav){
                        // this->position_sub.shutdown();
                        server_gym = n.advertiseService("/gym_step", &ActiveAgentDQN::handle_gym_crowdnav, this);
                        server_gym_reset = n.advertiseService("/gym_reset", &ActiveAgentDQN::handle_reset_crowdnav, this);
                        this->odom_sub = n.subscribe("/robot_" + to_string(id) + "/odom", 1, &ActiveAgentDQN::odomCallback, this);
                    }
                    else if (sac){
                        // this->position_sub.shutdown();
                        server_gym = n.advertiseService("/gym_step", &ActiveAgentDQN::handle_gym_sac, this);
                        server_gym_reset = n.advertiseService("/gym_reset", &ActiveAgentDQN::handle_reset_sac, this);
                        this->odom_sub = n.subscribe("/robot_" + to_string(id) + "/odom", 1, &ActiveAgentDQN::odomCallback, this);
                    }
                    else{
                        if(lidar_observation){
                           this->laser_sub = n.subscribe("/robot_" + to_string(id) + "/ranger_0/base_scan", 1, &ActiveAgentDQN::lidarCallback, this);
                        }
                        server_gym = n.advertiseService("/gym_step", &ActiveAgentDQN::handle_gym, this);
                        server_gym_reset = n.advertiseService("/gym_reset", &ActiveAgentDQN::handle_reset, this);
                        server_act_rules = n.advertiseService("/select_action_rules", &ActiveAgentDQN::select_action_rules, this);           
                        this->odom_sub = n.subscribe("/robot_" + to_string(id) + "/odom", 1, &ActiveAgentDQN::odomCallback, this);             
                    }
                    int idx_pub = 0;
                    while (idx_pub < nActives){
                        amcl_pub.push_back(n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/robot_" + to_string(idx_pub) + "/initialpose", 1));
                        reset_position_pub.push_back(n.advertise<geometry_msgs::PoseArray>("/robot_" + to_string(idx_pub) + "/reset_pos", 1));
                        idx_pub++;
                    }
                    while(idx_pub < nActives + nPasives){
                        new_vel_pub.push_back(n.advertise<geometry_msgs::Twist>("/robot_" + to_string(idx_pub) + "/velocity_change", 1));
                        idx_pub++;
                    }
                    for (int idx = 1; idx < (nActives + nPasives); idx++){
                        shutdown_agent_pub.push_back(n.advertise<std_msgs::String>("/robot_" + to_string(idx) + "/shutdown_agent", 1));
                    }
                }
                if (!random_scenario){
                    this->own_amcl_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/robot_" + to_string(id) + "/initialpose", 1);
                }
            }
};

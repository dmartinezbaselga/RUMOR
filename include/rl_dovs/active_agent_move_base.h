#include <rl_dovs/active_agent.h>
#include <stage_ros_dovs/teleport.h>
#include <stage_ros_dovs/step_by_step.h>
#include <std_srvs/Empty.h>
#include "rl_dovs/DQN_restart_parameters.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


#include <random>

#include <std_msgs/UInt8.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class ActiveAgentMoveBase: public ActiveAgentClass {
private:
    ros::Subscriber ack_sub, collision_sub, reset_position_sub, cmd_vel_sub, laser_sub;
    ros::Publisher end_msg;
    ros::ServiceClient client_teleport, client_reset;
    std::vector<ros::Publisher> amcl_pub, new_vel_pub, reset_position_pub, shutdown_agent_pub;
    bool test_agent_with_saved_scenarios;
    std::vector<float> lidar_measurements;


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
    void teleportAgents(const int n_actives, const int n_pasives, const bool visualize);
    void newIteration(const bool visualize);
    void changeTestingScenario(const int n_actives, const int n_pasives, const int episodes, const bool visualize);
    void computeChangesVelFinish(bool aborted);
    void recordLastVel(const geometry_msgs::Twist& msg);
    void lidarCallback(const sensor_msgs::LaserScan& msg);
    void checkCollision();

    
    ActiveAgentMoveBase(int id, double v_x, double v_th, double x_goal, double y_goal, 
        double av, double aw, bool learning, bool deep_q_learning, bool graph, double time_step, int lookAhead, 
        int timeHorizon, bool accConst, int algorithm, bool video, bool debug, bool readqtable, 
        double epsilon, const string& fileQTable, bool manual, bool test, double alpha, double gamma,
        bool track_metric, string q_table_name, bool record_steps, bool no_training, int nActives, int nPasives, int n_episodes,
        double epsilon_discount, bool random_scenario, double original_x, double original_y, double original_theta, bool test_agent_with_saved_scenarios,
        bool two_lasers, bool collaborative_scenario, bool global_scenario, bool orca_agents) : 
            ActiveAgentClass (id, v_x, v_th, x_goal, y_goal, av, aw, learning, deep_q_learning, graph, time_step, lookAhead, 
            timeHorizon, accConst, algorithm, video, debug, readqtable, epsilon, fileQTable, manual, test, 
            alpha, gamma, track_metric, q_table_name, record_steps, no_training, collaborative_scenario, global_scenario, orca_agents, false, 
            false, false, false, false, nActives, nPasives) 
            {
                if (test){
                    episodes = -1;
                    geometry_msgs::Twist msg;
                    msg.linear.x = 0.1;
                    msg.angular.z = 0.1;
                    // cout << "Chosen vel: " << v.v << ", " << v.w << endl;
                    pub.publish(msg);
                }
                this->laser_sub = n.subscribe("/robot_" + to_string(id) + "/ranger_0/base_scan", 1, &ActiveAgentMoveBase::lidarCallback, this);
                const auto p1 = std::chrono::system_clock::now();
                if (learning){
                    f_results_metrics_episode.open(ros::package::getPath("rl_dovs") + "/data/metrics/metrics_learning_" + to_string(nActives)  + "-" + 
                                    to_string(nPasives) + "_" + to_string(id)+ "_" + to_string(std::chrono::duration_cast<std::chrono::seconds>(
                                        p1.time_since_epoch()).count())  + ".txt", ios::app);
                }
                else if (deep_q_learning){
                    f_results_metrics_episode.open(ros::package::getPath("rl_dovs") + "/data/metrics/metrics_deep_q_learning_" + to_string(nActives)  + "-" + 
                                    to_string(nPasives) + "_" + to_string(id)+ "_" + to_string(std::chrono::duration_cast<std::chrono::seconds>(
                                        p1.time_since_epoch()).count()) + ".txt", ios::app);
                }
                else if (two_lasers){
                    f_results_metrics_episode.open(ros::package::getPath("rl_dovs") + "/data/metrics/metrics_two_lasers_" + to_string(nActives)  + "-" + 
                                    to_string(nPasives) + "_" + to_string(id) + "_" + to_string(std::chrono::duration_cast<std::chrono::seconds>(
                                        p1.time_since_epoch()).count()) + ".txt", ios::app);
                }
                else {
                    f_results_metrics_episode.open(ros::package::getPath("rl_dovs") + "/data/metrics/metrics_move_base_" + to_string(nActives)  + "-" + 
                                    to_string(nPasives) + "_" + to_string(id) + "_" + to_string(std::chrono::duration_cast<std::chrono::seconds>(
                                        p1.time_since_epoch()).count()) + ".txt", ios::app);
                }
                this->test_agent_with_saved_scenarios = test_agent_with_saved_scenarios;
                this->multi = true;
                this->client_teleport = n.serviceClient<stage_ros_dovs::teleport>("/teleport_stage");
                this->client_reset = n.serviceClient<std_srvs::Empty>("/reset_positions");
                this->client_step_by_step = n.serviceClient<stage_ros_dovs::step_by_step>("/step_stage");
                this->finished = false;
                if (test_agent_with_saved_scenarios){
                    this->finished = true;
                }
                this->waiting_for_ack = false;
                if (global_scenario){
                    this->goal_sub = n.subscribe("dovs_goal", 1, &ActiveAgentClass::rvizGoalCallback, dynamic_cast<ActiveAgentClass*>(this));            
                    this->index_goal = -1;
                    this->finished = true;
                    this->new_iteration_global = false;
                    geometry_msgs::PoseStamped global_goal;
                    global_goal.pose.position.x = x_goal;
                    global_goal.pose.position.y = y_goal;
                    global_planner_goal_pub.publish(global_goal);
                    this->goal.x = x_goal;
                    this->goal.y = y_goal;
                }
                else{
                    this->goal.x = x_goal;
                    this->goal.y = y_goal;
                    this->robot->AddGoal(x_goal, y_goal);
                }
                // this->end_msg = n.advertise<std_msgs::String>("/shutdown", 1);
                if (id == 0){ 
                    // this->collision_sub = n.subscribe("near_robot_positions", 1, &ActiveAgentMoveBase::robotNearbyColl, this);
                    this->collision_sub = n.subscribe("collision", 1, &ActiveAgentMoveBase::collisionCall, this);
                }
                this->position_sub = n.subscribe("amcl_pose", 1, &ActiveAgentMoveBase::positionChange, this);
                this->cmd_vel_sub = n.subscribe("robot_0/cmd_vel", 1, &ActiveAgentMoveBase::recordLastVel, this);
                this->reset_position_sub = n.subscribe("reset_pos", 1, &ActiveAgentClass::resetPositionCallback, dynamic_cast<ActiveAgentClass*>(this));
                // this->ack_sub = n.subscribe("ack", 1, &ActiveAgentMoveBase::ackCallback, this);
                this->nActives = nActives;
                this->nPasives = nPasives;
                this->n_episodes = n_episodes;
                this->epsilon_discount = epsilon_discount;
                this->random_scenario = random_scenario;
                this->original_x = original_x;
                this->original_y = original_y;
                this->original_theta = original_theta;
                if (id == 0){
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

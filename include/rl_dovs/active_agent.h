#include "ros/ros.h"
#include <numeric>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <obstacle_detector/Obstacles.h>
#include <rl_dovs/CollaborativeInfo.h>
#include "std_msgs/String.h"
#include <cmath>
#include <random>
#include <rl_dovs/dovts.h>
#include <rl_dovs/graficas.h>
#include <rl_dovs/config.h>
#include <rl_dovs/PassiveAgent.h>
#include <rl_dovs/ActiveAgent.h>
#include <rl_dovs/utilidades.h>
#include <rl_dovs/State.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include "rl_dovs/DQN_action.h"
#include "rl_dovs/SAC_action.h"
#include "rl_dovs/DQN_store.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "rl_dovs/request_goal.h"
#include <std_srvs/Empty.h>



#include <iostream>
using namespace std;

struct ObstacleInformation{
    int obs_id; 
    double x;
    double y; 
    double theta; 
    double linear_vel;
    double angular_vel;
    double radius;
};

class ActiveAgentClass {
    protected:
        bool too_many_iterations = false;
        std::vector<std::unique_ptr<Agent>> agents_before;
        std::vector<Velocidad> possibleVelocities;
        bool collaborative_scenario = true;  
        std::vector<ObstacleInformation> agents_collaborative;  
        ros::Publisher pub_collaborative_position;
        ros::Subscriber sub_collaborative_position;
        int nActives, nPasives;
        tf::TransformListener listener;
        bool multi = false;
        double alpha = 0, gamma = 0;
        // double v_x = 0, v_th = 0, av = 0, aw = 0, x = 0, y = 0, theta = 0, radio = 0, time_step = 0;
        double time_step = 0;
        std::vector<std::unique_ptr<Agent>> agents;
        bool learning = false, deep_q_learning = false, graph = false, accConst = false, video = false, debug = false;
        ros::NodeHandle n;
        ros::Publisher pub;
        ros::Publisher global_planner_goal_pub, own_amcl_pub, new_episode_pub;
        ros::Subscriber position_sub;
        ros::Subscriber near_robots;
        ros::Subscriber collision_sub;
        ActiveAgent* robot;
        Tpf goal;
        int id = 0, lookAhead = 0, timeHorizon = 0, algorithm = 0, iteracion = 0;
        char* library;

        // Learning variables
        // Variable initialization
        // Number of possible actions
        static const int numActions = 8;

        // Number of episodes
        double epsilon = 0;
        bool readQTable = false;

        // State frequency
        std::map<State, int> stateFrequencies;
        std::map<State, int>::iterator stateFrequency;

        // Q-Table
        std::map<State, std::array<double, numActions>> qTable;
        std::map<State, std::array<double, numActions>>::iterator qValue;
        std::map<State, std::array<double, numActions>>::iterator qValueNew;

        // Actions
        std::vector<std::pair<int, Velocidad>> validActions;
        int numValidActions = 0;
        Velocidad action;
        Velocidad oldAction;
        int numAction = 0, zone = 0;
        int numChosenAction = 0, previous_chosen_action = 0;
        int numNewStates = 0;
        double max = 0;
        bool chosenDirToGoal = false;
        double min_dist = 0.15;
        std::pair<int, Velocidad> velEnd;

        // State and new state
        State  state, newState, previous_state;
        bool free_actions = false; //Free actions
        // Reward
        double reward = 0;
        double rewardModifier = 0;
        bool manual = false;
        // needed for rewards
        double distLastState = 0, distCurrentState = 0, thetaLastState = 0, thetaCurrentState = 0, distObsLastState = 0,
        distObsCurrentState = 0, vLastState = 0, vCurrentState = 0, wLastState = 0, wCurrentState = 0, lastClosestAgent = 0,
        currentClosestAgent = 0, thetaObsLastState = 0, thetaObsCurrentState = 0, relThetaLastState = 0, 
        relThetaCurrentState = 0, lastCollision = 0, currentCollision = 0, lastCollisionAgent = 0, currentCollisionAgent = 0,
        thetaObsOriginal = 0;
        //finish agent
        bool finished, unreachable = false, collision_end = false, success = false, too_far = true;
        bool test = false;
        bool coll_received = false;
        ofstream f_metrics, f_record_steps, f_results_metrics_episode;
        enum end_flag {success_f, unreachable_f, too_far_f, collision_f};
        string q_table_name;
        bool record_steps = false;
        Tsc posObs;
        double vObs = 0;
        ros::ServiceClient client_DQN_action, client_DQN_store, client_SAC_action;
        rl_dovs::DQN_action srv_DQN_action;
        rl_dovs::DQN_store srv_DQN_store;
        rl_dovs::SAC_action srv_SAC_action;
        bool no_training = false;
        ros::Subscriber finish_msg;
        double acum_reward = 0;

        std::vector<double> linear_velocities_episode, distance_obstacles_episode;
        bool global_scenario = false;
        bool orca_agents = false;
        bool sac = false;
        bool random_scenario = true, waiting_for_ack = false;
        double index_goal = 0, original_x, original_y, original_theta;
        ros::Subscriber goal_sub;
        bool new_iteration_global = true;
        ros::ServiceServer goal_server;
        ros::ServiceClient clean_map_client;
        std::uniform_real_distribution<double> distributionX;
        std::uniform_real_distribution<double> distributionY;
        std::uniform_real_distribution<double> distributionTita;
        std::uniform_real_distribution<double> distributionEpsilon;
        std::uniform_real_distribution<double> distributionExplore;
        std::uniform_real_distribution<double> distributionV;
        std::uniform_real_distribution<double> distributionW;
        std::uniform_real_distribution<double> distributionFixed;
        double dist_maxGoal = 1;
        double dis_min_goal_min = 0.3;
        int n_iterations_getGoal = 0;
        std::random_device rd{};
        std::mt19937 generator{rd()};
        const double xmin = -4; 
        const double xmax = 4;  
        const double ymin = -4; 
        const double ymax = 4;
        double vMax = 0.7, wMax = 3.14;
        int episodes = 0, n_episodes;
        double epsilon_discount;
        bool curricular_obstacles = false, static_obstacles = false, random_n_obstacles = false, curricular_goal = false;

    public:
        bool skip_iteration = false;
        ActiveAgentClass (int id, double v_x, double v_th, double x_goal, double y_goal, 
        double av, double aw, bool learning, bool deep_q_learning, bool graph, double time_step, int lookAhead, 
        int timeHorizon, bool accConst, int algorithm, bool video, bool debug, bool readqtable, 
        double epsilon, const string& fileQTable, bool manual, bool test, double alpha, double gamma,
        bool track_metric, string q_table_name, bool record_steps, bool no_training, bool collaborative, 
        bool global_scenario, bool orca_agents, bool curricular_obstacles, bool curricular_goal,
        bool random_n_obstacles, bool static_obstacles, bool sac, int nActives = -1, int nPasives = -1);
            
        double GetRealRadius();
        int GetId();
        Tsc GetLocalization();
        double GetV();
        double GetW();
        double getAcumReward();
        void incrementAcumReward(const double r);
        void resetAcumReward();
        bool isFinished();
        bool CollisionObs(Tpf ag1, Tpf ag2, double securityDist);

        void getVelocities (double& vx, double& vth);

        // virtual void positionChange(const nav_msgs::Odometry& msg);   
        void robotNearby(const nav_msgs::Odometry& msg);
        void robotNearbyColl(const nav_msgs::Odometry& msg);
        void observe(std::vector<float>& state, std::vector<int>& goal_actions, double& reward, bool& done, bool from_reset = false);

        
        void obstaclesCallback(const obstacle_detector::Obstacles& msg);
        
        void publishVelocity();

        virtual void setFinish(bool& flag, const end_flag type) = 0;

        void writeMetrics();   

        void writeTrackerEstimation();

        void writeQTable(std::map<State, std::array<double, 8>> qTable, const std::string& fileName);

        void writeLearningStats(const end_flag flag_out, const int iteracion, const double r = 0);

        void readQTableFile(std::map<State, std::array<double, 8>> &qTable, std::string fileName);

        Velocidad applyLearning(State state);

        std::vector<std::pair<int, Velocidad>> getActions();

        State getState(int zone, string& times_get_state);

        double getReward();
        
        void getStateDQN (std::vector<float>& state);
        bool storeTransitionDQN (const double reward, const bool set_done);

        void finishROS (const std_msgs::String& s);

        Tpf getCurrentGoal();
        Velocidad velFromAction(double a1, double a2);

        void getObstaclesCollaborative();
        void collaborativeCallback(const rl_dovs::CollaborativeInfo& msg);
        void collaborativePublish();
        void resetPositionCallback(const geometry_msgs::PoseArray& msg);
        void rvizGoalCallback(const geometry_msgs::PoseStamped& msg);
        bool startIterationGlobalPlanner();
        bool handle_request_current_goal(rl_dovs::request_goal::Request  &req, rl_dovs::request_goal::Response &res);

        void getPosition (double& x, double& y, double& theta, std::vector<Tpf>& agents, bool active = false);
        void getGoal (double& xg, double& yg, const double x, const double y, const int n_iterations, const int idx);
};
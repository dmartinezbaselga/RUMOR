#include <rl_dovs/active_agent.h>
#include <nav_msgs/GetPlan.h>

class ActiveAgentSingle: public ActiveAgentClass {
private:
    ros::ServiceClient make_plan_client;
    nav_msgs::Path current_path;
public:
    void fillPathRequest(nav_msgs::GetPlan::Request &request, float start_x, float start_y, float goal_x, float goal_y);
    void callPlanningService(ros::ServiceClient &serviceClient, nav_msgs::GetPlan &srv);
    void positionChange(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void setFinish(bool& flag, const end_flag type) override;
    void createVideo(int id_video);
    
    ActiveAgentSingle(int id, double v_x, double v_th, double x_goal, double y_goal, 
        double av, double aw, bool learning, bool deep_q_learning, bool graph, double time_step, int lookAhead, 
        int timeHorizon, bool accConst, int algorithm, bool video, bool debug, bool readqtable, 
        double epsilon, const string& fileQTable, bool manual, bool test, double alpha, double gamma,
        bool track_metric, string q_table_name, bool record_steps, bool no_training, bool collaborative_scenario, bool orca_agents, bool sac, int nActives,
        int nPasives) : 
            ActiveAgentClass (id, v_x, v_th, x_goal, y_goal, av, aw, learning, deep_q_learning, graph, time_step, lookAhead, 
            timeHorizon, accConst, algorithm, video, debug, readqtable, epsilon, fileQTable, manual, test, 
            alpha, gamma, track_metric, q_table_name, record_steps, no_training, collaborative_scenario, false, orca_agents, false,
            false, false, false, sac, nActives, nPasives) 
            {
                this->multi = false;
                this->finished = true;
                this->position_sub = n.subscribe("amcl_pose", 1, &ActiveAgentSingle::positionChange, this);
                this->goal_sub = n.subscribe("dovs_goal", 1, &ActiveAgentClass::rvizGoalCallback, dynamic_cast<ActiveAgentClass*>(this));            
                this->index_goal = -1;
                this->make_plan_client = n.serviceClient<nav_msgs::GetPlan>("move_base/make_plan",true);
            }
};
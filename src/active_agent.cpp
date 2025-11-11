#include <rl_dovs/active_agent.h>

double offset = 1e-3*0.25/2;

double NormalisePI(double d){
//Transforms the value into +/-pi range

    while (d > M_PI) d -= 2*M_PI;
    while (d < -M_PI) d += 2*M_PI;

    if (d>=-offset && d<=offset) return 0.0;
    else if (d>=M_PI-offset && d<=M_PI+offset) return M_PI;
    else if (d>=-(M_PI+offset) && d<=-(M_PI-offset)) return -M_PI;
    else if (d>=M_PI_2-offset && d<=M_PI_2+offset) return M_PI_2;
    else if (d>=-(M_PI_2+offset) && d<=-(M_PI_2-offset)) return -M_PI_2;

    return d;
}
 
ActiveAgentClass::ActiveAgentClass (int id, double v_x, double v_th, double x_goal, double y_goal, 
double av, double aw, bool learning, bool deep_q_learning, bool graph, double time_step, int lookAhead, 
int timeHorizon, bool accConst, int algorithm, bool video, bool debug, bool readqtable, 
double epsilon, const string& fileQTable, bool manual, bool test, double alpha, double gamma,
bool track_metric, string q_table_name, bool record_steps, bool no_training, bool collaborative, 
bool global_scenario, bool orca_agents, bool curricular_obstacles, bool curricular_goal,
bool random_n_obstacles, bool static_obstacles, bool sac, int nActives, int nPasives){
    ROS_INFO("Constructor active agent");
    finish_msg = n.subscribe("shutdown_agent", 1, &ActiveAgentClass::finishROS, this);
    this->nActives = nActives;
    this->nPasives = nPasives;
    this->orca_agents = orca_agents;
    // this->collaborative_scenario = collaborative;
    collaborative_scenario = false;
    if (collaborative_scenario){
        for (int n_agent = 0; n_agent < nActives + nPasives; n_agent++){
            agents_collaborative.push_back(ObstacleInformation());
            agents_collaborative[n_agent].obs_id = -1;
        }
    }
    this->id = id;
    this->robot = new ActiveAgent(id, 5, 5, 0, v_x, v_th, av, aw, 0.2, true);
    this->robot->SetVelocity(Velocidad(v_x, v_th));
    this->robot->SetAcceleration(av, aw);
    this->learning = learning;
    this->deep_q_learning = deep_q_learning;
    this->sac = sac;
    this->graph = graph;
    this->pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    //this->near_robots = n.subscribe("near_robot_positions", 1, &ActiveAgentClass::robotNearby, this);
    this->near_robots = n.subscribe("obstacles", 1, &ActiveAgentClass::obstaclesCallback, this);
    if (collaborative_scenario){
        this->sub_collaborative_position = n.subscribe("/collaborative_info", 100, &ActiveAgentClass::collaborativeCallback, this);
        this->pub_collaborative_position = n.advertise<rl_dovs::CollaborativeInfo>("/collaborative_info", 1);
    }
    this->new_episode_pub = n.advertise<std_msgs::Empty>("/new_episode", 1);
    this->time_step = time_step;
    ROS_INFO("Active agent constructor");
    ROS_INFO("Create bounds");
    robot->SetBoundsVS(time_step);
    ROS_INFO("Bounds created");
    robot->SetNameLog("log_robot_" + to_string(id) + ".txt");
    this->accConst = accConst;
    this->timeHorizon = timeHorizon;
    this->lookAhead = lookAhead;
    this->algorithm = algorithm;
    this->video = video;
    this->iteracion = 0;
    this->debug = debug;
    this->epsilon = epsilon;
    this->readQTable = readqtable;
    this->manual = manual;
    this->test = test;
    this->alpha = alpha;
    this->gamma = gamma;
    this->q_table_name = q_table_name;
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
    this->nActives = nActives;
    this->nPasives = nPasives;
    this->n_episodes = n_episodes;
    this->epsilon_discount = epsilon_discount;
    this->random_scenario = random_scenario;
    this->original_x = original_x;
    this->original_y = original_y;
    this->original_theta = original_theta;
    this->curricular_obstacles = curricular_obstacles;
    this->curricular_goal = curricular_goal;
    this->static_obstacles = static_obstacles;
    this->random_n_obstacles = random_n_obstacles;
    velEnd.first = 0;
    velEnd.second = Velocidad(0, 0);
    this->record_steps = record_steps;
    if (record_steps && id == 0){
        f_record_steps.open(ros::package::getPath("rl_dovs") + "/data/record_steps/output.txt", std::ios_base::app);
    }
    if (video){
        this->graph = false;
    }
    if(graph){
        Inicializar(video);
        NuevaVentana(this->GetId()+1);
    }
    // Load Q-Table from file if necessary
    if (readQTable) {
        readQTableFile(qTable, fileQTable);
    }
    rewardModifier = 0;
    std::uniform_real_distribution<double>::param_type parEpsilon(0, 1);
    distributionEpsilon.param(parEpsilon);
    // f_metrics.open(ros::package::getPath("rl_dovs") + "/data/metrics/metrics_" + to_string(id) + ".txt", ios::app);
    if(sac){
        this->client_SAC_action = n.serviceClient<rl_dovs::SAC_action>("compute_action_sac");
    }
    else if (deep_q_learning){
        this->client_DQN_action = n.serviceClient<rl_dovs::DQN_action>("compute_action_dqn");
        // this->client_DQN_store = n.serviceClient<rl_dovs::DQN_store>("/store_transition");
    }
    this->no_training = no_training;
    // tf::StampedTransform transform;
    // try{
    //   listener.lookupTransform("/map", "robot_0/base_link", ros::Time(0), transform);
    // }
    // catch (tf::TransformException ex){
    //   ROS_ERROR("%s",ex.what());
    //   ros::Duration(1.0).sleep();
    // }
    this->global_scenario = global_scenario;
    if (global_scenario){
        this->global_planner_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/rviz_goal", 1);
        this->clean_map_client = n.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
    }
    this->goal_server = n.advertiseService("get_current_goal", &ActiveAgentClass::handle_request_current_goal, this);
    ROS_INFO("Constructor active agent check");
}

bool ActiveAgentClass::CollisionObs(Tpf ag1, Tpf ag2, double securityDist){

    double distancia = Distancia(ag1.x - ag2.x, ag1.y - ag2.y);

    if (distancia - securityDist < 0.0)
        return true;
    else
        return false;

}


void ActiveAgentClass::getVelocities (double& vx, double& vth){
    if (orca_agents){
        //Velocity
        std::uniform_real_distribution<double>::param_type parV(0.1, 0.5);
        distributionV.param(parV);
        vx = distributionV(generator);

        //Angular velocity
        std::uniform_real_distribution<double>::param_type parW(0.1, 0.5);
        distributionW.param(parW);
        vth = distributionW(generator);     
    }
    else{
        //Velocity
        std::uniform_real_distribution<double>::param_type parV(vMax*0.20, vMax*1.20);
        distributionV.param(parV);
        vx = distributionV(generator);

        //Angular velocity
        std::uniform_real_distribution<double>::param_type parW(-wMax*0.5, wMax*0.5);
        distributionW.param(parW);
        vth = distributionW(generator);
    }
    if (rand()%100 >= 95 || this->static_obstacles){
        vx = 0.0;
        vth = 0.0;
    }
}

bool ActiveAgentClass::handle_request_current_goal(rl_dovs::request_goal::Request  &req, rl_dovs::request_goal::Response &res){
    res.x = this->goal.x;
    res.y = this->goal.y;
    return true;
}

bool ActiveAgentClass::isFinished(){
    return this->finished;
}

Tpf ActiveAgentClass::getCurrentGoal(){
    return this->goal;
}

double ActiveAgentClass::GetRealRadius(){
    return this->robot->GetRealRadius();
}
int ActiveAgentClass::GetId(){
    return this->id;
}
Tsc ActiveAgentClass::GetLocalization(){
    return this->robot->GetLocalization();
}
double ActiveAgentClass::GetV(){
    return this->robot->GetV();
}
double ActiveAgentClass::GetW(){
    return this->robot->GetW();
}    

double ActiveAgentClass::getAcumReward(){
    return acum_reward;
}

void ActiveAgentClass::incrementAcumReward(const double r){
    acum_reward+=r;
}


void ActiveAgentClass::resetAcumReward(){
    acum_reward = 0;
}

void ActiveAgentClass::writeMetrics(){
    // f_metrics << ros::Time::now().toSec() << "," << x << "," << y << "," << this->robot->GetV() << "," << this->robot->GetW() << endl;
    // f_metrics << this->robot->GetV() << "-" << this->robot->GetW() << "-" << this->distObsCurrentState << ", " << flush;
}         

void ActiveAgentClass::writeTrackerEstimation(){
    double t = ros::Time::now().toSec();
    for (auto& a:agents){
        if (a->GetV() != 0 && a->GetW() != 0){
            // f_metrics << t << "," << a->GetLocalization().x << "," << a->GetLocalization().y << "," << a->GetV() << "," << a->GetW() << endl;
        }
    }
}

void ActiveAgentClass::resetPositionCallback(const geometry_msgs::PoseArray& msg){
    if (random_scenario){
        this->robot->SetVelocity(Velocidad(0,0));
        this->waiting_for_ack = false;
        robot->SetLocalization(Tsc(msg.poses[0].position.x, msg.poses[0].position.y, msg.poses[0].orientation.z));
        this->iteracion = 0;
        this-> index_goal = 0;
        if (!global_scenario){
            this->finished = false; 
            this->goal.x = msg.poses[1].position.x;
            this->goal.y = msg.poses[1].position.y;
            this->robot->AddGoal(msg.poses[1].position.x, msg.poses[1].position.y);
        }
        else{
            std_srvs::Empty empty_srv;
            while (!this->clean_map_client.call(empty_srv))
            {
                ROS_ERROR("Failed to call service reset position");
            }
            geometry_msgs::PoseStamped global_goal;
            global_goal.pose.position.x = msg.poses[1].position.x;
            global_goal.pose.position.y = msg.poses[1].position.y;
            global_planner_goal_pub.publish(global_goal);
            this->new_iteration_global = false; 
        }
    }
    else{
        robot->SetLocalization(Tsc(original_x, original_y, original_theta));
        this->robot->SetVelocity(Velocidad(0,0));
        this->waiting_for_ack = false;
        geometry_msgs::PoseWithCovarianceStamped amcl_msg;
        amcl_msg.header.frame_id = "map";
        amcl_msg.header.stamp = ros::Time::now();
        amcl_msg.pose.pose.position.x = original_x;
        amcl_msg.pose.pose.position.y = original_y;
        tf::Quaternion q(0,0,original_theta);
        amcl_msg.pose.pose.orientation.w = q.getW();
        amcl_msg.pose.pose.orientation.x = q.getX();
        amcl_msg.pose.pose.orientation.y = q.getY();
        amcl_msg.pose.pose.orientation.z = q.getZ();
        amcl_msg.pose.covariance = {0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787};
        this->own_amcl_pub.publish(amcl_msg);
        this->iteracion = 0;
        if (!global_scenario){
            this->finished = false;
            this->robot->AddGoal(this->goal.x, this->goal.y);
            this-> index_goal = 0;
        }
        else{
            std_srvs::Empty empty_srv;
            while (!this->clean_map_client.call(empty_srv))
            {
                ROS_ERROR("Failed to call service global goal");
            }
            geometry_msgs::PoseStamped global_goal;
            global_goal.pose.position.x = msg.poses[1].position.x;
            global_goal.pose.position.y = msg.poses[1].position.y;
            global_planner_goal_pub.publish(global_goal);
            this->new_iteration_global = false;
        }
        // cout << "Sent amcl no random" << endl;
    }
}

// void ActiveAgentClass::robotNearby(const nav_msgs::Odometry& msg){
    // string id_s = msg.header.frame_id;
    // id_s = id_s.substr(id_s.find('_')+1);
    // id_s = id_s.substr(0, id_s.find('/'));
    // int id = 0;
    // if(id_s != ""){
    //     id = stoi(id_s);
    // }
    // tf::Quaternion q(
    //     msg.pose.pose.orientation.x,
    //     msg.pose.pose.orientation.y,
    //     msg.pose.pose.orientation.z,
    //     msg.pose.pose.orientation.w);
    // tf::Matrix3x3 m(q);
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);
    // Tsc loc(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw);
    // double v_lin = sqrt(msg.twist.twist.linear.x * msg.twist.twist.linear.x + 
    // msg.twist.twist.linear.y * msg.twist.twist.linear.y);
    // Velocidad v(v_lin, msg.twist.twist.angular.z);
    // bool found = false;
    // for (auto& a : agents){
    //     if (a->GetId() == id){
    //         found = true;
    //         a->SetVelocity(v);
    //         a->SetLocalization(loc);
    //     }
    // }
    // if(!found){
    //     std::unique_ptr<Agent> a(new PassiveAgent(id, msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, v_lin, msg.twist.twist.angular.z, 0.2));
    //     agents.push_back(move(a));
    // }
// }

void ActiveAgentClass::collaborativeCallback(const rl_dovs::CollaborativeInfo& msg){
    if (msg.id != this->id){
        this->agents_collaborative[msg.id].radius = msg.radius;
        this->agents_collaborative[msg.id].x = msg.x;
        this->agents_collaborative[msg.id].y = msg.y;
        this->agents_collaborative[msg.id].linear_vel = msg.linear_vel;
        this->agents_collaborative[msg.id].angular_vel = msg.angular_vel;
        this->agents_collaborative[msg.id].theta = msg.theta;
        this->agents_collaborative[msg.id].obs_id = msg.id;
    }
}

void ActiveAgentClass::rvizGoalCallback(const geometry_msgs::PoseStamped& msg){
    // this->goal.x = msg.pose.position.x;
    // this->goal.y = msg.pose.position.y;
    // printf("goal received!!\n"); 
    // this->robot->AddGoal(this->goal.x, this->goal.y);

    // nav_msgs::GetPlan srv;
    
    // fillPathRequest(srv.request,this->x,this->y,msg.pose.position.x,msg.pose.position.y);
    // callPlanningService(make_plan_client,srv);
    // this->current_path = srv.response.plan;
    this-> index_goal = 0;
    // this->goal.x = current_path.poses[index_goal].pose.position.x;
    // this->goal.y = current_path.poses[index_goal].pose.position.y;
    this->goal.x = msg.pose.position.x;
    this->goal.y = msg.pose.position.y;
    printf("goal received!!\n"); 
    // cout << goal.x << ", " << goal.y << ", " << current_path.poses.size() << endl;
    this->robot->AddGoal(this->goal.x, this->goal.y);
    if (!coll_received && !too_many_iterations){
        finished = false;
    }
}

void ActiveAgentClass::collaborativePublish(){
    rl_dovs::CollaborativeInfo msg;
    msg.radius = this->GetRealRadius();
    Tsc loc = this->GetLocalization();
    msg.x = loc.x;
    msg.y = loc.y;
    msg.linear_vel = this->GetV();
    msg.angular_vel = this->GetW();
    msg.theta = loc.tita;
    msg.id = this->id;
    this->pub_collaborative_position.publish(msg);
}

void ActiveAgentClass::obstaclesCallback(const obstacle_detector::Obstacles& msg){
    agents.clear();
    double difx, dify;
    int obs_id = this->robot->GetId()+1;
    if (!collaborative_scenario){
        for (auto obs : msg.circles){
            // std::unique_ptr<Agent> a(new PassiveAgent(obs_id++, obs.center.x, obs.center.y, obs.theta, obs.linear_vel, obs.angular_vel, sqrt(0.08)));
            std::unique_ptr<Agent> a(new PassiveAgent(obs_id++, obs.center.x, obs.center.y, obs.theta, obs.linear_vel, obs.angular_vel, obs.true_radius));
            agents.push_back(move(a));
        }
    }
    else{
        for (auto obs : msg.circles){
            // if (obs.linear_vel > 0.1){
                // std::unique_ptr<Agent> a(new PassiveAgent(obs_id++, obs.center.x, obs.center.y, obs.theta, obs.linear_vel, obs.angular_vel, sqrt(0.08)));
                std::unique_ptr<Agent> a(new PassiveAgent(obs_id++, obs.center.x, obs.center.y, obs.theta, obs.linear_vel, obs.angular_vel, obs.true_radius));
                agents.push_back(move(a));
            // }
        }
    }
    for (auto obs : msg.segments){
        // double x0, y0, xn, yn;
        // if (obs.first_point.x < obs.last_point.x){
        //     x0 = obs.first_point.x;
        //     y0 = obs.first_point.y;
        //     xn = obs.last_point.x;
        //     yn = obs.last_point.y;
        // }
        // else{
        //     x0 = obs.last_point.x;
        //     y0 = obs.last_point.y;
        //     xn = obs.first_point.x;
        //     yn = obs.first_point.y;                   
        // }
        // double angle = atan2(xn-x0, yn-y0);
        // while (x0 + 0.1*sin(angle) < xn){
        //     difx = x0+0.1*sin(angle) - this->x;
        //     dify = y0+0.1*cos(angle) - this->y;
        //     if (sqrt(difx*difx + dify*dify) <= 5){
        //         std::unique_ptr<Agent> a(new PassiveAgent(obs_id++, x0+0.1*sin(angle), y0+0.1*cos(angle), 0, 0, 0, 0.1));
        //         agents.push_back(move(a));
        //     }
        //     x0 += 0.1*sin(angle);
        //     y0 += 0.1*cos(angle);
        // }
        // difx = x0+0.1*sin(angle) - this->x;
        // dify = y0+0.1*cos(angle) - this->y;
        // if (sqrt(difx*difx + dify*dify) <= 5){
        //     std::unique_ptr<Agent> a(new PassiveAgent(obs_id++, (xn+x0)/2, (yn+y0)/2, 0, 0, 0, sqrt((xn-x0)*(xn-x0) + (yn-y0)*(yn-y0))/2));
        //     agents.push_back(move(a));
        // }
        double difx = obs.last_point.x-obs.first_point.x;
        double dify = obs.last_point.y-obs.first_point.y;
        
        double theta_obs = NormalisePI(atan2(dify, difx));
        
        double radius_obs = sqrt(difx*difx + dify*dify)/2;
        Line segment_line = Line(Tpf(obs.first_point.x, obs.first_point.y), Tpf(obs.last_point.x, obs.last_point.y));
        Tsc loc = this->GetLocalization();
        double dist_segm = segment_line.Distance(Tpf(loc.x, loc.y));
        Tsc nearest_point;
        int sol;
        SolDosRectas(segment_line, Line(Tpf(loc.x,loc.y), theta_obs + M_PI_2), nearest_point, sol);
        std::unique_ptr<Agent> a(new PassiveAgent(obs_id++, nearest_point.x, 
                                    nearest_point.y, theta_obs, 0, 0, radius_obs, true,
                                    dist_segm, Tpf(obs.first_point.x, obs.first_point.y), Tpf(obs.last_point.x, obs.last_point.y)));
        // cout << "Nearest point: " << nearest_point.x << ", " << nearest_point.y << ". Distance: " << dist_segm << endl;
        // std::unique_ptr<Agent> a(new PassiveAgent(obs_id++, obs.first_point.x, 
        //                             obs.first_point.y, theta_obs, 0, 0, radius_obs, true,
        //                             dist_segm));
        // geometry_msgs::PointStamped pt0, pt1, pt0_tf, pt1_tf;
        // pt0.header.frame_id = "map";
        // pt1.header.frame_id = "map";
        // ros::Time cur_time = ros::Time::now();
        // pt0.header.stamp = cur_time;
        // pt1.header.stamp = cur_time;
        // pt0.point.x = obs.first_point.x;
        // pt0.point.y = obs.first_point.y;
        // pt0.point.z = 0;
        // pt1.point.x = obs.last_point.x;
        // pt1.point.y = obs.last_point.x;
        // pt1.point.z = 0;
        // listener.transformPoint("robot_0/base_link", pt0, pt0_tf);
        // listener.transformPoint("robot_0/base_link", pt1, pt1_tf);
        // cout << "Obstacle rerceived: "<< pt0_tf.point.x << ", " << pt0_tf.point.y << " - " << pt1_tf.point.x << ", " << pt1_tf.point.y << endl;
        // cout << "Center in local: " << (pt0_tf.point.x + pt1_tf.point.x)/2 << ", "  << (pt0_tf.point.y + pt1_tf.point.y)/2 << endl;
        agents.push_back(move(a));
    }
}

void ActiveAgentClass::observe(std::vector<float>& state, std::vector<int>& goal_actions, double& reward, bool& done, bool from_reset){
    state.clear();
    goal_actions.clear();
    if (!finished){
        if(graph){
            SeleccionaVentana(this->GetId()+1);
            LimpiaVentana();
        }
        // If actions 5, 6 or 7 (dir to goal) are valid, they are returned in the vector safeVel.
        // Actions 0-4 are always valid
        this->robot->ModelAgentsFusion(&agents, time_step, lookAhead, timeHorizon, accConst);
        possibleVelocities =  this->robot->isGoalInDW (time_step, &agents, 
                                        video, iteracion, graph, debug, goal_actions, true);
        // cout << "Goal vel size: " << goalVel.size() << endl;
        // validActions = getActions();
        // To get the state get whether the velocities are free for all the grid. Check findClosestSafe as example.
        // Do not do it with velocities that do not satisfy dynamic constraints.
        this->getStateDQN(state);
        // cout << "Chosen action: " << chosen_velocity << endl;
        // v = possibleVelocities[chosen_velocity];
        // for (size_t idx_vel = 0; idx_vel < possibleVelocities.size(); idx_vel++){
        //     cout << "Vel " << idx_vel << "-> V: " << possibleVelocities[idx_vel].v << ", W: " << possibleVelocities[idx_vel].w << endl; 
        // }
        // previous_state = state;
        // previous_chosen_action = numChosenAction;
        // string times_get_state;
        // state = getState(zone, times_get_state);
        end_flag end_flag_iter;
        done = false;
        // reward = -0.0025;
        reward = getReward();
        // reward = -(float) rand()/RAND_MAX;
        if (iteracion!=0){
            // reward = getReward();
            // reward = -0.001;
            if (iteracion >= 500 && multi){
                done = true;
                // reward = -10;
                unreachable = true;
                end_flag_iter = unreachable_f;
            }
            if (this->robot->IsFinished(min_dist)){
                success = true;
                done = true;
                reward = 15;
                // reward = 1;
                success = true;
                end_flag_iter = success_f;
            }
            // if (state.getDistanceToGoal() == 4) {
            //     reward = -10;
            // }
            if (coll_received){
                done = true;
                reward = -15;
                // reward = -0.25;
                collision_end = true;
                end_flag_iter = collision_f;
            }
            // if (!no_training){
            //     this->storeTransitionDQN(reward, done);
            // }
        }
        incrementAcumReward(reward);
        if (done && !from_reset){
            bool aux_bool;
            setFinish(aux_bool, end_flag_iter);
        }
        if (test){
            double min_d = 30;
            double radio_seg = 0;
            double distancia;
            for (auto it2 = agents.begin(); it2 != agents.end(); ++it2){
                if ((*it2)->GetId() != this->robot->GetId()){      
                    if ((*it2)->isSegment()){
                        distancia = (*it2)->getDistanceSegment() - 0.15;
                    }
                    else{
                        radio_seg = this->robot->GetRealRadius() + (*it2)->GetRealRadius();
                        distancia = Distancia(this->robot->GetLocalization().x 
                            - (*it2)->GetLocalization().x, this->robot->GetLocalization().y 
                            - (*it2)->GetLocalization().y);
                    }
                    if (distancia - radio_seg < min_d) {
                        min_d = distancia - radio_seg;
                    }
                }
            }
            distance_obstacles_episode.push_back(min_d);
        }
        agents_before.clear();
        for (unsigned int i_agent = 0; i_agent < agents.size(); i_agent++){
            agents_before.push_back(std::move(agents[i_agent]));
        }
        agents.clear();
        iteracion++;
    }
}

void ActiveAgentClass::publishVelocity(){
    Velocidad v(0,0);
    // cout << iteracion << endl;
    if (!finished){
        if (collaborative_scenario){
            for (int obs_id = 0; obs_id < nActives+nPasives; obs_id++){
                if (obs_id != this->id && agents_collaborative[obs_id].obs_id != -1){
                    std::unique_ptr<Agent> a(new PassiveAgent(agents_collaborative[obs_id].obs_id, agents_collaborative[obs_id].x, agents_collaborative[obs_id].y, 
                                                                    agents_collaborative[obs_id].theta, agents_collaborative[obs_id].linear_vel, agents_collaborative[obs_id].angular_vel,
                                                                    agents_collaborative[obs_id].radius));
                    agents.push_back(move(a));
                    agents_collaborative[obs_id].obs_id = -1;
                }
            }
        }
        if (video){
            // SeleccionaVentana(this->GetId()+1);
            // LimpiaVentana();
            disfin();
            boost::filesystem::create_directory("output");
            setfil("output/vs.png");
            filopt( "NONE", "SEPARATOR");
            filopt("LONG", "NUMBER");
            filopt("6", "DIGITS");
            Inicializar(video);
        }
        if (sac){
            std::vector<float> state, goal_state;
            std::vector<int> goal_actions;
            double reward;
            bool done;
            observe(state, goal_actions, reward, done);
            srv_SAC_action.request.state = state;
            while (!this->client_SAC_action.call(srv_SAC_action))
            {
                ROS_ERROR("Failed to call service dqn action");
            }
            v = velFromAction(srv_SAC_action.response.x1, srv_SAC_action.response.x2);
        }
        else if(deep_q_learning){
            std::vector<float> state, goal_state;
            std::vector<int> goal_actions;
            double reward;
            bool done;
            observe(state, goal_actions, reward, done);
            srv_DQN_action.request.state = state;
            srv_DQN_action.request.goal_actions = goal_actions;
            while (!this->client_DQN_action.call(srv_DQN_action))
            {
                ROS_ERROR("Failed to call service dqn action");
            }
            v = possibleVelocities[srv_DQN_action.response.action];
        }
        else if(learning){
            if(graph){
                SeleccionaVentana(this->GetId()+1);
                LimpiaVentana();
            }
            ros::Time times_measured[6];
            times_measured[0] = ros::Time::now();
            this->robot->ModelAgentsFusion(&agents, time_step, lookAhead, timeHorizon, accConst);
            times_measured[1] = ros::Time::now();
            validActions = getActions();
            times_measured[2] = ros::Time::now();
            previous_state = state;
            previous_chosen_action = numChosenAction;
            string times_get_state;
            state = getState(zone, times_get_state);
            times_measured[3] = ros::Time::now();
            if (iteracion!=0 && !no_training && !skip_iteration){
                reward = getReward();
                // if (iteracion >= 500 && multi){
                //     // setFinish(unreachable, unreachable_f);
                //     reward = -0;
                // }
                if (this->robot->IsFinished(min_dist)){
                    // setFinish(success, success_f);
                    success = true;
                    reward = 15;
                }
                if (coll_received){
                    // setFinish(collision_end, collision_f);
                    reward = -15;
                }
                qValueNew = qTable.find(state);
                // cout << "Reward: " << reward << endl;
                if (qValueNew != qTable.end()) {    // The new state exists
                    max = -INF;
                    for (int i=0; i<numActions; i++) {      // Obtain maximum Q-Value of new state
                        if (qValueNew->second[i] > max) {
                            max = qValueNew->second[i];
                        }
                    }
                    // qValue stores the value of the previous iteration
                    qValue->second[previous_chosen_action] = qValue->second[previous_chosen_action] 
                            + alpha*(reward + gamma*max - qValue->second[previous_chosen_action]);
                    // cout << "qValue: " << qValue->second[previous_chosen_action] 
                    //         + alpha*(reward + gamma*max - qValue->second[previous_chosen_action]) << endl;
                } else {    // The new state doesn't exist
                    qValue->second[previous_chosen_action] = qValue->second[previous_chosen_action] 
                            + alpha*(reward - qValue->second[previous_chosen_action]);
                    // cout << "qValue: " << qValue->second[previous_chosen_action] 
                    //         + alpha*(reward - qValue->second[previous_chosen_action]) << endl;
                }
            }
            times_measured[4] = ros::Time::now();
            v = applyLearning(state);
            times_measured[5] = ros::Time::now();
            if (record_steps && id == 0){
                f_record_steps << "--------------------------------------------------------------------------------------------" << endl;
                f_record_steps << "Time: " << ros::Time::now() << endl;
                f_record_steps << "State: " << state.getCode() << endl;
                f_record_steps << "Chosen action: " << numChosenAction << endl;
                f_record_steps << "Previous reward: " << reward << endl;
                Tsc loc = this->GetLocalization();
                f_record_steps << "Position: " << loc.x << ", " << loc.y << endl;
                f_record_steps << "Velocity: V " << v.v << " ,W " << v.w << endl;
                f_record_steps << "GetV: " << this->robot->GetV() << " GetW: " << this->robot->GetW() << endl;
                Tpf goalAg;
                this->robot->GetCurrentGoal(goalAg);
                f_record_steps << "DistCurrentState (dGoal): " << distCurrentState << "Angle goal: " << atan2(goalAg.y, goalAg.x) << endl;
                f_record_steps << "PosObs: X " << posObs.x << ", Y "<< posObs.y << " Vobs: " << vObs << endl;
                f_record_steps << "RelativeDist: " << distObsCurrentState << " RelTheta: " << relThetaCurrentState << endl;                    
                for (int i_times = 0; i_times < 5; i_times++){
                    f_record_steps << "Time recorded " << i_times << ": " << times_measured[i_times+1] - times_measured[i_times] << endl;
                }
                f_record_steps << "Number of agents: " << agents.size() << endl;
            }
            if (iteracion >= 500 && multi){
                setFinish(unreachable, unreachable_f);
                // reward = -10;
            }
            if (this->robot->IsFinished(min_dist)){
                setFinish(success, success_f);
                success = true;
            }
            if (coll_received){
                setFinish(collision_end, collision_f);
            }
            acum_reward+=reward;
            // cout << state.getCode() << endl;
        }
        else{
            if (this->robot->IsFinished(min_dist)){
                setFinish(success, success_f);
                success = true;
            }
            if (iteracion >= 500 && multi){
                setFinish(unreachable, unreachable_f);
                // reward = -10;
            }
            // for (const auto& a:agents){
            //     Tsc pos_a = a->GetLocalization();
            //     // cout << "Agent " << a->GetId() << ": " << Distancia(x-pos_a.x, y-pos_a.y) << ", " << radio + a->GetRealRadius() + 0.01 << endl;
            //     // if(Distancia(x-pos_a.x, y-pos_a.y) <= radio + a->GetRealRadius() + 0.001){
            //     //     setFinish(collision);
            //     // }
            // }
            if (coll_received){
                setFinish(collision_end, collision_f);
                reward = -10;
            }
            if(graph){
                SeleccionaVentana(this->GetId()+1);
                LimpiaVentana();
            }
            this->robot->ModelAgentsFusion(&agents, time_step, lookAhead, timeHorizon, accConst);
            validActions = getActions();
            switch (algorithm) {
                case 0: //Greedy (2D planning)
                    v = this->robot->MotionGreedy(time_step, &agents, video);
                    break;
                case 1: //Strategies (2D planning)
                    v = this->robot->MotionStrategy(time_step, &agents, video, iteracion, graph, debug);
                    break;
                case 2: //A* (3D planning)
                    std::ifstream fp("data/planner3d_data.txt");
                    double th; unsigned n, m; unsigned long numC;
                    fp >> th;   //time horizon
                    fp >> n; fp >> m; //dovt size
                    fp >> numC;  //number of cells to filled downwards

                    assert(th > 0 && n > 0 && m > 0 && numC >= 0);
                    v = this->robot->MotionPlan(time_step, th, n, m, numC, lookAhead);
                    break;
            }
        }
        if (test){
            double min_d = 30;
            double radio_seg = 0;
            double distancia;
            for (auto it2 = agents.begin(); it2 != agents.end(); ++it2){
                if ((*it2)->GetId() != this->robot->GetId()){      
                    if ((*it2)->isSegment()){
                        distancia = (*it2)->getDistanceSegment() - 0.15;
                    }
                    else{
                        radio_seg = this->robot->GetRealRadius() + (*it2)->GetRealRadius();
                        distancia = Distancia(this->robot->GetLocalization().x 
                            - (*it2)->GetLocalization().x, this->robot->GetLocalization().y 
                            - (*it2)->GetLocalization().y);
                    }
                    if (distancia - radio_seg < min_d) {
                        min_d = distancia - radio_seg;
                    }
                }
            }
            distance_obstacles_episode.push_back(min_d);
            linear_velocities_episode.push_back(v.v);
        }
        agents.clear();
        iteracion++;    
    }
    this->robot->SetVelocity(v);
    // v.v = 0.0;
    // v.w = 0.0;
    geometry_msgs::Twist msg;
    msg.linear.x = v.v;
    msg.angular.z = v.w;
    // cout << "Chosen vel: " << v.v << ", " << v.w << endl;
   // msg.linear.x = 0.0;
   // msg.angular.z = 0.0;
    pub.publish(msg);
    if (collaborative_scenario){
        this->collaborativePublish();
    }
}

class Vector2D {
public:
    double w, v;

    // Constructors
    Vector2D() : w(0.0f), v(0.0f) {}
    Vector2D(double w, double v) : w(w), v(v) {}

    // Member functions
    double magnitude() const {
        return std::sqrt(w * w + v * v);
    }

    // Overloaded operators
    Vector2D operator+(const Vector2D& other) const {
        return Vector2D(w + other.w, v + other.v);
    }

    Vector2D operator-(const Vector2D& other) const {
        return Vector2D(w - other.w, v - other.v);
    }

    Vector2D operator*(double scalar) const {
        return Vector2D(w * scalar, v * scalar);
    }

    // Dot product
    double dot(const Vector2D& other) const {
        return w * other.w + v * other.v;
    }
};

Velocidad ActiveAgentClass::velFromAction(double a1, double a2){
    Velocidad v;
    Vector2D v_down(this->GetW(), this->GetV()-this->robot->GetAV()*time_step);
    Vector2D b1(-this->robot->GetAW()*time_step, this->robot->GetAV()*time_step);
    Vector2D b2(this->robot->GetAW()*time_step, this->robot->GetAV()*time_step);
    double w_p1 = (wMax*v_down.v/vMax + v_down.w - wMax)/2;
    double v_p1 = (vMax*v_down.w/wMax + v_down.v - vMax)/2 + vMax;
    double w_p2 = (-wMax*v_down.v/vMax + v_down.w + wMax)/2;
    double v_p2 = (-vMax*v_down.w/wMax + v_down.v - vMax)/2 + vMax;
    Vector2D dist1(w_p1 - v_down.w, v_p1 - v_down.v);
    Vector2D dist2(w_p2 - v_down.w, v_p2 - v_down.v);
    a1 = std::min(a1, dist1.magnitude()/b1.magnitude());
    a2 = std::min(a2, dist2.magnitude()/b2.magnitude());
    Vector2D v_k = v_down + b1*a1 + b2*a2;
    v.v = std::max(0.0, v_k.v);
    v.w = v_k.w;
    return v;
}


bool ActiveAgentClass::storeTransitionDQN (const double reward, const bool set_done){
    srv_DQN_store.request.reward = reward;
    srv_DQN_store.request.done = set_done;
    bool ret_value = 1;
    if (client_DQN_store.call(srv_DQN_store))
    {
        ret_value = srv_DQN_store.response.ret_value;
    }
    else
    {
        ROS_ERROR("Failed to call service DQN_store");
    }
    return ret_value;
}

void ActiveAgentClass::getStateDQN (std::vector<float>& state){
    // Update last distance
    distLastState = distCurrentState;

    // Update last theta
    thetaLastState = thetaCurrentState;

    // Update last distance to obstacle
    distObsLastState = distObsCurrentState;

    // Update last velocities
    vLastState = vCurrentState;
    wLastState = wCurrentState;
    this->wCurrentState = this->robot->GetW();

    // Update ID of last closest agent
    lastClosestAgent = currentClosestAgent;

    // Update orientation of last closest agent
    thetaObsLastState = thetaObsCurrentState;
    relThetaLastState = relThetaCurrentState;

    // Update last distances and orientations to CB points
    lastCollision = currentCollision;

    double d = 10000;
    Tpf goalAg;
    double goalX, goalY;
    double radio_seg = 0;
    Tsc loc;
    double thetaObs = 0;
    int idx;
    double distancia;
    this->vObs = 0;
    for (auto it2 = agents.begin(); it2 != agents.end(); ++it2){
        if ((*it2)->GetId() != this->robot->GetId()){      
            if ((*it2)->isSegment()){
                distancia = (*it2)->getDistanceSegment() - this->robot->GetRealRadius();
            }
            else{
                radio_seg = this->robot->GetRealRadius() + (*it2)->GetRealRadius();
                distancia = Distancia(this->robot->GetLocalization().x 
                    - (*it2)->GetLocalization().x, this->robot->GetLocalization().y 
                    - (*it2)->GetLocalization().y);
            }
            if (distancia - radio_seg < d) {
                d = distancia - radio_seg;
                this->currentClosestAgent = (*it2)->GetId();
                this->posObs = (*it2)->GetLocalization();
                this->vObs = (*it2)->GetV();
                thetaObs = posObs.tita;
                idx = (*it2)->GetId();
            }
        }
    }
    this->robot->GetCurrentGoal(goalAg);
    goalX = goalAg.x; goalY = goalAg.y;
    loc = this->robot->GetLocalization();
    transfor_inversa_p(goalX, goalY, &loc, &goalAg);

    // Update current distance
    this->distCurrentState = sqrt(goalAg.x*goalAg.x + goalAg.y*goalAg.y);

    // Update current theta
    double angle_goal = NormalisePI(atan2(goalAg.y, goalAg.x));
    this->thetaCurrentState = angle_goal;

    // Update current distance to obstacle
    this->distObsCurrentState = d;


    // cout << "Free velocities" << freeVelocities << " - ClosestFreeVel: " << closestFreeVelocity << endl;

    float theta = loc.tita;
    float angle = 0;
    std::vector<double> currentDirection {cos(theta), sin(theta)};
    if (this->distObsCurrentState < 1000){
        std::vector<double> obsDirection {posObs.x - loc.x,posObs.y - loc.y};
        float dot = currentDirection[0] * obsDirection[0] + currentDirection[1]*obsDirection[1];
        float det = currentDirection[0]*obsDirection[1] - currentDirection[1]*obsDirection[0];
        angle = NormalisePI(atan2(det, dot));
        this->thetaObsCurrentState = angle;
        this->relThetaCurrentState =  NormalisePI(atan2(sin(thetaObs-theta), cos(thetaObs-theta)));
    }
    else{
        this->relThetaCurrentState = 0;
    }

    int collision;
    int collisionDistance = 3;
    // int collision = this->distObsCurrentState < collisionDistance;

    // Going towards collision
    collision = this->lastCollision;

    if ( (collision == 1) && this->currentClosestAgent != this->lastClosestAgent 
    // if ( (this->lastCollision == 1) && this->currentClosestAgent != this->lastClosestAgent 
        && this->distObsCurrentState < collisionDistance ) {
        // std::cout << "New agent" << std::endl;
        this->currentCollision = 0;
        this->lastCollision = 0;
        // collision = 1;
        collision = 0;
    }

    if (this->lastCollision == 0 && this->distObsCurrentState < collisionDistance 
        && this->distCurrentState > 2 && (angle <= M_PI/3 && angle > -M_PI/3) ) {
            // std::cout << "New danger: agent " << this->currentClosestAgent << std::endl;
            // std::cout << "THObs: " << thObs << std::endl;
            this->lastCollisionAgent = this->currentCollisionAgent;
            this->currentCollisionAgent = this->currentClosestAgent;
            collision = 1;
            this->currentCollision = 1;
            this->thetaObsOriginal = angle;
    }

    int thObs;
    if (collision > 0 ) {
        if (angle > 2*M_PI/3) {
            thObs = 1;
        } else if (angle > M_PI/2) {
            thObs = 2;
        } else if (angle > M_PI/3) {
            thObs = 3;
        } else if (angle > 0) {
            thObs = 4;
        } else if (angle > -M_PI/3) {
            thObs = 5;
        } else if (angle > -M_PI/2) {
            thObs = 6;
        } else if (angle > -2*M_PI/3) {
            thObs = 7;
        } else {
            thObs = 8;
        }
    } else {
        thObs = 0;
    }

    if (this->currentCollision == 1 && this->lastCollision == 1 
        // && ( (thObs != 4 && thObs != 5) 
        && ( (thObs != 2 && thObs != 3) 
        || this->currentClosestAgent != this->lastClosestAgent) ) {
        // std::cout << "Evaded obstacle" << std::endl;
        collision = 0;
        this->currentCollision = 0;
    }

    this->robot->getFreeVelocitiesVector(state);
    state.push_back(this->robot->GetV());
    state.push_back(this->robot->GetW());
    state.push_back(sqrt(goalAg.x*goalAg.x + goalAg.y*goalAg.y));
    state.push_back(NormalisePI(atan2(goalAg.y, goalAg.x)));
    state.push_back(this->relThetaCurrentState);
    state.push_back(vObs);
    state.push_back(angle);
    state.push_back(this->distObsCurrentState);
}

void ActiveAgentClass::writeQTable(std::map<State, std::array<double, 8>> qTable, const std::string& fileName) {
    std::map<State, std::array<double, 8>>::iterator it;
    std::ofstream of;
    // std::cout << "Writing qTable to " << fileName << std::endl;
    of.open(fileName);
    for (it = qTable.begin(); it != qTable.end(); it++) {
        of << it->first.getCode() << "\t" << it->second[0] << "\t" << it->second[1] << "\t" 
        << it->second[2] << "\t" << it->second[3] << "\t" << it->second[4] << "\t" 
        << it->second[5]<< "\t" << it->second[6]<< "\t" << it->second[7] << std::endl;
    }
    of.close();
    // std::cout << "Done!" << std::endl;
}

void ActiveAgentClass::writeLearningStats(const end_flag flag_out, const int iteracion, const double r) {
    if (linear_velocities_episode.size() > 0){
        // std::cout << "Writing stats" << std::endl;
        double max_v_episode = *std::max_element(linear_velocities_episode.begin(), linear_velocities_episode.end());
        size_t count = linear_velocities_episode.size();
        // cout << "1" << endl;
        double mean_v_episode = std::accumulate(linear_velocities_episode.begin(), linear_velocities_episode.end(),0.0) / (double)count;
        double min_d_episode = *std::min_element(distance_obstacles_episode.begin(), distance_obstacles_episode.end());
        count = distance_obstacles_episode.size();
        // cout << "2" << endl;
        double mean_d_episode = std::accumulate(distance_obstacles_episode.begin(), distance_obstacles_episode.end(),0.0) / (double)count;
        distance_obstacles_episode.clear();
        linear_velocities_episode.clear();
        // f_results_metrics_episode << iteracion << "/" << flag_out << "/" << max_v_episode << "/" << mean_v_episode << "/" << min_d_episode << "/" << mean_d_episode << "/" << r << endl;
    }
    distance_obstacles_episode.clear();
    linear_velocities_episode.clear();
    std::cout << "Iteracion: " << iteracion << std::endl;
    if (iteracion>0){
        f_results_metrics_episode << iteracion << "/" << flag_out <<  "/" << r << endl;
        std_msgs::Empty new_episode_msg;
        new_episode_pub.publish(new_episode_msg);
    }

    // std::cout << "Done!" << std::endl;
}

void ActiveAgentClass::readQTableFile(std::map<State, std::array<double, 8>> &qTable, std::string fileName) {
    qTable.clear();
    std::string value;
    std::ifstream f(fileName);
    if (f.is_open()) {
        // std::cout << "Reading qTable from " << fileName << std::endl;
        while (f.peek() != EOF) {
            f >> value;
            State newState(std::atol(value.c_str()));
            std::array<double, 8> newValues;
            for (int i=0; i < 8; i++) {
                f >> value;
                newValues[i] = std::atof(value.c_str());
            }

            qTable.insert(std::pair<State, std::array<double, 8>>(newState, newValues));
        }
        f.close();
        // std::cout << "Done!" << std::endl;
    } else {
        std::cerr << "Could not open file " << fileName << std::endl;
    }
}

Velocidad ActiveAgentClass::applyLearning(State state){
    oldAction = action;
    numAction = 0;
    
    // For each state of active agents, explore or exploit
        
    // Update state frequency
    stateFrequency = stateFrequencies.find(state);
    if (stateFrequency == stateFrequencies.end()) {
        stateFrequencies.insert(std::pair<State, int> (state, 1));
    } else {
        stateFrequency->second++;
    }

    // Find the Q-Values of the state, if they exist
    qValue = qTable.find(state);
    if (qValue == qTable.end()) { // The q value does not exist
        numNewStates++;
        std::array<double, numActions> newValues;
        qTable.insert(std::pair<State, std::array<double, numActions>>(state, newValues));
        qValue = qTable.find(state);
    }

    if (!manual) {      // Non-manual mode, explore or exploit

        // Explore or exploit to choose action
        double candidate = distributionEpsilon(generator);
        if (distributionEpsilon(generator) < epsilon) { //Explore
            cout << "EXPLORE " << candidate << ", eps: " << epsilon << endl;
            std::uniform_real_distribution<double>::param_type 
                    parExplore(0, validActions.size()-0.000000001);
            distributionExplore.param(parExplore);
            numAction = floor(distributionExplore(generator));
        } else { // Exploit
            max = -INF;
            std::vector<int> max_vector;
            for (int i=0; i<validActions.size(); i++) {
                if (qValue->second[validActions[i].first] > max ) {
                    max = qValue->second[validActions[i].first];
                    max_vector.clear();
                    max_vector.push_back(i);
                }
                else if (qValue->second[validActions[i].first] == max){
                    max_vector.push_back(i);
                }
            }
            static std::mt19937 gen(rd());
            std::vector<int>::iterator start_vec = max_vector.begin();
            std::vector<int>::iterator end_vec = max_vector.end();
            std::uniform_int_distribution<> dis(0, std::distance(start_vec, end_vec) - 1);
            std::advance(start_vec, dis(gen));
            numAction = *start_vec;
            cout << "EXPLOIT: " << candidate << ", eps: " << epsilon << endl;
        }
    } else {            // Manual mode, user chooses action
        numAction = -1;
        char c;
        int  a = -1;
        while (numAction == -1) {
            std::cout << "Choose action: ";
            std::cin >> c;
            if (c == 'a') {
                a = 2;
            } else if (c == 's') {
                a = 1;
            } else if (c == 'w') {
                a = 3;
            } else if (c == 'd') {
                a = 4;
            } else if (c == 'e') {
                a = 0;
            } else if (c == '1') {
                a = 5;
            } else if (c == '2') {
                a = 6;
            } else if (c == '3') {
                a = 7;
            }

            for (int l=0; l < validActions.size(); l++) {
                if (a == validActions[l].first) {
                    numAction = l;
                }
            }
        }
    }

    // Double check limits on chosen action
    if (validActions[numAction].second.v >= 0 
        && validActions[numAction].second.v <= 1.5 
        && validActions[numAction].second.w >= -1 
        && validActions[numAction].second.w <= 1)
        
        action = validActions[numAction].second;
    else
        action = oldAction;

    // Check if the chosen action leads to goal
    if (validActions[numAction].first > 4) {
        chosenDirToGoal = true;
    } else {
        chosenDirToGoal = false;
    }

    // Store chosen action
    numChosenAction = validActions[numAction].first;
    return action;
}
std::vector<std::pair<int, Velocidad>> ActiveAgentClass::getActions(){
    std::vector<std::pair<int, Velocidad>> actions;
    if (this->robot->IsFinished(min_dist)) {
        this->robot->SetVelocity(Velocidad(0,0));
        actions.push_back(velEnd);
    }
    else{
        actions = this->robot->MotionStrategyRL(time_step, &agents, 
                free_actions, video, iteracion, graph, debug);
    }
    return actions;
}

State ActiveAgentClass::getState(int zone, string& times_get_state){
    ros::Time times_recorded_state[4];
    times_recorded_state[0] = ros::Time::now();
    // Update last distance
    this->distLastState = this->distCurrentState;

    // Update last theta
    this->thetaLastState = this->thetaCurrentState;

    // Update last distance to obstacle
    this->distObsLastState = this->distObsCurrentState;

    // Update last velocities
    this->vLastState = this->vCurrentState;
    this->wLastState = this->wCurrentState;
    // Update ID of last closest agent
    this->lastClosestAgent = this->currentClosestAgent;

    // Update orientation of last closest agent
    this->thetaObsLastState = this->thetaObsCurrentState;
    this->relThetaLastState = this->relThetaCurrentState;

    // Update last distances and orientations to CB points
    this->lastCollision = this->currentCollision;

    double d = 10000;
    Tpf goalAg;
    double goalX, goalY;
    double radio_seg;
    PosibleVelocidad pVel;
    double v, w, vObs;
    Tsc posObs;
    Tsc loc;
    double thetaObs = 0;
    int idx;
    for (auto it2 = agents.begin(); it2 != agents.end(); ++it2){
        radio_seg = robot->GetRealRadius() + (*it2)->GetRealRadius();
        double distancia = Distancia(robot->GetLocalization().x 
            - (*it2)->GetLocalization().x, robot->GetLocalization().y 
            - (*it2)->GetLocalization().y);
        if (distancia - radio_seg < d) {
            d = distancia - radio_seg;
            this->currentClosestAgent = (*it2)->GetId();
            posObs = (*it2)->GetLocalization();
            vObs = (*it2)->GetV();
            thetaObs = posObs.tita;
            idx = (*it2)->GetId();
        }
    }
    this->robot->GetCurrentGoal(goalAg);
    goalX = goalAg.x; goalY = goalAg.y;
    loc = this->robot->GetLocalization();
    transfor_inversa_p(goalX, goalY, &loc, &goalAg);

    pVel = this->robot->findClosestSafeVelocity();

    // Update current distance
    this->distCurrentState = sqrt(goalAg.x*goalAg.x + goalAg.y*goalAg.y);

    // Update current theta
    this->thetaCurrentState = atan2(goalAg.y, goalAg.x);

    // Update current distance to obstacle
    this->distObsCurrentState = d;

    // Update current velocities
    this->vCurrentState = v;
    this->wCurrentState = w;

    int dGoal;
    if (this->distCurrentState < 1) {
        dGoal = 0; // Very close
    } else if (this->distCurrentState < 2) {
        dGoal = 5; // Very close
    } else if (this->distCurrentState < 4) {
        dGoal = 1; // Close
    } else if (this->distCurrentState < 8) {
        dGoal = 2; // Far
    } else if (this->distCurrentState < 40) {
        dGoal = 3; // Very far
    } else {
        dGoal = 4; // Too far, terminate episode
    }

    int thGoal;
    if (atan2(goalAg.y, goalAg.x) < 0.1 && atan2(goalAg.y, goalAg.x) > -0.1) {
        thGoal = 0;
    } else if (atan2(goalAg.y, goalAg.x) > 2*M_PI/3 ) {
        thGoal = 1;
    } else if (atan2(goalAg.y, goalAg.x) < -2*M_PI/3) {
        thGoal = 2;
    } else if (atan2(goalAg.y, goalAg.x) > 0) {
        thGoal = 3; // Esta hacia la izquierda
    } else {
        thGoal = 4; // Esta hacia la derecha
    }

    int vel = 0;
    if (v > 1) {
        vel = 4;
    } else if (v > 0.75) {
        vel = 3;
    } else if (v > 0.5) {
        vel = 5;
    // } else if (v > 0.25) {
    //     vel = 2;
    } else if (v > 0.1) {
        vel = 1;
    } else {
        vel = 0;
    }

    int angular = 0;
    if (w > 0.5) {
        angular = 0;
    } else if (w > 0.1) {
        angular = 1;
    } else if (w <= 0.1 && w >= -0.1) {
        angular = 2;
    } else if (w > -0.5) {
        angular = 3;
    } else {
        angular = 4;
    }

    int freeVelocities;
    // if (this->freeActions) {
    if (true) {
        if (true) {
            freeVelocities = 1;
        } else {
            freeVelocities = 0;
        }
    } else {
        freeVelocities = 2;
    }

    int closestFreeVelocity;
    if (freeVelocities == 2) {
        if (pVel.valid) {
            if (pVel.vel.v > v) {
                closestFreeVelocity = 2; // Closest free velocity is above
            } else if (pVel.vel.v < v) {
                closestFreeVelocity = 4; // Closest free velocity is under
            } else if (pVel.vel.w < w) {
                closestFreeVelocity = 1; // Closest free velocity is to the left
            } else {
                closestFreeVelocity = 3; // Closest free velocity is to the right
            }
        } else {
            closestFreeVelocity = 0; // There are no free velocities
        }
    } else {
        closestFreeVelocity = 5;
    }

    float theta = loc.tita;
    std::vector<double> currentDirection {cos(theta), sin(theta)};
    std::vector<double> obsDirection {posObs.x - loc.x,posObs.y - loc.y};
    float dot = currentDirection[0] * obsDirection[0] + currentDirection[1]*obsDirection[1];
    float det = currentDirection[0]*obsDirection[1] - currentDirection[1]*obsDirection[0];
    float angle = atan2(det, dot);
    this->thetaObsCurrentState = angle;
    this->relThetaCurrentState =  atan2(sin(thetaObs-theta), cos(thetaObs-theta));

    int collision;

    // Going towards collision
    collision = this->lastCollision;

    int collisionDistance = 3;

    int thObs;
    if (collision > 0 ) {
        if (angle > 2*M_PI/3) {
            thObs = 1;
        } else if (angle > M_PI/2) {
            thObs = 2;
        } else if (angle > M_PI/3) {
            thObs = 3;
        } else if (angle > 0) {
            thObs = 4;
        } else if (angle > -M_PI/3) {
            thObs = 5;
        } else if (angle > -M_PI/2) {
            thObs = 6;
        } else if (angle > -2*M_PI/3) {
            thObs = 7;
        } else {
            thObs = 8;
        }
    } else {
        thObs = 0;
    }

    if ( (collision == 1) && this->currentClosestAgent != this->lastClosestAgent 
        && this->distObsCurrentState < collisionDistance ) {
        std::cout << "New agent" << std::endl;
        this->currentCollision = 0;
        this->lastCollision = 0;
        collision = 0;
    }

    if (this->lastCollision == 0 && this->distObsCurrentState < collisionDistance 
        && this->distCurrentState > 2 && (angle <= M_PI/3 && angle > -M_PI/3) ) {
            std::cout << "New danger: agent " << this->currentClosestAgent << std::endl;
            std::cout << "THObs: " << thObs << std::endl;
            this->lastCollisionAgent = this->currentCollisionAgent;
            this->currentCollisionAgent = this->currentClosestAgent;
            collision = 1;
            this->currentCollision = 1;
            this->thetaObsOriginal = angle;
    }


    if (this->currentCollision == 1 && this->lastCollision == 1 
        && ( (thObs != 4 && thObs != 5) 
        || this->currentClosestAgent != this->lastClosestAgent) ) {
        std::cout << "Evaded obstacle" << std::endl;
        collision = 0;
        this->currentCollision = 0;
    }

    int relativeTh;
    if (collision > 0) {
        if (this->relThetaCurrentState  > 2*M_PI/3) {
            relativeTh = 0;
        } else if (this->relThetaCurrentState  < -2*M_PI/3) {
            relativeTh = 1;
        } else if (this->relThetaCurrentState  > M_PI/3) {
            relativeTh = 2;
        } else if (this->relThetaCurrentState  < -M_PI/3) {
            relativeTh = 3;
        } else if (this->relThetaCurrentState > 0) {
            relativeTh = 4;
        } else {
            relativeTh = 5;
        }
    } else {
        relativeTh = 0;
    }

    int relativeVel;
    if (collision > 0) {
        if (vObs > 1) {
            relativeVel = 0;
        } else if (vObs > 0.5) {
            relativeVel = 1;
        } else if (vObs > 0.0) {
            relativeVel = 2;
        } else {
            relativeVel = 3;
        }
    } else {
        relativeVel = 0;
    }

    int dObs;
    if (collision == 1) {
        dObs = 1;
    } else {
        dObs = 0;
    }
    // cout << "THOBS: " << thObs << " relativeTH: " << relativeTh << " relativeVel: " << relativeVel << " distObs: " << this->distObsCurrentState << endl;
    // cout << "dGoal: " << dGoal << ". thGoal: " << thGoal << "goalAg.x: " << goalAg.x << "goalAg.y: " << goalAg.y << endl;
    State state(vel, angular, closestFreeVelocity, dGoal, thGoal, 
                        relativeTh, relativeVel, thObs);
    times_recorded_state[4] = ros::Time::now();
    // cout << state.getCode() << endl;
    return state;
}
double ActiveAgentClass::getReward() {
    double rDistGoal, rThetaGoal, rDistObs, rTimeStep, rAcceleration, rVelocity, rThetaObs;
    double kDistGoal, kThetaGoal, kDistObs, kAcceleration, kVelocity, kThetaObs;
    double reward = 0;
    
    // Reward for orientation to goal
    // if distance less than 1
    // - increase of the angle if distance greater than 1
    // increment of 0.3 if vel to goal
    if (this->distCurrentState < 1) {
        rThetaGoal = 0;
    } else {
        rThetaGoal = -(abs(this->thetaCurrentState) - abs(this->thetaLastState));
    }

    if (chosenDirToGoal && this->thetaCurrentState < M_PI/3 
        && this->thetaCurrentState > -M_PI/3 )
        rThetaGoal += 0.3;

    // Reward for distance to goal
    // - increment of the distance if distance greater than 1
    if (this->distCurrentState >= 1) {
        rDistGoal = -(this->distCurrentState - this->distLastState);
    } else {
        rDistGoal = 0;
    }
    // Penalization for time-step
    rTimeStep = -0.1;
    bool invalidate = false;

    if (this->currentCollision == 1) {
        invalidate = true;
        rAcceleration = -5;
    } else if (this->distCurrentState> 8 && ((this->thetaCurrentState < M_PI/3 
            && this->thetaCurrentState > -M_PI/3) || chosenDirToGoal) 
            && this->vCurrentState <= 1) {
        rAcceleration = (this->vCurrentState-this->vLastState);
    } else if (this->distCurrentState> 8 && (this->thetaCurrentState >= M_PI/3 
            || this->thetaCurrentState <= -M_PI/3) && this->vCurrentState > 1) {
        rAcceleration = -(this->vCurrentState-this->vLastState);
    } else if ( this->distCurrentState < 1) {
        rAcceleration = -(this->vCurrentState-this->vLastState);
    } else if (this->distCurrentState < 2 && this->vCurrentState > 0.5) {
        rAcceleration = -(this->vCurrentState-this->vLastState);
    } else if (this->distCurrentState < 8 && this->vCurrentState > 1) {
        rAcceleration = -(this->vCurrentState-this->vLastState);
    } else if (this->vCurrentState < 0.5) {
        rAcceleration = (this->vCurrentState-this->vLastState);
    } else {
        rAcceleration = -5;
        invalidate = true;
    }


    if (rAcceleration > 0.01) {
        rAcceleration = 0.20;
    } else if (rAcceleration < 0.01) {
        rAcceleration = -0.20;
    } else {
        rAcceleration = -0.1;
    }

    if (invalidate) {
        rAcceleration = 0;
    }


    // If agent in collision state, try to avoid
    double rObs;
    if (this->currentCollision > 0 && this->thetaObsCurrentState < M_PI/3 
        && this->thetaObsCurrentState > -M_PI/3) {
        rObs = abs(this->thetaObsCurrentState - this->thetaObsOriginal);
    } else {
        rObs = 0;
    }

    if (this->currentCollision > 0)
        rObs -= 0.3;

    if (this->currentCollision > 0 && this->thetaObsCurrentState < M_PI/3 
        && this->thetaObsCurrentState > -M_PI/3) {
        // rThetaGoal = 0;
        rDistGoal = 0;
    }

    // Try to make natural movements without sudden changes
    double rAcc = 0;
    if ( (this->vCurrentState != this->vLastState)) {
        rAcc = -0.3;
    }
    // This is Diego
    rAcc = 0;
    if (chosenDirToGoal){
        rAcc = 0.3;
    }
    

    // kDistGoal = 0.75; kThetaGoal = 2; kDistObs = 0; kAcceleration = 4; kVelocity = 0; 
    kDistGoal = 4; kThetaGoal = 4; kDistObs = 0.5; kAcceleration = 0.5; kVelocity = 0; 
    kThetaObs = 0;
    // cout << "RdistGoal: " << rDistGoal << "rThetaGoal: " << rThetaGoal << endl;
    reward = kDistGoal*rDistGoal + kThetaGoal*rThetaGoal + kDistObs*this->distObsCurrentState + kAcceleration*rAcceleration + 4*rObs + rAcc;
    f_record_steps << "Reward computation: rDistGoal->" << rDistGoal << " rThetaGoal->" << rThetaGoal << " rAcceleration->" << rAcceleration << " rObs->" << rObs << " rAcc->" << rAcc << endl;
    
    double r_col = 0;
    if (abs(this->distObsCurrentState) < 0.2){
        r_col = -0.1*abs(0.2 - distObsCurrentState);
    }
    double r_goal = (this->distLastState - this->distCurrentState)*2.5;
    reward =  r_col + r_goal;

    // r_goal = (this->distLastState - this->distCurrentState)/10.0;
    // if (abs(this->distObsCurrentState) < 0.2){
    //     r_col = -0.1 + distObsCurrentState/2.0;
    // }
    // reward =  r_col + r_goal;
    // if (this->robot->GetV() == 0.0){
    //     reward-=0.0025;
    // }
    // cout << "R_goal: " << r_goal << "R_col: " << r_col << "Distlaststate: "  << distLastState << "DistCurrent:" << distCurrentState<< endl;
    return reward;
}

void ActiveAgentClass::finishROS (const std_msgs::String& s){
    ros::shutdown();
}

bool ActiveAgentClass::startIterationGlobalPlanner(){
    return new_iteration_global;
}

int n_resets = 0;
int n_iterations_reset = 0;

int n_episodes_getGoal = 0;
double dist_maxGoal = 1000;
double dis_min_goal_min = 4;

void ActiveAgentClass::getGoal (double& xg, double& yg, const double x, const double y, const int n_episodes, const int idx){
    bool done = false;
    // if (idx == 0){
    //     n_iterations_reset += n_iterations;
    //     if (n_iterations_reset >= 12000){ 
    //         n_iterations_reset = n_iterations_reset %= 12000;
    //         n_resets++;
    //     }
    // }
    // double dist_minGoal = 4;
    if (curricular_goal && n_episodes < 1000){
        // if (idx == 0){
        //     this->n_iterations_getGoal += n_iterations;
        //     // if (n_iterations_getGoal >= 12000){
        //     if (iteracion % 100 ){
        //         n_iterations_getGoal = n_iterations_getGoal %= 12000;
        //         dist_maxGoal+=1;
        //         dis_min_goal_min = min(4.0, dist_maxGoal/2.0);
        //     }
        // }
        n_episodes_getGoal++;
        if (n_episodes_getGoal == 100){
            n_episodes_getGoal = 0;
            dist_maxGoal+=1;
            dis_min_goal_min = min(6.0, dist_maxGoal/2.0);
        }
        double dist_minGoal;
        if (false && rand()%10 >=7){
            dist_minGoal = 0.3;
        }
        else{
            dist_minGoal = dis_min_goal_min;
        }
        // cout << "GOAL RANGE: " << dist_minGoal << " - " << dist_maxGoal << ". N_12000: " << n_resets << " Steps: "<< n_iterations_reset << endl;
        while(!done){
            done = true;
            std::uniform_real_distribution<double>::param_type parX(std::max(xmin+1, x-dist_maxGoal), min(xmax-1, x+dist_minGoal));
            distributionX.param(parX);
            xg = distributionX(generator);
            std::uniform_real_distribution<double>::param_type parY(std::max(ymin+1, y-dist_maxGoal), min(ymax-1, y+dist_minGoal));
            distributionY.param(parY);
            yg = distributionY(generator);
            if ((x - xg)*(x - xg) + (y - yg)*(y - yg) < dist_minGoal*dist_minGoal) {
                done = false;
            }
            // cout << "X: " << std::max(xmin+1, x-dist_maxGoal) << " - " << min(xmax-1, x+dist_minGoal) << endl;
            // cout << "Y: " << std::max(ymin+1, y-dist_maxGoal) << " - " << min(ymax-1, y+dist_minGoal) << endl;
        }
    }
    else{
        double dist_minGoal = 6.0;
        if (false && rand()%10 >=7){
            dist_minGoal = 0.3;
        }
        while(!done){
            done = true;
            std::uniform_real_distribution<double>::param_type parX(xmin+1, xmax-1);
            distributionX.param(parX);
            xg = distributionX(generator);
            std::uniform_real_distribution<double>::param_type parY(ymin+1, ymax-1);
            distributionY.param(parY);
            yg = distributionY(generator);
            if ((x - xg)*(x - xg) + (y - yg)*(y - yg) < dist_minGoal*dist_minGoal) {
                done = false;
            }
        }
    }
}

void ActiveAgentClass::getPosition (double& x, double& y, double& theta, std::vector<Tpf>& agents, bool active){
    bool done = false;

    while(!done){
        done = true;
        if (!active){
            //x
            std::uniform_real_distribution<double>::param_type parX(xmin+1, xmax-1);
            distributionX.param(parX);
            x = distributionX(generator);
            //y
            std::uniform_real_distribution<double>::param_type parY(ymin+1, ymax-1);
            distributionY.param(parY);
            y = distributionY(generator);
        }
        else{
            //x
            std::uniform_real_distribution<double>::param_type parX(xmax-2, xmax-1);
            distributionX.param(parX);
            x = distributionX(generator);
            //y
            std::uniform_real_distribution<double>::param_type parY(ymax-2, ymax-1);
            distributionY.param(parY);
            y = distributionY(generator);
        }

        for (auto it=agents.begin(); it!=agents.end(); ++it){
            if (CollisionObs(*it, Tpf(x, y), (0.2)*6)) {
            done = false;
            }
        }
    }
    agents.push_back(Tpf(x, y));
    //Theta
    double titamin_ini = -3.1416, titamax_ini = 3.1416;
    double titamin = titamin_ini; 
    double titamax = titamax_ini;

    if (y > ymax/3) {
        titamax =  -0.7;
    } else if (y < 0){
        titamin = 0.7;
        titamax = 1.57 + 0.7;
    }

    titamin = titamin_ini; titamax = titamax_ini;
    std::uniform_real_distribution<double>::param_type parTita(titamin, titamax);
    distributionTita.param(parTita);
    theta = distributionTita(generator);
}
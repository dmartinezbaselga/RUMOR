#include <rl_dovs/active_agent_dqn.h>

// bool ActiveAgentDQN::waitingForACK(){
//     return this->waiting_for_ack;
// }

// void ActiveAgentDQN::ackCallback(const std_msgs::String& s){
//     cout << "ACK RECEIVEEEEEEEEEEEEEEEEEEEEED" << endl;
//     cout << "ACK RECEIVEEEEEEEEEEEEEEEEEEEEED" << endl;
//     cout << "ACK RECEIVEEEEEEEEEEEEEEEEEEEEED" << endl;
//     cout << "ACK RECEIVEEEEEEEEEEEEEEEEEEEEED" << endl;
//     cout << "ACK RECEIVEEEEEEEEEEEEEEEEEEEEED" << endl;
//     ros::shutdown();
// }

void ActiveAgentDQN::positionChange(const geometry_msgs::PoseWithCovarianceStamped& msg){
    double w = msg.pose.pose.orientation.w;
    double z = msg.pose.pose.orientation.z;
    tf::Quaternion q(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    robot->SetLocalization(Tsc(msg.pose.pose.position.x, msg.pose.pose.position.y,yaw));
}  

void ActiveAgentDQN::setFinish(bool& flag, const end_flag type){
    flag = true;
    finished = true;
    // cout << "FINISHED: " << type << collision_f << endl;
    // if (learning){
    //     std::string path = ros::package::getPath("rl_dovs");
    //     writeQTable(qTable, path + "/data/outQ_" + to_string(GetId()) + ".txt");
    // }
    if (test){
        writeLearningStats(type, iteracion, this->getAcumReward());
        // cout << "After writing stats" << endl;
    }
    geometry_msgs::Twist msg_v;
    Velocidad v(0,0);
    msg_v.linear.x = v.v;
    msg_v.angular.z = v.w;
    pub.publish(msg_v);
    coll_received = false;
    // std_msgs::String msg;
    // msg.data = to_string(id);
    // end_msg.publish(msg);
    // if (this->id == 0){
    //     this->waiting_for_ack = true;
    // }
    // else{
    //     ros::shutdown();
    // }
}

// void ActiveAgentDQN::sendEndMessage(){
//     std_msgs::String msg;
//     msg.data = to_string(id);
//     end_msg.publish(msg);
// }

// void ActiveAgentDQN::robotNearbyColl(const nav_msgs::Odometry& msg){
//     cout << "FINISHEEEEEEEED COOOOOL" << endl;
//     coll_received = true;
// }

void ActiveAgentDQN::collisionCall(const std_msgs::UInt8& msg){
    // cout << "COLLISION RECEIVED!!!!!!!!!!!!!!" << endl;
    coll_received = true;
}

bool ActiveAgentDQN::isFinished(){
    return this->finished;
}

void ActiveAgentDQN::act(int action){
    Velocidad v;
    if (use_crowdnav_actions){
        v = crowdnavVelocities[action];
        // Velocidad v_aux = possibleVelocities[0];
        // double current_min = Distancia(v.v-possibleVelocities[0].v, v.w-possibleVelocities[0].w);
        // for (size_t i_vel = 1; i_vel < possibleVelocities.size(); i_vel++){
        //     if (Distancia(v.v-possibleVelocities[i_vel].v, v.w-possibleVelocities[i_vel].w) < current_min){
        //         current_min = Distancia(v.v-possibleVelocities[i_vel].v, v.w-possibleVelocities[i_vel].w);
        //         v_aux = possibleVelocities[i_vel];
        //     }
        // }
        // v = v_aux;
    }
    else{
        v = possibleVelocities[action];
    }
    this->robot->SetVelocity(v);
    geometry_msgs::Twist msg;
    msg.linear.x = v.v;
    msg.angular.z = v.w;
    if (test){
        linear_velocities_episode.push_back(v.v);
    }
    // cout << "Chosen vel: " << v.v << ", " << v.w << endl;
    pub.publish(msg);
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

void ActiveAgentDQN::act(double a1, double a2){
    Velocidad v;
    if (use_crowdnav_actions){
        v.v = vMax*a1;
        v.w = -wMax + 2*wMax*a2;
    }
    else{
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
    }
    this->robot->SetVelocity(v);
    geometry_msgs::Twist msg;
    msg.linear.x = v.v;
    msg.angular.z = v.w;
    if (test){
        linear_velocities_episode.push_back(v.v);
    }
    // cout << "Chosen vel: " << v.v << ", " << v.w << endl;
    pub.publish(msg);
}

bool ActiveAgentDQN::select_action_rules(rl_dovs::DQN_action_rules::Request& req, rl_dovs::DQN_action_rules::Response& res){
    this->robot->ModelAgentsFusion(&agents_before, time_step, lookAhead, timeHorizon, accConst);
    Velocidad v = this->robot->MotionStrategy(time_step, &agents_before, video, iteracion, graph, debug);
    int best_candidate = -1;
    double min_dist_vel = 9999;
    for (int i_vel = 0; i_vel < possibleVelocities.size(); i_vel++){
        // cout << Distancia(v.v-possibleVelocities[i_vel].v, v.w-possibleVelocities[i_vel].w) << endl;
        if (Distancia(v.v-possibleVelocities[i_vel].v, v.w-possibleVelocities[i_vel].w) <= min_dist_vel){
            best_candidate = i_vel;
            min_dist_vel = Distancia(v.v-possibleVelocities[i_vel].v, v.w-possibleVelocities[i_vel].w);
        }
    }
    // if (vel_found==-1){
    //     ROS_ERROR("No suitable velocity comparison, needs to be fixed");
    //     cout << "Velocity found: " << v.v << " - " << v.w << endl;
    //     for (int i_vel = 0; i_vel < possibleVelocities.size(); i_vel++){
    //         cout << "Candidate " << i_vel << ": " << possibleVelocities[i_vel].v << " - " << possibleVelocities[i_vel].w << endl;
    //     }
    //     return false;
    // }
    res.action = best_candidate;
    return true;
}

void ActiveAgentDQN::changeTestingScenario(const int n_actives, const int n_pasives, const int episodes, const bool visualize){
    ifstream ifs;
    if (orca_agents){
        ifs.open(ros::package::getPath("rl_dovs") + "/data/testing_scenarios_orca/" + to_string(n_actives) + "-" + to_string(n_pasives) + "/" + to_string(episodes+1) + ".dovs", ios::binary );
    }
    else{
        ifs.open(ros::package::getPath("rl_dovs") + "/data/testing_scenarios/" + to_string(n_actives) + "-" + to_string(n_pasives) + "/" + to_string(episodes+1) + ".dovs", ios::binary );
    }    
    if (learning){
        epsilon = std::max(0.01, epsilon-epsilon_discount);
    }
    std::vector<geometry_msgs::PoseWithCovarianceStamped> amcl_msgs_vec;
    stage_ros_dovs::teleport srv;
    std::vector<Tpf> targets_new;
    std::vector<double> x_vec, y_vec, z_vec, angle_vec, vx_vec, vy_vec, t_vec, t_ini_vec;
    for (int idx = 0; idx < n_actives; idx++){
        double x_new, y_new, theta_new, x_goal_new, y_goal_new;
        ifs.read(reinterpret_cast<char*>(&x_new), sizeof(x_new));
        ifs.read(reinterpret_cast<char*>(&y_new), sizeof(y_new));
        ifs.read(reinterpret_cast<char*>(&theta_new), sizeof(theta_new));
        ifs.read(reinterpret_cast<char*>(&x_goal_new), sizeof(x_goal_new));
        ifs.read(reinterpret_cast<char*>(&y_goal_new), sizeof(y_goal_new));
        geometry_msgs::PoseArray reset_pos_msg;
        geometry_msgs::Pose position_new, goal_new;
        position_new.position.x = x_new;
        position_new.position.y = y_new;
        position_new.orientation.z = theta_new;
        reset_pos_msg.poses.push_back(position_new);
        goal_new.position.x = x_goal_new;
        goal_new.position.y = y_goal_new;
        reset_pos_msg.poses.push_back(goal_new);
        geometry_msgs::PoseWithCovarianceStamped amcl_msg;
        amcl_msg.header.frame_id = "map";
        amcl_msg.header.stamp = ros::Time::now();
        amcl_msg.pose.pose.position.x = x_new;
        amcl_msg.pose.pose.position.y = y_new;
        tf::Quaternion q(0,0,theta_new);
        amcl_msg.pose.pose.orientation.w = q.getW();
        amcl_msg.pose.pose.orientation.x = q.getX();
        amcl_msg.pose.pose.orientation.y = q.getY();
        amcl_msg.pose.pose.orientation.z = q.getZ();
        amcl_msg.pose.covariance = {0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.006853892326654787};
        amcl_msgs_vec.push_back(amcl_msg);
        x_vec.push_back(x_new);
        y_vec.push_back(y_new);
        z_vec.push_back(0);
        angle_vec.push_back(theta_new);
        targets_new.push_back(Tpf(x_goal_new, y_goal_new));
        amcl_pub[idx].publish(amcl_msg);
        // cout << "Sent amcl random" << endl;
        reset_position_pub[idx].publish(reset_pos_msg);
    }  
    for (int idx = 0; idx < n_pasives; idx++){
        double x_new, y_new, theta_new, v_new, w_new, t_new, t_new_init;
        ifs.read(reinterpret_cast<char*>(&x_new), sizeof(x_new));
        ifs.read(reinterpret_cast<char*>(&y_new), sizeof(y_new));
        ifs.read(reinterpret_cast<char*>(&theta_new), sizeof(theta_new));
        ifs.read(reinterpret_cast<char*>(&v_new), sizeof(v_new));
        ifs.read(reinterpret_cast<char*>(&w_new), sizeof(w_new));
        x_vec.push_back(x_new);
        y_vec.push_back(y_new);
        z_vec.push_back(0);
        angle_vec.push_back(theta_new);
        if (orca_agents){
            ifs.read(reinterpret_cast<char*>(&t_new), sizeof(t_new));
            ifs.read(reinterpret_cast<char*>(&t_new_init), sizeof(t_new_init));
            vx_vec.push_back(v_new);
            vy_vec.push_back(w_new);
            t_vec.push_back(t_new);
            t_ini_vec.push_back(t_new_init);
        }
        else{
            geometry_msgs::Twist v_msg;
            v_msg.linear.x = v_new;
            v_msg.angular.z = w_new;
            new_vel_pub[idx].publish(v_msg);
        }
    }
    for (auto t:targets_new){
        x_vec.push_back(t.x);
        y_vec.push_back(t.y);
        z_vec.push_back(0);
        angle_vec.push_back(0);
    }  
    srv.request.x = x_vec;
    srv.request.y = y_vec;
    srv.request.z = z_vec;
    srv.request.angle = angle_vec;
    srv.request.vx = vx_vec;
    srv.request.vy = vy_vec;
    srv.request.t = t_vec;
    srv.request.t_ini = t_ini_vec;
    srv.request.stop_world = !visualize;
    // cout << "CALIING SERVICEEEEEEEEEEEEEEEEEE" << endl;
    if (!client_teleport.call(srv))
    {
        ROS_ERROR("Failed to call service teleport");
    }
    ros::spinOnce();
    for (int idx = 0; idx < n_actives; idx++){
        amcl_pub[idx].publish(amcl_msgs_vec[idx]);
    }
    ros::spinOnce();
    // cout << "new testing scenario released" << endl;
    coll_received = false;
}

void ActiveAgentDQN::teleportAgents(const int n_actives, const int n_pasives, const bool visualize, const bool finish_training){
    episodes++;
    // cout << "Finished episode " << episodes<< endl;
    if (!finish_training && test){
        // cout << "Test scenario" << endl;
        this->changeTestingScenario(n_actives, n_pasives, episodes, visualize);
    }
    // if (episodes < n_episodes && random_scenario){
    else if (!finish_training && random_scenario){
        if (learning){
            epsilon = std::max(0.05, epsilon-epsilon_discount);
        }
        stage_ros_dovs::teleport srv;
        std::vector<Tpf> agents_new, targets_new;
        std::vector<double> x_vec, y_vec, z_vec, angle_vec, vx_vec, vy_vec, t_vec, t_ini_vec;
        std::vector<geometry_msgs::PoseWithCovarianceStamped> amcl_pub_vec;
        std::vector<geometry_msgs::PoseArray> reset_pos_vec;        
        for (int idx = 0; idx < n_actives; idx++){
            double x_new, y_new, theta_new, x_goal_new, y_goal_new;
            this->getPosition(x_new, y_new, theta_new, agents_new, true);
            // if (this->crowdnav){
            //     theta_new = 0.0;
            // }
            this->getGoal(x_goal_new, y_goal_new, x_new, y_new, episodes, idx);
            geometry_msgs::PoseArray reset_pos_msg;
            geometry_msgs::Pose position_new, goal_new;
            position_new.position.x = x_new;
            position_new.position.y = y_new;
            position_new.orientation.z = theta_new;
            reset_pos_msg.poses.push_back(position_new);
            goal_new.position.x = x_goal_new;
            goal_new.position.y = y_goal_new;
            reset_pos_msg.poses.push_back(goal_new);
            geometry_msgs::PoseWithCovarianceStamped amcl_msg;
            amcl_msg.header.frame_id = "map";
            amcl_msg.header.stamp = ros::Time::now();
            amcl_msg.pose.pose.position.x = x_new;
            amcl_msg.pose.pose.position.y = y_new;
            tf::Quaternion q(0,0,theta_new);
            amcl_msg.pose.pose.orientation.w = q.getW();
            amcl_msg.pose.pose.orientation.x = q.getX();
            amcl_msg.pose.pose.orientation.y = q.getY();
            amcl_msg.pose.pose.orientation.z = q.getZ();
            amcl_msg.pose.covariance = {0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787};
            x_vec.push_back(x_new);
            y_vec.push_back(y_new);
            z_vec.push_back(0);
            angle_vec.push_back(theta_new);
            agents_new.push_back(Tpf(x_new, y_new));
            targets_new.push_back(Tpf(x_goal_new, y_goal_new));
            // amcl_pub[idx].publish(amcl_msg);
            // // cout << "Sent amcl random" << endl;
            // reset_position_pub[idx].publish(reset_pos_msg);            
            amcl_pub_vec.push_back(amcl_msg);
            reset_pos_vec.push_back(reset_pos_msg);
        }  
        for (int idx = 0; idx < n_pasives; idx++){
            double x_new, y_new, theta_new, v_new, w_new;
            this->getPosition(x_new, y_new, theta_new, agents_new);

            if (episodes < 1000 && this->curricular_obstacles && rand()%1000 >= episodes % 1000){
                x_new = 5+idx;
                y_new = 5+idx; 
            }
            else if (this->random_n_obstacles && rand()%100 >= 75){
                x_new = 5+idx;
                y_new = 5+idx;
            }
            this->getVelocities(v_new, w_new);
            x_vec.push_back(x_new);
            y_vec.push_back(y_new);
            z_vec.push_back(0);
            angle_vec.push_back(theta_new);
            agents_new.push_back(Tpf(x_new, y_new));
            if (orca_agents){
                vx_vec.push_back(v_new);
                vy_vec.push_back(w_new);
                std::uniform_real_distribution<double>::param_type parT(20, 60);
                distributionFixed.param(parT);
                double t_comp = distributionFixed(generator);
                t_vec.push_back(t_comp);    
                std::uniform_real_distribution<double>::param_type parTin(0, 500);
                distributionFixed.param(parTin);
                t_ini_vec.push_back(distributionFixed(generator));              
            }
            else{
                geometry_msgs::Twist v_msg;
                v_msg.linear.x = v_new;
                v_msg.angular.z = w_new;
                new_vel_pub[idx].publish(v_msg);
            }
        }
        for (auto t:targets_new){
            x_vec.push_back(t.x);
            y_vec.push_back(t.y);
            z_vec.push_back(0);
            angle_vec.push_back(0);
        }  
        srv.request.x = x_vec;
        srv.request.y = y_vec;
        srv.request.z = z_vec;
        srv.request.vx = vx_vec;
        srv.request.vy = vy_vec;
        srv.request.t = t_vec;
        srv.request.t_ini = t_ini_vec;
        srv.request.angle = angle_vec;
        srv.request.stop_world = !visualize;
        // cout << "CALIING SERVICEEEEEEEEEEEEEEEEEE" << endl;
        if (!client_teleport.call(srv))
        {
            ROS_ERROR("Failed to call service teleport");
            sleep(5);
        }
        ros::spinOnce();
        for (unsigned int i_active = 0; i_active<amcl_pub_vec.size(); i_active++){
            amcl_pub[i_active].publish(amcl_pub_vec[i_active]);
            reset_position_pub[i_active].publish(reset_pos_vec[i_active]);
        }
        ros::spinOnce();
        coll_received = false;
    }
    // else if (episodes < n_episodes && !random_scenario){
    else if (!finish_training && !random_scenario){
        if (learning){
            epsilon = std::max(0.05, epsilon-epsilon_discount);
        }
        // cout << "Reseting stage. N actives: " << n_actives << endl;
        for (int idx = 0; idx < n_actives; idx++){
            // cout << "Sending message to active" << endl;
            geometry_msgs::PoseArray reset_pos_msg;
            reset_position_pub[idx].publish(reset_pos_msg);
        }
        std_srvs::Empty srv;
        if (!client_reset.call(srv))
        {
            ROS_ERROR("Failed to call service teleport");
        }
        ros::spinOnce();
        coll_received = false;
    }
    else{
        // cout << "Sending shutdown to agents" << endl;
        if (learning){
            std::string path = ros::package::getPath("rl_dovs");
            writeQTable(qTable, path + "/data/outQ_" + to_string(GetId()) + ".txt");
        }
        for (int idx = 1; idx < (n_actives + n_pasives); idx ++){
            std_msgs::String msg;
            shutdown_agent_pub[idx-1].publish(msg);
        }
        if (deep_q_learning){
            // ros::ServiceClient client_end_training;
            // client_end_training = n.serviceClient<rl_dovs::DQN_restart_parameters>("/restart_learning_parameters");
            // rl_dovs::DQN_restart_parameters srv_DQN_restart_params;
            // cout << "CALLING END TRAINIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIING" << endl;
            // if (!client_end_training.call(srv_DQN_restart_params))
            // {
            //     ROS_ERROR("Failed to call service end_training");
            // }
            // else{
            //     cout << "SUCCESFULLY CALLEEEEEEEEEEEED" << endl;
            // }

        }
        // ros::shutdown();
    }
}

void ActiveAgentDQN::newIteration(const bool visualize, const bool finish_training){
    this->robot->SetVelocity(Velocidad(0, 0));
    geometry_msgs::Twist vel;
    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.angular.z = 0.0;
    this->last_vel_published = vel;
    this->teleportAgents(this->nActives, this->nPasives, visualize, finish_training);    
}

void ActiveAgentDQN::getGoalState(std::vector<float> goalState, double reward){
    goalState.clear();
    goalState.push_back(this->robot->GetV());
    goalState.push_back(this->distCurrentState);
    goalState.push_back(this->reward==-10);
    goalState.push_back(min_dist);
}

bool ActiveAgentDQN::handle_gym(rl_dovs::DQN_gym::Request  &req, rl_dovs::DQN_gym::Response &res){
    act(req.action);
    stage_ros_dovs::step_by_step srv;
    srv.request.pause = false;
    if ((ros::Time::now() - before_stop).toSec() > 0.21){
        cout << "Time slept: " << (ros::Time::now() - before_stop).toSec() << endl;
    }
    before_stop = ros::Time::now();
    while (!this->client_step_by_step.call(srv))
    {
        ROS_ERROR("Failed to call service step by step");
    }
    sleep_rate.sleep();
    srv.request.pause = true;
    while (!this->client_step_by_step.call(srv))
    {
        ROS_ERROR("Failed to call service step by step");
    }
    std::vector<float> state, goal_state;
    std::vector<int> goal_actions;
    double reward;
    bool done;
    ros::spinOnce();
    // cout << "HOLAAAAAAAAAAAAA" << endl;
    observe(state, goal_actions, reward, done);

    if (this->lidar_observation){
        // Not interested in obstacle information
        for (int i_state = 0; i_state<4; i_state++){
            state.pop_back();
        }
        goal_state.clear();
        // Robot and goal information
        for (int i_state = 0; i_state<4; i_state++){
            goal_state.push_back(state.back());
            state.pop_back();
        }
        goal_state.insert(goal_state.end(),lidar_measurements.begin(), lidar_measurements.end());
        state = goal_state;
    }
    // cout << "DONE: " << done << endl;
    getGoalState(goal_state, reward);
    res.state = state;
    res.goal_actions = goal_actions;
    res.reward = reward;
    res.done = done;
    res.reward_data = goal_state;
    return true;
}

bool ActiveAgentDQN::handle_reset(rl_dovs::DQN_gym_reset::Request& req, rl_dovs::DQN_gym_reset::Response& res){
    ROS_INFO("HANDLING RESET");
    this->newIteration(visualize, req.finish_training);
    // cout << "Episode REWARD: " << this->getAcumReward() << endl;
    this->resetAcumReward();
    stage_ros_dovs::step_by_step srv;
    srv.request.pause = false;
    // cout << "Before step by step" << endl;
    while (!this->client_step_by_step.call(srv))
    {
        ROS_ERROR("Failed to call service step by step");
    }
    // cout << "Before sleep" << endl;
    ros::Duration sleep_reset(0.1);
    // cout << "Calling sleep" << endl;
    sleep_reset.sleep();
    srv.request.pause = true;
    // cout << "End sleep" << endl;
    while (!this->client_step_by_step.call(srv))
    {
        ROS_ERROR("Failed to call service step by step");
    }
    // cout << "End step by step" << endl;
    std::vector<float> state, goal_state;
    std::vector<int> goal_actions;
    double reward;
    bool done;
    finished = false;
    ros::spinOnce();
    observe(state, goal_actions, reward, done, true);
    if (this->lidar_observation){
        // Not interested in obstacle information
        for (int i_state = 0; i_state<4; i_state++){
            state.pop_back();
        }
        goal_state.clear();
        // Robot and goal information
        for (int i_state = 0; i_state<4; i_state++){
            goal_state.push_back(state.back());
            state.pop_back();
        }
        goal_state.insert(goal_state.end(),lidar_measurements.begin(), lidar_measurements.end());
        state = goal_state;
    }
    // cout << "State size: " << state.size() << endl;
    getGoalState(goal_state, reward);
    res.state = state;
    res.reward_data = goal_state;
    // ROS_INFO("State size %d", state.size());
    res.goal_actions = goal_actions;
    if (req.finish_training){
        ros::shutdown();
    }
    return true;
}

bool ActiveAgentDQN::handle_gym_sac(rl_dovs::SAC_gym::Request  &req, rl_dovs::SAC_gym::Response &res){
    act(req.x1, req.x2);
    stage_ros_dovs::step_by_step srv;
    srv.request.pause = false;
    if ((ros::Time::now() - before_stop).toSec() > 0.21){
        cout << "Time slept: " << (ros::Time::now() - before_stop).toSec() << endl;
    }
    before_stop = ros::Time::now();
    while (!this->client_step_by_step.call(srv))
    {
        ROS_ERROR("Failed to call service step by step");
    }
    sleep_rate.sleep();
    srv.request.pause = true;
    while (!this->client_step_by_step.call(srv))
    {
        ROS_ERROR("Failed to call service step by step");
    }
    std::vector<float> state, goal_state;
    std::vector<int> goal_actions;
    double reward;
    bool done;
    ros::spinOnce();
    // cout << "HOLAAAAAAAAAAAAA" << endl;
    observe(state, goal_actions, reward, done);

    // cout << "DONE: " << done << endl;
    getGoalState(goal_state, reward);
    res.state = state;
    res.reward = reward;
    res.done = done;
    return true;
}

bool ActiveAgentDQN::handle_reset_sac(rl_dovs::SAC_gym_reset::Request& req, rl_dovs::SAC_gym_reset::Response& res){
    ROS_INFO("HANDLING RESET");
    this->newIteration(visualize, req.finish_training);
    // cout << "Episode REWARD: " << this->getAcumReward() << endl;
    this->resetAcumReward();
    stage_ros_dovs::step_by_step srv;
    srv.request.pause = false;
    // cout << "Before step by step" << endl;
    while (!this->client_step_by_step.call(srv))
    {
        ROS_ERROR("Failed to call service step by step");
    }
    // cout << "Before sleep" << endl;
    ros::Duration sleep_reset(0.1);
    // cout << "Calling sleep" << endl;
    sleep_reset.sleep();
    srv.request.pause = true;
    // cout << "End sleep" << endl;
    while (!this->client_step_by_step.call(srv))
    {
        ROS_ERROR("Failed to call service step by step");
    }
    // cout << "End step by step" << endl;
    std::vector<float> state, goal_state;
    std::vector<int> goal_actions;
    double reward;
    bool done;
    finished = false;
    ros::spinOnce();
    observe(state, goal_actions, reward, done, true);
    // cout << "State size: " << state.size() << endl;
    getGoalState(goal_state, reward);
    res.state = state;
    // ROS_INFO("State size %d", state.size());
    if (req.finish_training){
        ros::shutdown();
    }
    return true;
}

bool ActiveAgentDQN::handle_reset_lidar(rl_dovs::DQN_gym_reset_lidar::Request& req, rl_dovs::DQN_gym_reset_lidar::Response& res){
    ROS_INFO("HANDLING RESET");
    this->newIteration(visualize, req.finish_training);
    cout << "Episode REWARD: " << this->getAcumReward() << endl;
    this->resetAcumReward();
    stage_ros_dovs::step_by_step srv;
    srv.request.pause = false;
    cout << "Before step by step" << endl;
    while (!this->client_step_by_step.call(srv))
    {
        ROS_ERROR("Failed to call service step by step");
    }
    cout << "Before sleep" << endl;
    ros::Duration sleep_reset(0.1);
    cout << "Calling sleep" << endl;
    sleep_reset.sleep();
    srv.request.pause = true;
    cout << "End sleep" << endl;
    while (!this->client_step_by_step.call(srv))
    {
        ROS_ERROR("Failed to call service step by step");
    }
    cout << "End step by step" << endl;
    std::vector<double> state;
    std::vector<int> goal_actions;
    double reward;
    bool done;
    ros::spinOnce();
    observe_lidar(state, reward, done, true);
    // getGoalState(goal_state, reward);
    res.robot_state = state;
    res.lidar_measurements = this->lidar_measurements;
    cout << "Lidar measurements size: " << this->lidar_measurements.size() << endl;
    res.reward = reward;
    if (req.finish_training){
        ros::shutdown();
    }
    return true;
}

bool ActiveAgentDQN::handle_gym_lidar(rl_dovs::DQN_gym_lidar::Request &req, rl_dovs::DQN_gym_lidar::Response &res){
    geometry_msgs::Twist vel;
    vel.linear.x = req.v_x;
    vel.linear.y = req.v_y;
    vel.angular.z = req.v_th;
    act_lidar(vel);
    stage_ros_dovs::step_by_step srv;
    srv.request.pause = false;
    if ((ros::Time::now() - before_stop).toSec() > 0.21){
        cout << "Time slept: " << (ros::Time::now() - before_stop).toSec() << endl;
    }
    before_stop = ros::Time::now();
    while (!this->client_step_by_step.call(srv))
    {
        ROS_ERROR("Failed to call service step by step");
    }
    sleep_rate.sleep();
    srv.request.pause = true;
    while (!this->client_step_by_step.call(srv))
    {
        ROS_ERROR("Failed to call service step by step");
    }
    std::vector<double> state;
    std::vector<int> goal_actions;
    double reward;
    bool done;
    ros::spinOnce();
    observe_lidar(state, reward, done);
    // getGoalState(goal_state, reward);
    res.robot_state = state;
    res.lidar_measurements = this->lidar_measurements;
    res.reward = reward;
    res.done = done;
    return true;
}

bool ActiveAgentDQN::handle_reset_crowdnav(rl_dovs::DQN_gym_reset_crowdnav::Request& req, rl_dovs::DQN_gym_reset_crowdnav::Response& res){
    ROS_INFO("HANDLING RESET");
    this->newIteration(visualize, req.finish_training);
    cout << "Episode REWARD: " << this->getAcumReward() << endl;
    this->resetAcumReward();
    stage_ros_dovs::step_by_step srv;
    srv.request.pause = false;
    cout << "Before step by step" << endl;
    while (!this->client_step_by_step.call(srv))
    {
        ROS_ERROR("Failed to call service step by step");
    }
    cout << "Before sleep" << endl;
    ros::Duration sleep_reset(0.1);
    cout << "Calling sleep" << endl;
    sleep_reset.sleep();
    srv.request.pause = true;
    cout << "End sleep" << endl;
    while (!this->client_step_by_step.call(srv))
    {
        ROS_ERROR("Failed to call service step by step");
    }
    cout << "End step by step" << endl;
    std::vector<double> state;
    std::vector<rl_dovs::FloatArray> observable_state;
    std::vector<int> goal_actions;
    double reward;
    bool done;
    ros::spinOnce();
    observe_crowdnav(state, observable_state, reward, done, true);
    // getGoalState(goal_state, reward);
    res.robot_state = state;
    res.observable_state = observable_state;
    // cout << "Lidar measurements size: " << this->lidar_measurements.size() << endl;
    res.reward = reward;
    if (req.finish_training){
        ros::shutdown();
    }
    return true;
}

bool ActiveAgentDQN::handle_gym_crowdnav(rl_dovs::DQN_gym_crowdnav::Request &req, rl_dovs::DQN_gym_crowdnav::Response &res){
    Velocidad v_aux = possibleVelocities[0];
    if (use_crowdnav_actions){
        v_aux.v = req.v_x;
        v_aux.w = req.v_th;
    }
    else{
        double current_min = Distancia(req.v_x-possibleVelocities[0].v, req.v_th-possibleVelocities[0].w);
        for (size_t i_vel = 1; i_vel < possibleVelocities.size(); i_vel++){
            if (Distancia(req.v_x-possibleVelocities[i_vel].v, req.v_th-possibleVelocities[i_vel].w) < current_min){
                current_min = Distancia(req.v_x-possibleVelocities[i_vel].v, req.v_th-possibleVelocities[i_vel].w);
                v_aux = possibleVelocities[i_vel];
            }
        }
    }
    this->robot->SetVelocity(v_aux);
    geometry_msgs::Twist vel;
    vel.linear.x = v_aux.v;
    vel.linear.y = req.v_y;
    vel.angular.z = v_aux.w;
    act_lidar(vel);
    stage_ros_dovs::step_by_step srv;
    srv.request.pause = false;
    if ((ros::Time::now() - before_stop).toSec() > 0.21){
        cout << "Time slept: " << (ros::Time::now() - before_stop).toSec() << endl;
    }
    before_stop = ros::Time::now();
    while (!this->client_step_by_step.call(srv))
    {
        ROS_ERROR("Failed to call service step by step");
    }
    sleep_rate.sleep();
    srv.request.pause = true;
    while (!this->client_step_by_step.call(srv))
    {
        ROS_ERROR("Failed to call service step by step");
    }
    std::vector<double> state;
    std::vector<rl_dovs::FloatArray> observable_state;
    std::vector<int> goal_actions;
    double reward;
    bool done;
    ros::spinOnce();
    observe_crowdnav(state, observable_state, reward, done);
    // getGoalState(goal_state, reward);
    res.robot_state = state;
    res.observable_state = observable_state;
    res.reward = reward;
    res.done = done;
    return true;
}

void ActiveAgentDQN::lidarCallback(const sensor_msgs::LaserScan& msg){
    this->lidar_measurements = msg.ranges;
}

void ActiveAgentDQN::odomCallback(const nav_msgs::Odometry& msg){
    this->vel_measurement = msg.twist.twist;
    this->pos_measurement = msg.pose.pose;
    tf::Quaternion q(
            pos_measurement.orientation.x,
            pos_measurement.orientation.y,
            pos_measurement.orientation.z,
            pos_measurement.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    robot->SetLocalization(Tsc(pos_measurement.position.x, pos_measurement.position.y, yaw));
}

void ActiveAgentDQN::getStateLidar (std::vector<double>& robot_state){
    Tpf goalAg;
    this->robot->GetCurrentGoal(goalAg);
    double goalX = goalAg.x; double goalY = goalAg.y;
    // Tsc loc = this->robot->GetLocalization();
    tf::Quaternion q(
            pos_measurement.orientation.x,
            pos_measurement.orientation.y,
            pos_measurement.orientation.z,
            pos_measurement.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    Tsc loc(pos_measurement.position.x, pos_measurement.position.y, yaw);
    robot->SetLocalization(loc);
    transfor_inversa_p(goalX, goalY, &loc, &goalAg);
    robot_state.push_back(goalAg.x);
    robot_state.push_back(goalAg.y);
    robot_state.push_back(this->vel_measurement.linear.x);
    robot_state.push_back(this->vel_measurement.linear.y);
    robot_state.push_back(this->vel_measurement.angular.z);
}

double NormalisePI(double d){
//Transforms the value into +/-pi range
    double offset = 1e-3*0.25/2;
    while (d > M_PI) d -= 2*M_PI;
    while (d < -M_PI) d += 2*M_PI;

    if (d>=-offset && d<=offset) return 0.0;
    else if (d>=M_PI-offset && d<=M_PI+offset) return M_PI;
    else if (d>=-(M_PI+offset) && d<=-(M_PI-offset)) return -M_PI;
    else if (d>=M_PI_2-offset && d<=M_PI_2+offset) return M_PI_2;
    else if (d>=-(M_PI_2+offset) && d<=-(M_PI_2-offset)) return -M_PI_2;

    return d;
}

void ActiveAgentDQN::getStateCrowdnav (std::vector<double>& robot_state){
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

    // Tpf goalAg;
    // this->robot->GetCurrentGoal(goalAg);
    // Tsc loc = this->robot->GetLocalization();    
    tf::Quaternion q(
        pos_measurement.orientation.x, 
        pos_measurement.orientation.y,
        pos_measurement.orientation.z,
        pos_measurement.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    Tsc loc2(pos_measurement.position.x, pos_measurement.position.y, yaw);
    robot->SetLocalization(loc2);
    robot_state.push_back(loc2.x);
    robot_state.push_back(loc2.y);
    robot_state.push_back(goalX);
    robot_state.push_back(goalY);
    robot_state.push_back(this->vel_measurement.linear.x);
    robot_state.push_back(this->vel_measurement.linear.y);
    robot_state.push_back(loc2.tita);
    robot_state.push_back(this->GetRealRadius());
    robot_state.push_back(this->vMax);
}

void ActiveAgentDQN::act_lidar(geometry_msgs::Twist& action){
    if (test){
        linear_velocities_episode.push_back(action.linear.x);
    }
    pub.publish(action);
    this->last_vel_published = action;
}

void ActiveAgentDQN::observe_crowdnav(std::vector<double>& robot_state, std::vector<rl_dovs::FloatArray>& observable_state, 
double& reward, bool& done, bool from_reset){
    robot_state.clear();
    if (!finished){
        this->robot->ModelAgentsFusion(&agents, time_step, lookAhead, timeHorizon, accConst);
        std::vector<int> goal_actions;
        possibleVelocities =  this->robot->isGoalInDW (time_step, &agents, 
                                        video, iteracion, graph, debug, goal_actions, true);
        this->getStateCrowdnav(robot_state);
        end_flag end_flag_iter;
        done = false;
        reward = getReward();
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
                success = true;
                end_flag_iter = success_f;
            }
            // if (robot_state.getDistanceToGoal() == 4) {
            //     reward = -10;
            // }
            if (coll_received){
                done = true;
                reward = -15;
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
        observable_state.clear();
        for (unsigned int i_agent = 0; i_agent < agents.size(); i_agent++){
            if (!agents[i_agent]->isSegment()){
                std::vector<double> observable_agent;
                observable_agent.clear();

                Tsc pos = agents[i_agent]->GetLocalization();
                observable_agent.push_back(pos.x);
                observable_agent.push_back(pos.y);
                observable_agent.push_back(agents[i_agent]->GetV());
                observable_agent.push_back(0.0);
                observable_agent.push_back(agents[i_agent]->GetRealRadius());
                observable_agent.push_back(pos.tita);
                rl_dovs::FloatArray float_array_msg;
                float_array_msg.array = observable_agent;
                observable_state.push_back(float_array_msg);
            }
            agents_before.push_back(std::move(agents[i_agent]));
        }
        agents.clear();
        iteracion++;
    }
}

void ActiveAgentDQN::observe_lidar(std::vector<double>& robot_state, double& reward, bool& done, bool from_reset){
    robot_state.clear();
    if (!finished){
        this->getStateLidar(robot_state);
        end_flag end_flag_iter;
        done = false;
        reward = getReward();
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
                success = true;
                end_flag_iter = success_f;
            }
            // if (robot_state.getDistanceToGoal() == 4) {
            //     reward = -10;
            // }
            if (coll_received){
                done = true;
                reward = -15;
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

/**
 * This example publish cmd_vel command with a fixed velocities.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "active_agent");

    int rate = 5;
    int id, nActives, nPasives, n_episodes;
    double v_x, v_th, x_goal, y_goal, av, aw, x, y, theta;
    bool learning, deep_q_learning, readQTable, test, graph, track_metric, record_steps, no_training, visualize, random_scenario,
            collaborative_scenario, lidar_state, crowdnav, curricular_obstacles, static_obstacles, 
            random_n_obstacles, cadrl, curricular_goal, global_scenario, rllib, orca_agents, use_crowdnav_actions,
            lidar_observation = false, sac;
    double alpha, gamma, epsilon, epsilon_discount;
    string crowdnav_policy = "", rllib_policy = "";
    string fileQTable, q_table_name;
    ros::param::get("~v_x", v_x); 
    ros::param::get("~v_th", v_th); 
    ros::param::get("~x_goal", x_goal); 
    ros::param::get("~y_goal", y_goal); 
    ros::param::get("~av", av); 
    ros::param::get("~aw", aw); 
    ros::param::get("~id", id); 
    ros::param::get("~learning", learning); 
    ros::param::get("~deep_q_learning", deep_q_learning);
    ros::param::get("~readQTable", readQTable); 
    ros::param::get("~epsilon", epsilon); 
    ros::param::get("~epsilon_discount", epsilon_discount); 
    epsilon_discount = 0.00025;
    ros::param::get("~test", test);
    ros::param::get("~alpha", alpha); 
    ros::param::get("~gamma", gamma); 
    ros::param::get("~fileQTable", fileQTable); 
    ros::param::get("~graph", graph); 
    ros::param::get("~track_metric", track_metric);
    ros::param::get("~q_table_name", q_table_name);
    ros::param::get("~record_steps", record_steps);
    ros::param::get("~no_training", no_training);
    ros::param::get("~nActives", nActives);
    ros::param::get("~nPasives", nPasives);
    ros::param::get("~n_episodes", n_episodes);
    ros::param::get("~visualize", visualize);
    ros::param::get("~random_scenario", random_scenario);
    ros::param::get("~x", x);
    ros::param::get("~y", y);
    ros::param::get("~theta", theta);
    ros::param::get("~collaborative", collaborative_scenario);
    ros::param::get("~lidar_state", lidar_state);
    ros::param::get("~crowdnav", crowdnav);
    ros::param::get("~curricular_obstacles", curricular_obstacles);
    ros::param::get("~curricular_goal", curricular_goal);
    ros::param::get("~static_obstacles", static_obstacles);
    ros::param::get("~random_n_obstacles", random_n_obstacles);
    ros::param::get("~cadrl", cadrl);
    ros::param::get("~crowdnav_policy", crowdnav_policy);
    ros::param::get("~global_scenario", global_scenario);
    ros::param::get("~rllib", rllib);
    ros::param::get("~rllib_policy", rllib_policy);
    ros::param::get("~orca_agents", orca_agents);
    ros::param::get("~use_crowdnav_actions", use_crowdnav_actions);
    ros::param::get("~lidar_observation", lidar_observation);
    ros::param::get("~sac", sac);
    

    v_x = 0;
    v_th = 0;

    bool manual = false;
    int lookAhead = 0;
    int timeHorizon = 0;
    bool accConst = false;
    int algorithm = 1;
    bool video = false;
    bool debug = false;
    // std::string path = ros::package::getPath("rl_dovs") + "/data/";
    // fileQTable = path + fileQTable;
    ActiveAgentDQN agent(id, v_x, v_th, x_goal, y_goal, av, aw, learning, deep_q_learning, graph, 1.0/rate, lookAhead,
    timeHorizon, accConst, algorithm, video, debug, readQTable, epsilon, fileQTable, manual, test, alpha, gamma,
    track_metric, q_table_name, record_steps, no_training, nActives, nPasives, n_episodes, epsilon_discount, random_scenario,
    x, y, theta, visualize, collaborative_scenario, lidar_state, crowdnav, curricular_obstacles, static_obstacles, random_n_obstacles,
    cadrl, crowdnav_policy, curricular_goal, global_scenario, rllib, rllib_policy, orca_agents, use_crowdnav_actions,
    lidar_observation, sac); 
    ros::Rate loop_rate(rate);
    ros::Time previous, current = ros::Time::now();
    ofstream f_error(ros::package::getPath("rl_dovs") + "/data/error_msgs/time_errors.txt", std::ios_base::app);
    stage_ros_dovs::step_by_step srv;
    srv.request.pause = true;
    while (!agent.client_step_by_step.call(srv))
    {
        ROS_ERROR("Failed to call service step by step");
    }
    ros::spin();





    // while (ros::ok()){
    //     // ros::spinOnce();
    //     if (id == 0 && !visualize){
    //         cout << "STOP" << endl;
    //         stage_ros_dovs::step_by_step srv;
    //         srv.request.pause = true;
    //         if (!agent.client_step_by_step.call(srv))
    //         {
    //             ROS_ERROR("Failed to call service step by step");
    //         }
    //     }
    //     ros::Time tstart = ros::Time::now();
    //     if (id == 0 && agent.isFinished()){
    //         agent.newIteration(visualize);
    //         cout << "Episode REWARD: " << agent.getAcumReward() << endl;
    //         agent.resetAcumReward();
    //     }
    //     agent.skip_iteration = false;
    //     previous = current;
    //     current = ros::Time::now();
    //     if ((current-previous).toSec() > 1.0/rate+0.02 && !agent.isFinished()){
    //         // agent.skip_iteration = true;
    //         // f_error << "WARNING: simulated time consumed: " << (current-previous).toSec() << endl;
    //         cout << "WARNING: simulated time consumed: " << (current-previous).toSec() << endl;
    //         // cout << ros::package::getPath("rl_dovs") + "/data/error_msgs/time_errors.txt" << endl;
    //     }
    //     if (!agent.isFinished()){
    //         if (id == 0 && track_metric){
    //             // agent.writeTrackerEstimation();
    //         }
    //         agent.publishVelocity();
    //         if (id == 0 && test){
    //             agent.writeMetrics();
    //         }
    //     }
    //     else{
    //         // agent.sendEndMessage();
    //     }
    //     // sleep(3);
    //     if (id == 0 && !visualize){
    //         stage_ros_dovs::step_by_step srv;
    //         srv.request.pause = false;
    //         if (!agent.client_step_by_step.call(srv))
    //         {
    //             ROS_ERROR("Failed to call service step by step");
    //         }
    //     }
    //     ros::spinOnce();
    //     // ros::Time tfinish = ros::Time::now();
    //     // double comp_time = (tfinish-tstart).toSec();
    //     // double rate = (1/(0.2-comp_time));
    //     // if (rate > 0 && rate < 100){
    //     //     ros::WallRate loop_rate(rate);
    //     //     loop_rate.sleep();
    //     // }
    //     loop_rate.sleep();
    // }
}

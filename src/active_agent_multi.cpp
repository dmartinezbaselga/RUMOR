#include <rl_dovs/active_agent_multi.h>

// bool ActiveAgentMulti::waitingForACK(){
//     return this->waiting_for_ack;
// }

// void ActiveAgentMulti::ackCallback(const std_msgs::String& s){
//     cout << "ACK RECEIVEEEEEEEEEEEEEEEEEEEEED" << endl;
//     cout << "ACK RECEIVEEEEEEEEEEEEEEEEEEEEED" << endl;
//     cout << "ACK RECEIVEEEEEEEEEEEEEEEEEEEEED" << endl;
//     cout << "ACK RECEIVEEEEEEEEEEEEEEEEEEEEED" << endl;
//     cout << "ACK RECEIVEEEEEEEEEEEEEEEEEEEEED" << endl;
//     ros::shutdown();
// }

// void ActiveAgentMulti::positionChange(const nav_msgs::Odometry& msg){
void ActiveAgentMulti::positionChange(const geometry_msgs::PoseWithCovarianceStamped& msg){
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
    robot->SetLocalization(Tsc(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw));
}  

void ActiveAgentMulti::setFinish(bool& flag, const end_flag type){
    flag = true;
    finished = true;
    new_iteration_global = true;

    // cout << "FINISHED: " << type << collision_f << endl;
    // if (learning){
    //     std::string path = ros::package::getPath("rl_dovs");
    //     writeQTable(qTable, path + "/data/outQ_" + to_string(GetId()) + ".txt");
    // }
    // cout << "HOLAAAAAAAAAAA" << test << endl;
    if (test){
        writeLearningStats(type, iteracion);
    }
    geometry_msgs::Twist msg_v;
    Velocidad v(0,0);
    msg_v.linear.x = v.v;
    msg_v.angular.z = v.w;
    pub.publish(msg_v);
    // coll_received = false;
    if (type == end_flag::unreachable_f){
        too_many_iterations = true;
    }
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

// void ActiveAgentMulti::sendEndMessage(){
//     std_msgs::String msg;
//     msg.data = to_string(id);
//     end_msg.publish(msg);
// }

// void ActiveAgentMulti::robotNearbyColl(const nav_msgs::Odometry& msg){
//     cout << "FINISHEEEEEEEED COOOOOL" << endl;
//     coll_received = true;
// }

void ActiveAgentMulti::collisionCall(const std_msgs::UInt8& msg){
    // cout << "COLLISION RECEIVED!!!!!!!!!!!!!!" << endl;
    coll_received = true;
}

bool ActiveAgentMulti::isFinished(){
    return this->finished;
}

void ActiveAgentMulti::changeTestingScenario(const int n_actives, const int n_pasives, const int episodes, const bool visualize){
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


void ActiveAgentMulti::teleportAgents(const int n_actives, const int n_pasives, const bool visualize){
    episodes++;
    // cout << "Finished episode " << episodes<< endl;
    bool finish_training = episodes >= n_episodes;
    cout << episodes << " episodes of " << n_episodes << endl;
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
        ros::shutdown();
    }
}

void ActiveAgentMulti::newIteration(const bool visualize){
    this->robot->SetVelocity(Velocidad(0, 0));
    sleep(1);
    this->teleportAgents(this->nActives, this->nPasives, visualize);  
    finished = false;
    coll_received = false;  
    too_many_iterations = false;
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
            two_lasers = false, collaborative_scenario, global_scenario, orca_agents, curricular_obstacles, curricular_goal, static_obstacles,
            random_n_obstacles, sac;
    double alpha, gamma, epsilon, epsilon_discount;
    // cout << "GRAPH FIRST "  << graph << endl;
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
    ros::param::get("~two_lasers", two_lasers);
    ros::param::get("~collaborative", collaborative_scenario);
    ros::param::get("~global_scenario", global_scenario);
    ros::param::get("~orca_agents", orca_agents);
    ros::param::get("~curricular_obstacles", curricular_obstacles);
    ros::param::get("~curricular_goal", curricular_goal);
    ros::param::get("~static_obstacles", static_obstacles);
    ros::param::get("~random_n_obstacles", random_n_obstacles);
    ros::param::get("~sac", sac);

    v_x = 0;
    v_th = 0;
    // cout << "GRAPH SECOND "  << graph << endl;

    bool manual = false;
    int lookAhead = 0;
    int timeHorizon = 0;
    bool accConst = false;
    int algorithm = 1;
    bool video = false;
    bool debug = false;
    bool test_agent_with_saved_scenarios = test;
    std::string path = ros::package::getPath("rl_dovs") + "/data/";
    fileQTable = path + fileQTable;
    ActiveAgentMulti agent(id, v_x, v_th, x_goal, y_goal, av, aw, learning, deep_q_learning, graph, 1.0/rate, lookAhead,
    timeHorizon, accConst, algorithm, video, debug, readQTable, epsilon, fileQTable, manual, test, alpha, gamma,
    track_metric, q_table_name, record_steps, no_training, nActives, nPasives, n_episodes, epsilon_discount, random_scenario,
    x, y, theta, test_agent_with_saved_scenarios, two_lasers, collaborative_scenario, global_scenario, orca_agents, curricular_obstacles, 
    curricular_goal, random_n_obstacles, static_obstacles, sac); 
    sleep(1);
    ros::Rate loop_rate(rate);
    ros::Time previous, current = ros::Time::now();
    ofstream f_error(ros::package::getPath("rl_dovs") + "/data/error_msgs/time_errors.txt", std::ios_base::app);
    while (ros::ok()){
        ros::spinOnce();
        if (id == 0 && !visualize){
            cout << "STOP" << endl;
            stage_ros_dovs::step_by_step srv;
            srv.request.pause = true;
            if (!agent.client_step_by_step.call(srv))
            {
                ROS_ERROR("Failed to call service step by step");
            }
        }
        ros::Time tstart = ros::Time::now();
        // cout << "is finished?" << agent.isFinished() << endl;
        if (id == 0 && agent.isFinished() && agent.startIterationGlobalPlanner()){
            agent.newIteration(visualize);
            cout << "Episode REWARD: " << agent.getAcumReward() << endl;
            agent.resetAcumReward();
        }
        agent.skip_iteration = false;
        previous = current;
        current = ros::Time::now();
        // cout << "WARNING: simulated time consumed: " << (current-previous).toSec() << endl;
        if ((current-previous).toSec() > 1.0/rate+0.02 && !agent.isFinished()){
            // agent.skip_iteration = true;
            // f_error << "WARNING: simulated time consumed: " << (current-previous).toSec() << endl;
            cout << "WARNING: simulated time consumed: " << (current-previous).toSec() << endl;
            // cout << ros::package::getPath("rl_dovs") + "/data/error_msgs/time_errors.txt" << endl;
        }
        if (!agent.isFinished()){
            // if (id == 0 && track_metric){
            //     // agent.writeTrackerEstimation();
            // }
            agent.publishVelocity();
            // if (id == 0 && test){
            //     agent.writeMetrics();
            // }
        }
        else{
            // agent.sendEndMessage();
        }
        // sleep(3);
        if (id == 0 && !visualize){
            stage_ros_dovs::step_by_step srv;
            srv.request.pause = false;
            if (!agent.client_step_by_step.call(srv))
            {
                ROS_ERROR("Failed to call service step by step");
            }
        }
        ros::spinOnce();
        // ros::Time tfinish = ros::Time::now();
        // double comp_time = (tfinish-tstart).toSec();
        // double rate = (1/(0.2-comp_time));
        // if (rate > 0 && rate < 100){
        //     ros::WallRate loop_rate(rate);
        //     loop_rate.sleep();
        // }
        loop_rate.sleep();
    }
}

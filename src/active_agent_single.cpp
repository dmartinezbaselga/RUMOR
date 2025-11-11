#include <rl_dovs/active_agent_single.h>

void ActiveAgentSingle::createVideo(int id_video) {
    if (video) {
        // system("ffmpeg -r 60 -f image2 -s 1920x1080 -i output/we%06d.png -vcodec libx264 -crf 25 -pix_fmt yuv420p we.mp4");
        system((string("ffmpeg -r 60 -f image2 -s 1920x1080 -i output/vs%06d.png -vcodec libx264 -crf 25 -pix_fmt yuv420p vs") + to_string(id_video) + string(".mp4")).c_str());
        // system("ffmpeg -i we.mp4 -i vs.mp4 -filter_complex hstack output.mp4");
        // system("rm we.mp4");
        //system("rm vs.mp4");
        // boost::filesystem::remove_all("output");
    }
}

void ActiveAgentSingle::positionChange(const geometry_msgs::PoseWithCovarianceStamped& msg){
    cout << "Pos change 1" << endl;
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
    int difx = msg.pose.pose.position.x - this->goal.x;
    int dify = msg.pose.pose.position.y - this->goal.y;
    if (index_goal!=-1 && sqrt(difx*difx - dify*dify)<0.1){
        index_goal++;
        if (index_goal >= current_path.poses.size()){
            index_goal = -1;
        }
        else{
            this->goal.x = current_path.poses[index_goal].pose.position.x;
            this->goal.y = current_path.poses[index_goal].pose.position.y;
            printf("goal received!!\n"); 
            this->robot->AddGoal(this->goal.x, this->goal.y);
        }
    }
    cout << "Pos change 2" << endl;
}      

void ActiveAgentSingle::fillPathRequest(nav_msgs::GetPlan::Request &request, float start_x, float start_y, float goal_x, float goal_y)
{
    request.start.header.frame_id ="map";
    request.start.pose.position.x = start_x;//initial position x coordinate
        request.start.pose.position.y = start_y;//initial position y coordinate
        request.start.pose.orientation.w = 1.0;//Orientation
    request.goal.header.frame_id = "map";
        request.goal.pose.position.x = goal_x;//End point coordinates
    request.goal.pose.position.y = goal_y;
    request.goal.pose.orientation.w = 1.0;
        request.tolerance = 0.0;//If the goal cannot be reached, the nearest available constraint
}

//Route planning result callback
void ActiveAgentSingle::callPlanningService(ros::ServiceClient &serviceClient, nav_msgs::GetPlan &srv)
{
    // Perform the actual path planner call
        //Execute the actual path planner
    if (serviceClient.call(srv)) {
                //srv.response.plan.poses is the container for storing the results, traversed and taken out
        if (!srv.response.plan.poses.empty()) {
//            std::for_each(srv.response.plan.poses.begin(),srv.response.plan.poses.end(),myfunction);
            ROS_INFO("make_plan success!");
        }
    }
    else {
        ROS_WARN("Got empty plan");
    }
}

void ActiveAgentSingle::setFinish(bool& flag, const end_flag type){
    flag = true;
    finished = true;
    cout << "FINISHED" << endl;
    if (learning){
        std::string path = ros::package::getPath("rl_dovs");
        writeQTable(qTable, path + "/data/outQ_" + to_string(GetId()) + ".txt");
    }
    if (test){
        writeLearningStats(type, iteracion);
    }
    // geometry_msgs::Twist msg_v;
    // Velocidad v(0,0);
    // msg_v.linear.x = v.v;
    // msg_v.angular.z = v.w;
    // pub.publish(msg_v);
    // std_msgs::String msg;
    // msg.data = to_string(id);
    // end_msg.publish(msg);
    // ros::shutdown();
}





/**
 * This example publish cmd_vel command with a fixed velocities.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "active_agent");

    int rate = 5;
    int id, nActives, nPasives;
    double v_x, v_th, x_goal, y_goal, av, aw;
    bool sac, learning, deep_q_learning, readQTable, test, graph, track_metric, record_steps, no_training, collaborative_scenario, orca_agents;
    double alpha, gamma, epsilon;
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
    ros::param::get("~test", test);
    ros::param::get("~alpha", alpha); 
    ros::param::get("~gamma", gamma); 
    ros::param::get("~fileQTable", fileQTable); 
    ros::param::get("~graph", graph); 
    ros::param::get("~track_metric", track_metric);
    ros::param::get("~q_table_name", q_table_name);
    ros::param::get("~record_steps", record_steps);
    ros::param::get("~no_training", no_training);
    ros::param::get("~collaborative", collaborative_scenario);
    ros::param::get("~nActives", nActives);
    ros::param::get("~nPasives", nPasives);
    ros::param::get("~sac", sac);

    // ros::param::get("~orca_agents", orca_agents);

    v_x = 0;
    v_th = 0;

    bool manual = false;
    int lookAhead = 0;
    int timeHorizon = 0;
    bool accConst = false;
    int algorithm = 1;
    bool video = true;
    bool debug = false;
    std::string path = ros::package::getPath("rl_dovs") + "/data/";
    fileQTable = path + fileQTable;

    ActiveAgentSingle agent(id, v_x, v_th, x_goal, y_goal, av, aw, learning, deep_q_learning, graph, 1.0/rate, lookAhead,
    timeHorizon, accConst, algorithm, video, debug, readQTable, epsilon, fileQTable, manual, test, alpha, gamma,
    track_metric, q_table_name, record_steps, no_training, collaborative_scenario, false, sac, nActives, nPasives); 
    ros::Rate loop_rate(rate);

    bool create_video = false;
    int id_video = 0;

    while (ros::ok()){
        // printf("hola\n");
        if (id == 0 && track_metric){
            agent.writeTrackerEstimation();
        }
        // printf("hey\n");
        agent.publishVelocity();
        if (!agent.isFinished()){
            create_video = true;
        }
        if (agent.isFinished() && create_video){
            agent.createVideo(id_video++);
            create_video = false;
        }
        if (id != 0 && track_metric){
            agent.writeMetrics();
        }
        // printf("um\n");
        ros::spinOnce();

        loop_rate.sleep();
    }
    if (video){
        agent.createVideo(id_video++);
    }
}

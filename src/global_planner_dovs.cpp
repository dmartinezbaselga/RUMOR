#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <rl_dovs/TData.h>
#include <obstacle_detector/Obstacles.h>
#include <rl_dovs/add_obstacle_map.h>
#include "rl_dovs/request_goal.h"


using namespace std;

double distance (double x, double y){
	return (std::sqrt(x*x+y*y));
}

class GlobalPlannerDOVS{
private:
  ros::Subscriber goal_sub, position_sub, obstacles_sub, velocity_sub;
  ros::Publisher path_pub, goal_pub;
  int index_goal = 0;
  ros::ServiceClient make_plan_client, add_obstacle_client;
  nav_msgs::Path current_path;
  bool finished = true;
  ros::NodeHandle n;
  double x, y, theta;
  bool request_goal = true;
  int first_request = 0, prev_request;
  bool velocity_low = false;
public:
  GlobalPlannerDOVS(){
    this->position_sub = n.subscribe("amcl_pose", 1, &GlobalPlannerDOVS::positionChange, this);
    this->velocity_sub = n.subscribe("odom", 1, &GlobalPlannerDOVS::velChange, this);
    this->goal_sub = n.subscribe("/rviz_goal", 1, &GlobalPlannerDOVS::rvizGoalCallback, this);     
    this->path_pub = n.advertise<nav_msgs::Path>("global_planner_path", 1);      
    this->goal_pub = n.advertise<geometry_msgs::PoseStamped>("dovs_goal", 1);      
    this->make_plan_client = n.serviceClient<nav_msgs::GetPlan>("move_base/make_plan",true); 
    this->add_obstacle_client = n.serviceClient<rl_dovs::add_obstacle_map>("/add_obstacle_to_map",true); 
    ros::param::get("~request_goal", request_goal);
    // this->obstacles_sub = n.subscribe("obstacles", 1, &GlobalPlannerDOVS::obstaclesCallback, this);
    if (request_goal){
      ros::ServiceClient client_goal_request;
      client_goal_request = n.serviceClient<rl_dovs::request_goal>("get_current_goal");
      rl_dovs::request_goal srv_goal;
      while (!client_goal_request.call(srv_goal))
      {
          ROS_ERROR("Failed to call request goal");
      }
      geometry_msgs::PoseStamped new_goal;
      new_goal.pose.position.x = srv_goal.response.x;
      new_goal.pose.position.y = srv_goal.response.y;
      rvizGoalCallback(new_goal);
    }
  }  
  
  void velChange(const nav_msgs::Odometry& msg){
    if (!finished && msg.twist.twist.linear.x < 0.1){
      if (!velocity_low){
        prev_request = first_request;
        rvizGoalCallback(this->current_path.poses[this->current_path.poses.size()-1]);
        first_request = prev_request;   
      }
      velocity_low = true;
    }
    else{
      velocity_low = false;
    }
  }

  void positionChange(const geometry_msgs::PoseWithCovarianceStamped& msg){
      this->x = msg.pose.pose.position.x;
      this->y = msg.pose.pose.position.y;
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
      this->theta = yaw;
      if (!finished){
        if (distance(this->x - this->current_path.poses[index_goal].pose.position.x, this->y - this->current_path.poses[index_goal].pose.position.y)<0.4){
          index_goal++;
          if (index_goal>=this->current_path.poses.size()){
            finished = true;
          }
          else{
            goal_pub.publish(current_path.poses[index_goal]);
          }
        }
        else {
          Line segment_line = Line(Tpf(this->current_path.poses[index_goal-1].pose.position.x, this->current_path.poses[index_goal-1].pose.position.y), 
                                  Tpf(this->current_path.poses[index_goal].pose.position.y, this->current_path.poses[index_goal].pose.position.y));
          if (segment_line.Distance(Tpf(this->x, this->y)) > 0.7){
            prev_request = first_request;
            rvizGoalCallback(this->current_path.poses[this->current_path.poses.size()-1]);
            first_request = prev_request;   
          }
        }
        cout << "FR: " << first_request << endl;
        if (first_request >= 0 && request_goal){
          prev_request = first_request;
          rvizGoalCallback(this->current_path.poses[this->current_path.poses.size()-1]);
          first_request = prev_request;   
          first_request++;     
          if (first_request >= 2){
            first_request = -1;
          }
          ROS_INFO("FIRST REQUEST\n");
        }
      }
  }  
  void callPlanningService(ros::ServiceClient &serviceClient, nav_msgs::GetPlan &srv)
  {
      // Perform the actual path planner call
          //Execute the actual path planner
      bool plan_received = false;
      while(!plan_received){
        if (serviceClient.call(srv)) {
                    //srv.response.plan.poses is the container for storing the results, traversed and taken out
            if (!srv.response.plan.poses.empty()) {
    //            std::for_each(srv.response.plan.poses.begin(),srv.response.plan.poses.end(),myfunction);
                ROS_INFO("make_plan success!");
                plan_received = true;
            }
            else {
                ROS_WARN("Got empty plan");
                ROS_INFO("Start x: %f, start y: %f\n", this->x, this->y);
                ros::Duration(1).sleep();
            }
        }
        else{
          ROS_WARN("Wrong service call");
          ros::Duration(1).sleep();
        }
        ros::spinOnce();
      }
  }
  void fillPathRequest(nav_msgs::GetPlan::Request &request, float start_x, float start_y, float goal_x, float goal_y)
  {
      request.start.header.frame_id ="map";
      request.start.header.stamp = ros::Time::now();
      request.goal.header.stamp = request.start.header.stamp;
      request.start.pose.position.x = start_x;//initial position x coordinate
          request.start.pose.position.y = start_y;//initial position y coordinate
          request.start.pose.orientation.w = theta;//Orientation
      request.goal.header.frame_id = "map";
          request.goal.pose.position.x = goal_x;//End point coordinates
      request.goal.pose.position.y = goal_y;
      request.goal.pose.orientation.w = theta;
      request.tolerance = 1.0;//If the goal cannot be reached, the nearest available constraint
  }
  void rvizGoalCallback(const geometry_msgs::PoseStamped& msg){
      // this->goal.x = msg.pose.position.x;
      // this->goal.y = msg.pose.position.y;
      // printf("goal received!!\n"); 
      // this->robot->AddGoal(this->goal.x, this->goal.y);

      nav_msgs::GetPlan srv;
      ROS_INFO("Start x: %f, start y: %f\n", this->x, this->y);
      fillPathRequest(srv.request,this->x,this->y,msg.pose.position.x,msg.pose.position.y);
      callPlanningService(make_plan_client,srv);
      this->current_path.poses.clear();
      nav_msgs::Path aux_path;
      aux_path.poses.clear();
      for (int i = 0; i<srv.response.plan.poses.size(); i+=20){
        aux_path.poses.push_back(srv.response.plan.poses[i]);
      }
      this->current_path.poses.push_back(aux_path.poses[0]);
      for (int i = 1; i<aux_path.poses.size()-1; i++){
        // double term_1 =aux_path.poses[i].pose.position.y -this->current_path.poses[this->current_path.poses.size()-1].pose.position.y;
        double term_1 =aux_path.poses[i].pose.position.y -aux_path.poses[i-1].pose.position.y;
        double term_2 =aux_path.poses[i+1].pose.position.x -aux_path.poses[i].pose.position.x;
        double term_3 =aux_path.poses[i+1].pose.position.y -aux_path.poses[i].pose.position.y;
        double term_4 =aux_path.poses[i].pose.position.x -aux_path.poses[i-1].pose.position.x;
        double diff = abs(term_1*term_2 - term_3*term_4);
        if (diff > 0.05){
          this->current_path.poses.push_back(aux_path.poses[i]);
        }
      }
      this->current_path.poses.push_back(srv.response.plan.poses.back());
      // this->current_path = srv.response.plan;
      this->current_path.header.frame_id = "map";
      srv.response.plan.header.frame_id = "map";
      aux_path.header.frame_id = "map";
      path_pub.publish(current_path);
      this-> index_goal = 1;
      goal_pub.publish(current_path.poses[index_goal]);
      // this->goal.x = current_path.poses[index_goal].pose.position.x;
      // this->goal.y = current_path.poses[index_goal].pose.position.y;
      // printf("goal received!!\n"); 
      // cout << goal.x << ", " << goal.y << ", " << current_path.poses.size() << endl;
      // this->robot->AddGoal(this->goal.x, this->goal.y);
      finished = false;
      first_request = 0;
  }

  // void obstaclesCallback(const obstacle_detector::Obstacles& msg){
  //   for (auto& obs : msg.circles){
  //     // ROS_INFO("Timestep: %d", obs.timesteps_static);
  //     if (obs.timesteps_static >= 20 && obs.timesteps_static <= 120){
  //       rl_dovs::add_obstacle_map srv;
  //       srv.request.obstacle = obs;
  //       while (!add_obstacle_client.call(srv)) {
  //         ROS_WARN("Wrong service call");
  //       }
  //       if (!finished){
  //         rvizGoalCallback(this->current_path.poses[this->current_path.poses.size()-1]);
  //       }
  //     }
  //   }
  // }
};

/**
 * This example publish cmd_vel command with a fixed velocities.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "global_planner");

  GlobalPlannerDOVS planner;

  ros::Rate loop_rate(5);

  while (ros::ok()){

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

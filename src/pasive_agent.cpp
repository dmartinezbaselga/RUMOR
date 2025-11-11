#include "ros/ros.h"
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <std_msgs/String.h>
#include <fstream>
#include <ros/package.h>
#include <rl_dovs/CollaborativeInfo.h>
#include <std_msgs/UInt8.h>


#include <iostream>
using namespace std;

const int NORMAL = 1, CHANGING = 2, WAIT_1 = 3, WAIT_2 = 10;

const double room_size = 8;

int change_dir = NORMAL;
int id;
double yaw_goal, v_x = 0.0, v_th = 0.0, v_save = 0.0, w_save = 0.0, x_pos = 0, y_pos = 0, x_inf, 
        x_sup, y_inf, y_sup;
bool coll_received = false;

void positionChange(const nav_msgs::Odometry& msg)
{	
    double x = msg.pose.pose.position.x;
    x_pos = x;
    double y = msg.pose.pose.position.y;
    y_pos = y;
    // printf("REAL: %f, %f\n", x, y);
    double w = msg.pose.pose.orientation.w;
    double z = msg.pose.pose.orientation.z;
    switch (change_dir)
    {
    case NORMAL:
    {
        if (x >= x_sup || x <= x_inf ||
            y >= y_sup || y <= y_inf){
            tf::Quaternion q(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            if (yaw>=0){
                yaw_goal = yaw - M_PI; 
            }
            else{
                yaw_goal = yaw + M_PI;
            }
            change_dir = CHANGING;
            v_x = 0.0;
            //v_th = 1.0472;
            v_th = 0.65;
        }
    }
    break;
    case CHANGING:
    {
        tf::Quaternion q(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        if (abs(yaw-yaw_goal) < 0.2){
            v_x = v_save;
            v_th = w_save;
            change_dir = WAIT_1;
        }
    }
    break;
    case WAIT_2:
        change_dir = NORMAL;
        break;
    default:
        change_dir += 1;
        break;
    }
}

void finishROS (const std_msgs::String& s){
    ros::shutdown();
}

void changeVelocity(const geometry_msgs::Twist& msg){
    change_dir = NORMAL;
    v_x = msg.linear.x;
    v_th = msg.angular.z;
    coll_received = false;
}

void collaborativePublish(ros::Publisher& pub_collaborative_position){
    rl_dovs::CollaborativeInfo msg;
    msg.radius = 0.2;
    msg.x = x_pos;
    msg.y = y_pos;
    msg.linear_vel = v_x;
    msg.angular_vel = v_th;
    msg.theta = yaw_goal;
    msg.id = id;
    pub_collaborative_position.publish(msg);
}

void collaborativePublishColl(ros::Publisher& pub_collaborative_position){
    rl_dovs::CollaborativeInfo msg;
    msg.radius = 0.2;
    msg.x = x_pos;
    msg.y = y_pos;
    msg.linear_vel = 0.0;
    msg.angular_vel = 0.0;
    msg.theta = yaw_goal;
    msg.id = id;
    pub_collaborative_position.publish(msg);
}

void collisionCall(const std_msgs::UInt8& msg){
    // cout << "COLLISION RECEIVED!!!!!!!!!!!!!!" << endl;
    coll_received = true;
}

void fill_x_y_limits(const double x_start, const double y_start){
    double central_point = round(x_start/room_size)*room_size;
    x_inf = central_point - (room_size/2 - 0.5);
    x_sup = central_point + (room_size/2 - 0.5);
    // cout << "Central x: " << x_inf << endl;
    central_point = round(y_start/room_size)*room_size;
    // cout << "Central y: " << x_inf << endl;
    y_inf = central_point - (room_size/2 - 0.5);
    y_sup = central_point + (room_size/2 - 0.5);    
    // cout << "X: " << x_inf << " to " << x_sup << " // Y: " << y_inf << " to " << y_sup << endl;
}

/**
 * This example publish cmd_vel command with a fixed velocities.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_agent");

    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Subscriber position_sub = n.subscribe("base_pose_ground_truth", 1, &positionChange);
    ros::Subscriber finish_msg = n.subscribe("shutdown_agent", 1, &finishROS);
    ros::Subscriber change_velocity = n.subscribe("velocity_change", 1, &changeVelocity);
    ros::Subscriber coll_sub = n.subscribe("collision", 1, collisionCall);

    bool track_metric = false, record_steps;
    bool collaborative_scenario = true;
    double x_start, y_start;
    
    ros::param::get("~v_x", v_x); 
    ros::param::get("~v_th", v_th); 
    ros::param::get("~x", x_start); 
    ros::param::get("~y", y_start); 
    x_pos = x_start;
    y_pos = y_start;
    ros::param::get("~track_metric", track_metric);
    ros::param::get("~id", id); 
    ros::param::get("~record_steps", record_steps);
    ros::param::get("~collaborative", collaborative_scenario);

    ros::Publisher pub_collaborative_position = n.advertise<rl_dovs::CollaborativeInfo>("/collaborative_info", 1);

    ofstream f_metrics, f_record_steps;
    // f_metrics.open(ros::package::getPath("rl_dovs") + "/data/metrics/metrics_" + to_string(id) + ".txt");
    if (record_steps){
        f_record_steps.open(ros::package::getPath("rl_dovs") + "/data/record_steps/output.txt" , std::ios_base::app);
    }

    v_save = v_x;
    w_save = v_th;

    ros::Rate loop_rate(10);
    int iteracion = 0;

    while (ros::ok()){
        // if (iteracion++ >= 1000){
		// 	ros::shutdown();
		// }
        fill_x_y_limits(x_pos, y_pos);
        geometry_msgs::Twist msg;
        // printf("REAL: %f\n", v_x);
        msg.linear.x = v_x;
        msg.angular.z = v_th;
        // f_metrics << ros::Time::now().toSec() << "," << x_pos << "," << y_pos << "," << v_x << "," << v_th << endl;
        if (record_steps){
            f_record_steps << "PASSIVE_" << id << "-" << ros::Time::now().toSec()<< ": X " << x_pos << " Y " << y_pos << " V " << v_x << " W " << v_th << endl;
        }

        pub.publish(msg);
        // cout << "V: " << msg.linear.x << " - W: " << msg.angular.z << endl;
        // cout << "X: " << x_inf << " to " << x_sup << " // Y: " << y_inf << " to " << y_sup << endl;
        if (collaborative_scenario && !coll_received){
            collaborativePublish(pub_collaborative_position);
        }
        else if (collaborative_scenario) {
            collaborativePublishColl(pub_collaborative_position);
        }
        coll_received = false;
        ros::spinOnce();

        loop_rate.sleep();
    }
}

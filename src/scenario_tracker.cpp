#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <bits/stdc++.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <stdint.h>
#include <std_msgs/String.h>
#include <ros/package.h>
#include <chrono>
#include <iomanip> // put_time
#include <sstream> // stringstream

using namespace std;

double Distancia (double x, double y){

	return (std::sqrt(x*x+y*y));
}

class PublishPositionsClass
{
	class RobotSubscriber
	{
		int id;
		nav_msgs::Odometry position;

	public:
		RobotSubscriber(int i):id(i) { 
			// cout << "Constructor: " << id << endl;
		}
			
		void positionChange(const nav_msgs::Odometry& msg)
		{	
			position = msg;
		}

		nav_msgs::Odometry getPosition(){
			return position;
		}
		int getId(){
			return id;
		}
	};

	ros::NodeHandle nh_;
	std::vector<RobotSubscriber*> robots;
	std::vector<ros::Subscriber> position_sub;
	ofstream f;

public:
	PublishPositionsClass(int nRob)
	{
		RobotSubscriber* robot = new RobotSubscriber(0);
		robots.push_back(robot);
		position_sub.push_back(nh_.subscribe("/robot_0/odom", 1, 
		&RobotSubscriber::positionChange, robot));
		for (int i = 0; i < nRob+1; i++)
		{
			RobotSubscriber* robot = new RobotSubscriber(i+1);
			robots.push_back(robot);
			position_sub.push_back(nh_.subscribe("/orca_" + to_string(i+1) + "/odom", 1, 
			&RobotSubscriber::positionChange, robot));
		}
		auto now = std::chrono::system_clock::now();
	    auto in_time_t = std::chrono::system_clock::to_time_t(now);
	    std::stringstream datetime;
	    datetime << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
		f.open(ros::package::getPath("rl_dovs") + "/data/saved_scenarios" + datetime.str() + ".txt", std::ofstream::out | std::ofstream::trunc);
	}

	~PublishPositionsClass() {}


	void writePositions(){
		for (int i = 0; i < robots.size();  i++){
			f << robots[i]->getPosition().pose.pose.position.x << "/" << robots[i]->getPosition().pose.pose.position.y << "/";
		}
		f<< endl;
		// f << robots[0]->getPosition().header.stamp.sec << "/" << robots[0]->getPosition().header.stamp.nsec << "/";
		// f << robots[0]->getPosition().twist.twist.linear.x << "/" << robots[0]->getPosition().twist.twist.angular.z << endl;
	}

};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "pos_pub");
	int nOrca = 10;
	// cout << "Index: " << (index_actives=="") << endl;
	PublishPositionsClass PP(nOrca);

    ros::Rate loop_rate(12);
	int iteracion = 0;

    while (ros::ok()){
		// PP.writePositions();
		ros::spinOnce();
		PP.writePositions();
		// if (iteracion++ >= 750){
		// 	// printf("TOO LONG\n");
		// 	// printf("TOO LONG\n");
		// 	// printf("TOO LONG\n");
		// 	// printf("TOO LONG\n");
		// 	// printf("TOO LONG\n");
		// 	// printf("TOO LONG\n");
		// 	// printf("TOO LONG\n");
		// 	std_msgs::String s;
		// 	PP.Finish(s);
		// }

		loop_rate.sleep();
    }
}

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

using namespace std;

double Distancia (double x, double y){

	return (std::sqrt(x*x+y*y));
}

class PublishPositionsClass
{
	class RobotSubscriber
	{
		int id;
		bool active;
		nav_msgs::Odometry position;
		bool finished;

	public:
		RobotSubscriber(int i):id(i) { 
			finished = false;
			// cout << "Constructor: " << id << endl;
		}

		bool isFinished(){
			return finished;
		}

		void setFinish(){
			finished = true;
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
	ros::Publisher test_msg_;
	ros::Publisher ack_msg;
	ros::Subscriber finish_msg;
	std::vector<ros::Publisher> send_finish;
	std::vector<ros::Publisher> position_pub;
	geometry_msgs::PoseStamped Goal;
	std::vector<RobotSubscriber*> robots;
	std::vector<ros::Subscriber> position_sub;
	int nRobots, nActives, nFinish;
	int indexActives[30];

public:
	PublishPositionsClass(int nRob, string& iActives)
	{
		nFinish = 0;
		nActives = 0;
		int pos = 0;
		if (iActives==""){
			// cout << "1 active" << endl;
			indexActives[0] = 0;
			nActives = 1;
			// position_pub.push_back(nh_.advertise<nav_msgs::Odometry>("/robot_0/near_robot_positions", 1));
		}
		else if (iActives != ""){
			while ((pos = iActives.find(" ")) != string::npos){
				int i =  stoi(iActives.substr(0, pos));
				indexActives[nActives] = i;
				nActives++;
				iActives = iActives.substr(pos+1, iActives.size());
				// position_pub.push_back(nh_.advertise<nav_msgs::Odometry>("/robot_" + to_string(i) + "/near_robot_positions", 1));
			}
			int i = stoi(iActives.substr(0, pos));
			indexActives[nActives] = i;
			// position_pub.push_back(nh_.advertise<nav_msgs::Odometry>("/robot_" + to_string(i) + "/near_robot_positions", 1));
			nActives++;
		}
		nRobots = nRob;
		for (int i = 0; i < nRobots; i++)
		{
			RobotSubscriber* robot = new RobotSubscriber(i);
			robots.push_back(robot);
			position_sub.push_back(nh_.subscribe("/robot_" + to_string(i) + "/base_pose_ground_truth", 1, 
			&RobotSubscriber::positionChange, robot));
			send_finish.push_back(nh_.advertise<std_msgs::String>("/robot_" + to_string(i) + "/shutdown_agent", 1));
		}
		// test_msg_ = nh_.advertise<geometry_msgs::PoseStamped>("/test/msg", 1);
		ack_msg = nh_.advertise<std_msgs::String>("/robot_0/ack", 1);
		finish_msg = nh_.subscribe("/shutdown", 1, &PublishPositionsClass::Finish, this);
	}

	~PublishPositionsClass() {}

	void Finish(const std_msgs::String& s){
		printf("FINISH RECIBIDOOOOOOOOO\n");
		printf("FINISH RECIBIDOOOOOOOOO\n");
		printf("FINISH RECIBIDOOOOOOOOO\n");
		printf("FINISH RECIBIDOOOOOOOOO\n");
		printf("FINISH RECIBIDOOOOOOOOO\n");
		printf("FINISH RECIBIDOOOOOOOOO\n");
		printf("FINISH RECIBIDOOOOOOOOO\n");
		nFinish++;
		if (nFinish == nActives || s.data == "0"){
			for (auto p: send_finish){
				p.publish(s);
			}
			ack_msg.publish(s);
			ros::shutdown();
		}
	}

	// void positionChange(const nav_msgs::Odometry &msg)
	// {
	// 	float x = msg.pose.pose.position.x;
	// 	float y = msg.pose.pose.position.y;
	// 	string id = msg.child_frame_id;
	// 	cout << id << ", " << x << ", " << y << endl;
	// 	//testMsg(x, y);
	// }

	// void writePositions(){
	// 	for (int i = 0; i < nActives;  i++){
	// 		if (!robots[indexActives[i]]->isFinished()){
	// 			nav_msgs::Odometry pos_act = robots[indexActives[i]]->getPosition();
	// 			if (abs(pos_act.pose.pose.position.x) > 10-0.4 || abs(pos_act.pose.pose.position.y) > 10-0.4){
	// 				cout << "FINISHED COLL POSE: " << pos_act.pose.pose.position.x << ", " << pos_act.pose.pose.position.y << endl;
	// 				position_pub[i].publish(pos_act);
	// 				robots[indexActives[i]]->setFinish();
	// 			}
	// 			else{
	// 				for (auto j : robots){
	// 					if(indexActives[i] != j->getId()){
	// 						nav_msgs::Odometry msg = j->getPosition();
	// 						double distancia = Distancia(msg.pose.pose.position.x-pos_act.pose.pose.position.x, 
	// 													msg.pose.pose.position.y-pos_act.pose.pose.position.y);
	// 						if (distancia < 0.45 && distancia != 0){
	// 							cout << "FINISHED COLL AG: " << Distancia(msg.pose.pose.position.x-pos_act.pose.pose.position.x, 
	// 									msg.pose.pose.position.y-pos_act.pose.pose.position.y) << " X1: " << pos_act.pose.pose.position.x 
	// 									<< " X2: " << msg.pose.pose.position.x << " Y1: " << pos_act.pose.pose.position.y << " Y2: " << 
	// 									msg.pose.pose.position.y << endl;
	// 							position_pub[i].publish(msg);
	// 							robots[indexActives[i]]->setFinish();
	// 						}
	// 					}
	// 				}	
	// 			}
	// 		}
	// 	}
	// }

	void testMsg(float x, float y)
	{
		Goal.pose.position.x = x;
		Goal.pose.position.y = y;
		cout << "Goal x: " << Goal.pose.position.x << endl;
		cout << "Goal y: " << Goal.pose.position.y << endl;
		test_msg_.publish(Goal);
		nav_msgs::Odometry m;
	}
};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "pos_pub");
	int nRobots;
	ros::param::get("~nRobots", nRobots); 
	string index_actives;
	ros::param::get("~actives", index_actives); 
	// cout << "Index: " << (index_actives=="") << endl;
	PublishPositionsClass PP(nRobots, index_actives);

    ros::Rate loop_rate(10);
	int iteracion = 0;

    while (ros::ok()){
		// PP.writePositions();
		ros::spinOnce();
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

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

/**
 * This example publish cmd_vel command with a fixed velocities.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_agent");

  ros::NodeHandle n;
  float v_x = 0.0;
  float v_th = 0.0;

  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  ros::param::get("~v_x", v_x); 
  ros::param::get("~v_th", v_th); 

  ros::Rate loop_rate(10);

  while (ros::ok()){

    geometry_msgs::Twist msg;
    msg.linear.x = v_x;
    msg.angular.z = v_th;

    pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

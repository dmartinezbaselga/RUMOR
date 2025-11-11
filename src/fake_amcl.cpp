#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "fake_amcl");

  ros::NodeHandle node;

  ros::Publisher publisher = 
    node.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 10);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/base_link", "/map",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    geometry_msgs::PoseWithCovarianceStamped msg;
    tf::quaternionTFToMsg(transform.getRotation(), msg.pose.pose.orientation);
    msg.pose.pose.position.x = transform.getOrigin().x();
    msg.pose.pose.position.y = transform.getOrigin().y();
    msg.pose.pose.position.z = transform.getOrigin().z();

    publisher.publish(msg);

    rate.sleep();
  }
  return 0;
};

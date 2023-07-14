#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

void filterVelocityCallback(const geometry_msgs::Twist& msg){

ROS_INFO_STREAM("Subscriber velocities:"<<" linear="<<msg.linear.x<<" angular="<<msg.angular.z);

}

int main(int argc, char **argv){
ros::init(argc, argv, "filter_velocity");
ros::NodeHandle nh;

ros::Subscriber sub = nh.subscribe("/cmd_vel",1000,&filterVelocityCallback);


ros::spin();
}

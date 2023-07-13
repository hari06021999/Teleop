/*********************************************************************************************************
*this node subscribes to (joy) topic from the TeleopeTurtle node (one of ROS example packages)          *
*then publish it as the standerd Twist message on topic (/cmd_vel),                                     * 
(this is the same topic of the teleope twist keboard)                                                   *
********************************************************************************************************/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <stdio.h>

class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};


TeleopTurtle::TeleopTurtle():
  linear_(1),
  angular_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
ROS_WARN("inside published");

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);
  
 // while(ros::ok())
 // {
 // geometry_msgs::Twist twist;
 // twist.angular.z = 0;//a_scale_*joy->axes[angular_]
 // twist.linear.x = 0;//l_scale_*joy->axes[linear_]
 // ROS_INFO("published");
 // vel_pub_.publish(twist);
  //sleep(1);
 // }

}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  
  geometry_msgs::Twist twist;
  twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];
  printf("Anglular:%lf ,Linear:%lf \n",twist.angular.z,twist.linear.x);
  vel_pub_.publish(twist);
 
  
 
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  ros::spin();
}

#include "cyrobot_pid/cyrobot_pid_core.h"

CyrobotPID::CyrobotPID()
{
}

CyrobotPID::~CyrobotPID()
{
}

void CyrobotPID::publishMessage(ros::Publisher *pub_message)
{
 cyrobot_msgs::PID msg;
  msg.p = p_;
  msg.d = d_;
  msg.i = i_;
  pub_message->publish(msg);
}

void CyrobotPID::messageCallback(const cyrobot_msgs::PID::ConstPtr &msg)
{
  p_ = msg->p;
  d_ = msg->d;
  i_ = msg->i;

  //echo P,I,D
  ROS_INFO("P: %f", p_);
  ROS_INFO("D: %f", d_);
  ROS_INFO("I: %f", i_);
}

void CyrobotPID::configCallback(cyrobot_pid::cyrobotPIDConfig &config, double level)
{
  //for PID GUI
  p_ = config.p;
  d_ = config.d;
  i_ = config.i;

}

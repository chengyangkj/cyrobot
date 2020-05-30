#ifndef SR_CYROBOT_PID_CORE_H
#define SR_CYROBOT_PID_CORE_H

#include "ros/ros.h"
#include "ros/time.h"

// Custom message includes. Auto-generated from msg/ directory.
#include <cyrobot_msgs/PID.h>

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <cyrobot_pid/cyrobotPIDConfig.h>

class CyrobotPID
{
public:
  CyrobotPID();
  ~CyrobotPID();
  void configCallback(cyrobot_pid::cyrobotPIDConfig &config, double level);
  void publishMessage(ros::Publisher *pub_message);
  void messageCallback(const cyrobot_msgs::PID::ConstPtr &msg);

  double p_;
  double d_;
  double i_;

};
#endif

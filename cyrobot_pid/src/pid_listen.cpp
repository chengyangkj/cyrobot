#include "cyrobot_pid/cyrobot_pid_core.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pid_listener");
  ros::NodeHandle nh;

  int rate;

  ros::NodeHandle pnh("~");
  pnh.param("rate", rate, int(40));

  CyrobotPID *cyrobot_pid = new CyrobotPID();

  ros::Subscriber sub_message = nh.subscribe("pid", 1000, &CyrobotPID::messageCallback, cyrobot_pid);

  ros::Rate r(rate);

  // Main loop.
  while (nh.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
} // end main()

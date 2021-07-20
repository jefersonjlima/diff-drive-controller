
#include "diff_drive_controller/pid_control.hpp"

Orientation test;

int main(int argc, char** argv)
{

  ros::init(argc, argv, "controller_node");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  PID_Control pid(node, private_nh);

  ros::spin();

  return 0;
}

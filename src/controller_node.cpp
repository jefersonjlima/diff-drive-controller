
#include "diff_drive_controller/pid_control.hpp"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller_node");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  PID_Control pid(node, private_nh);
  // reset simulation
  pid.resetGazebo();

  ros::Rate loop_rate(10); //10Hz

  do
  {
    ros::spinOnce();
    loop_rate.sleep();
  }while(ros::ok() && pid.moveTarget());


  return 0;
}

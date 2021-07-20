
#include "diff_drive_controller/pid_control.hpp"

PID_Control::PID_Control(ros::NodeHandle node, ros::NodeHandle private_nh)
{

  /*---Load Params------------------*/

  private_nh.param<std::string>("cmd_vel_publisher",   _cmd_vel_publisher,   "/cmd_vel");
  private_nh.param<std::string>("odometry_subscriber", _odometry_subscriber, "/odom");

  private_nh.param("pid_velocity_controller_gain/p", _pid.p, 5.5);
  private_nh.param("pid_velocity_controller_gain/i", _pid.i, 1.2);
  private_nh.param("pid_velocity_controller_gain/d", _pid.d, 0.5);
  private_nh.param("limits/linear/x/max_velocity", _pid.x_max_vel, 0.0);
  private_nh.param("limits/linear/z/max_velocity", _pid.z_max_vel, 0.0);
  private_nh.param("target_pose/x", _target.x, 1.0);
  private_nh.param("target_pose/y", _target.y, 1.0);

  /*---ROS Topics Init--------------*/

  odom_sub = node.subscribe(_odometry_subscriber, 1, &PID_Control::odomCallback, this);
  
  init();

  _node = node;

}

void PID_Control::init()
{
  ROS_INFO("Control Initialization!");
}

void PID_Control::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  double roll, pitch, yaw;
  tf::Quaternion quat;

  tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  _pose.x = msg->pose.pose.position.x;
  _pose.y = msg->pose.pose.position.y;
  _pose.yaw = yaw;
}

double PID_Control::calculateTargetDistance(const Orientation& pose, const Orientation& target)
{
  double dist;

  dist = sqrt((target.x - pose.x) + (target.y - pose.y));

  return dist;
}

double PID_Control::linearControl(const Orientation& pose)
{

  return 0;
}


double PID_Control::angularControl(const Orientation& pose)
{

  return 0;
}




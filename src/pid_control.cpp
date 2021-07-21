
#include "diff_drive_controller/pid_control.hpp"


PID_Control::PID_Control(ros::NodeHandle node, ros::NodeHandle private_nh)
{
  /*---Load Params------------------*/
  std::string model_path;
  private_nh.param<std::string>("cmd_vel_publisher",   _cmd_vel_publisher,   "/cmd_vel");
  private_nh.param<std::string>("odometry_subscriber", _odometry_subscriber, "/odom");
  private_nh.param<std::string>("goal_box", model_path, "");

#ifdef DEBUG
  ROS_INFO("Model Path: %s", model_path.c_str());
#endif

  private_nh.param("pid_velocity_controller_gain/linear/p", _pid.p_x, 1.5);
  private_nh.param("pid_velocity_controller_gain/linear/i", _pid.i_x, 0.0);
  private_nh.param("pid_velocity_controller_gain/linear/d", _pid.d_x, 0.0);
  private_nh.param("pid_velocity_controller_gain/angular/p", _pid.p_yaw, 6.0);
  private_nh.param("pid_velocity_controller_gain/angular/i", _pid.i_yaw, 0.0);
  private_nh.param("pid_velocity_controller_gain/angular/d", _pid.d_yaw, 0.0);

  private_nh.param("limits/linear/x/max_velocity", _pid.x_max_vel, DBL_MAX);
  private_nh.param("limits/angular/yaw/max_velocity", _pid.yaw_max_vel, DBL_MAX);
  private_nh.param("target_pose/x", _target.x, 1.0);
  private_nh.param("target_pose/y", _target.y, 1.0);

  /*---ROS Topics Init--------------*/
  _odom_sub = node.subscribe(_odometry_subscriber, 1, &PID_Control::odomCallback, this);
  _cmd_pub = node.advertise<geometry_msgs::Twist>(_cmd_vel_publisher, 10);
  ros::service::waitForService("gazebo/spawn_sdf_model");
  _spawn = node.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
  _dspawn = node.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");

  /*---Load Goal Marker-------------*/
  std::ifstream f_model(model_path);
  std::string model(std::istreambuf_iterator<char>(f_model),
      (std::istreambuf_iterator<char>()));
  _visual_target.request.model_xml = model;
  initGoal();

  _move = true;
  _node = node;

#ifdef DEBUG
  ROS_INFO("cmd_vel_publisher: %s", _cmd_vel_publisher.c_str());
  ROS_INFO("odometry_subscriberi: %s", _odometry_subscriber.c_str());

  ROS_INFO("pid_velocity_controller_gain/linear/p: %f", _pid.p_x);
  ROS_INFO("pid_velocity_controller_gain/linear/i: %f", _pid.i_x);
  ROS_INFO("pid_velocity_controller_gain/linear/d: %f", _pid.d_x);
  ROS_INFO("pid_velocity_controller_gain/angular/p: %f", _pid.p_yaw);
  ROS_INFO("pid_velocity_controller_gain/angular/i: %f", _pid.i_yaw);
  ROS_INFO("pid_velocity_controller_gain/angular/d: %f", _pid.d_yaw);

  ROS_INFO("limits/linear/x/max_velocity: %f", _pid.x_max_vel);
  ROS_INFO("limits/angular/yaw/max_velocity: %f", _pid.yaw_max_vel);
  ROS_INFO("target_pose/x: %f", _target.x);
  ROS_INFO("target_pose/y: %f", _target.y);
#endif
}

PID_Control::~PID_Control()
{
  geometry_msgs::Twist cmd_vel;

  // stop robot
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  _cmd_pub.publish(cmd_vel);

  // delete goal
  gazebo_msgs::DeleteModel deleteSrv;
  deleteSrv.request.model_name = "goal";

  bool result = false;
  _dspawn.call(deleteSrv);

  ROS_INFO("Closing Controller_node!");
  _node.shutdown();
}


void PID_Control::initGoal()
{
  geometry_msgs::Pose p;
  _visual_target.request.model_name = "goal";
  _visual_target.request.robot_namespace = "goal";
  p.position.x = _target.x;
  p.position.y = _target.y;
  p.position.z = 0.0;
  p.orientation = tf::createQuaternionMsgFromYaw(0.0);
  _visual_target.request.initial_pose = p;
  _visual_target.request.reference_frame = "";
  _spawn.call(_visual_target);

#ifdef DEBUG
  ROS_INFO("Control Initialization!");
#endif
}


void PID_Control::resetGazebo()
{
  std_srvs::Empty resetSrv;
  ros::service::call("/gazebo/reset_simulation", resetSrv);
  ros::service::call("/gazebo/reset_world", resetSrv);
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
  return sqrt((target.x - pose.x)*(target.x - pose.x) +
              (target.y - pose.y)*(target.y - pose.y));
}


double PID_Control::linearControl(const Orientation& pose, const Orientation& target)
{
  double distance, out_control;
  distance = calculateTargetDistance(_pose, _target);
  out_control = _pid.p_x * distance;
  if (out_control > _pid.x_max_vel)
  {
    out_control = _pid.x_max_vel;
  }
  return out_control;
}


double PID_Control::angularControl(const Orientation& pose, const Orientation& target)
{
  double goal_angle, out_control;

  goal_angle = atan2(target.y - pose.y, target.x - pose.x);

  out_control = _pid.p_yaw * (goal_angle - _pose.yaw);
  if (abs(out_control) > _pid.yaw_max_vel)
  {
    if (out_control < 0)
      out_control = -_pid.yaw_max_vel;
    else out_control = _pid.yaw_max_vel;
  }
#ifdef DEBUG
  ROS_INFO("Angle: %.2f, Out_Control: %.2f",goal_angle, out_control);
#endif
  return out_control;
}


bool PID_Control::moveTarget()
{
  geometry_msgs::Twist cmd_vel;
  auto target_tol = calculateTargetDistance(_pose, _target);

  if ((target_tol > 0.01))
  {
  cmd_vel.linear.x = linearControl(_pose, _target);
  cmd_vel.angular.z = angularControl(_pose, _target);

  _cmd_pub.publish(cmd_vel);
  }
  else
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      _cmd_pub.publish(cmd_vel);

      ROS_INFO("Finished!!!");
      _move = false;
    }
#ifdef DEBUG
  ROS_INFO("TB3 pose x:%.2f y:%.2f yaw:%.2f", _pose.x, _pose.y, _pose.yaw);
  auto ctrl_lin = linearControl(_pose, _target);
  auto ctrl_ang = angularControl(_pose, _target);
  ROS_INFO("Control Output: Linear %.2f,  Angular %.2f",ctrl_lin, ctrl_ang);
#endif
  return _move;
}

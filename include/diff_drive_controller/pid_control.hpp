#ifndef PID_CONTROL_HPP
#define PID_CONTROL_HPP
#define DEBUG

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <std_srvs/Empty.h>
#include <gazebo_msgs/SpawnModel.h>
#include <fstream>

typedef struct
{
  double p_x, i_x, d_x;
  double p_yaw, i_yaw, d_yaw;
  double x_max_vel, yaw_max_vel;
} PID;

typedef struct
{
  double x, y, yaw;

}Orientation;


class PID_Control{

  public:

    PID_Control(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~PID_Control(){ _node.shutdown();};

    void resetGazebo();
    double calculateTargetDistance(const Orientation& pose, const Orientation& target);
    double linearControl(const Orientation& pose, const Orientation& target);
    double angularControl(const Orientation& pose, const Orientation& target);
    void moveTarget();
  private:
    void initGoal();

    /*---ROS Topics-------------------*/
    ros::Publisher _cmd_pub;
    ros::Subscriber _odom_sub;
    ros::NodeHandle _node;
    ros::ServiceClient _spawn;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    /*---Variables--------------------*/
    std::string _cmd_vel_publisher;
    std::string _odometry_subscriber;
    gazebo_msgs::SpawnModel _visual_target;
    PID _pid;
    Orientation _pose;
    Orientation _target;
    bool _move;

};


#endif

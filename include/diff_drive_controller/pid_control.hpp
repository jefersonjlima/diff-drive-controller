#ifndef PID_CONTROL_HPP
#define PID_CONTROL_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>


typedef struct
{
  double p, i, d;
  double x_max_vel, z_max_vel;
} PID;

typedef struct
{
  double x, y, yaw;

}Orientation;


class PID_Control{

  public:

    PID_Control(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~PID_Control(){ _node.shutdown();};

     double calculateTargetDistance(const Orientation& pose, const Orientation& target);
     double linearControl(const Orientation& pose);
     double angularControl(const Orientation& pose);
  private:
    void init();

    /*---ROS Topics-------------------*/
    ros::Publisher cmd_pub;
    ros::Subscriber odom_sub;
    ros::NodeHandle _node;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    /*---Variables--------------------*/

    std::string _cmd_vel_publisher;
    std::string _odometry_subscriber;
    PID _pid;
    Orientation _pose;
    Orientation _target;

};


#endif

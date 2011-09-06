#ifndef GAZEBO_ROS_KURT_H
#define GAZEBO_ROS_KURT_H

#include <ros/ros.h>
#include <gazebo/Controller.hh>
#include <gazebo/Model.hh>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <sensor_msgs/JointState.h>

#include <tf/transform_broadcaster.h>

namespace gazebo
{
class GazeboRosKurt : public Controller
{
public:
  GazeboRosKurt(gazebo::Entity *parent);
  virtual ~GazeboRosKurt();

  virtual void LoadChild(XMLConfigNode *node);
  virtual void InitChild();
  virtual void FiniChild();
  virtual void UpdateChild();

private:
  static const size_t NUM_JOINTS = 6;

  void OnCmdVel(const geometry_msgs::TwistConstPtr &msg);

  ros::NodeHandle *rosnode_;

  ros::Publisher odom_pub_;
  ros::Publisher joint_state_pub_;

  ros::Subscriber cmd_vel_sub_;

  ParamT<std::string> *node_namespaceP_;
  std::vector<ParamT<std::string> *> joint_nameP_;

  /// Separation between the wheels
  ParamT<float> *wheel_sepP_;

  /// Diameter of the wheels
  ParamT<float> *wheel_diamP_;

  /// Torque applied to the wheels
  ParamT<float> *torqueP_;

  Model *my_parent_;

  /// Desired speeds of the wheels
  float wheel_speed_right_;
  float wheel_speed_left_;

  Joint *joints_[NUM_JOINTS];

  // Simulation time of the last update
  Time prev_update_time_;

  float odom_pose_[3];
  float odom_vel_[3];

  tf::TransformBroadcaster transform_broadcaster_;
  sensor_msgs::JointState js_;
};
}
#endif

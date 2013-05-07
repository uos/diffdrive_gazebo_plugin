#ifndef GAZEBO_ROS_KURT_H
#define GAZEBO_ROS_KURT_H

#include <ros/ros.h>
#include <common/Plugin.hh>
#include <common/Time.hh>
#include <common/Events.hh>
#include <physics/physics.hh>

#include <geometry_msgs/TwistWithCovariance.h>
#include <sensor_msgs/JointState.h>

#include <boost/thread.hpp>

namespace gazebo
{
class GazeboRosKurt : public ModelPlugin
{
public:
  GazeboRosKurt();
  virtual ~GazeboRosKurt();

  virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  virtual void UpdateChild();

private:
  static const size_t NUM_JOINTS = 6;
  static const double CMD_VEL_TIMEOUT = 0.6;

  void OnCmdVel(const geometry_msgs::TwistConstPtr &msg);

  ros::NodeHandle *rosnode_;

  ros::Publisher odom_pub_;
  ros::Publisher joint_state_pub_;

  ros::Subscriber cmd_vel_sub_;

  std::string node_namespace_;

  std::string cmd_vel_topic_name_;
  std::string odom_topic_name_;
  std::string joint_states_topic_name_;

  /// Separation between the wheels
  float wheel_sep_;

  /// Diameter of the wheels
  float wheel_diam_;

  /// Turning adaptation for odometry
  float turning_adaptation_;

  /// maximum torque applied to the wheels [Nm]
  float torque_;

  /// maximum forward speed of Kurt [m/s]
  float max_velocity_;

  physics::WorldPtr my_world_;
  physics::ModelPtr my_parent_;

  /// Desired speeds of the wheels
  float wheel_speed_right_;
  float wheel_speed_left_;

  physics::JointPtr joints_[NUM_JOINTS];

  // Simulation time of the last update
  common::Time prev_update_time_;

  // Simulation time when the last cmd_vel command was received (for timeout)
  common::Time last_cmd_vel_time_;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;

  float odom_pose_[3];
  float odom_vel_[3];

  sensor_msgs::JointState js_;

  void spin();
  boost::thread *spinner_thread_;

};
}
#endif

#include <kurt_gazebo_plugins/gazebo_ros_kurt.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <gazebo/Joint.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

#include <ros/time.h>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_kurt", GazeboRosKurt)

// index of left / right middle wheel joint
enum
{
  LEFT = 1, RIGHT = 4
};

GazeboRosKurt::GazeboRosKurt(Entity *parent) :
  Controller(parent)
{
  ros::MultiThreadedSpinner s(1);
  boost::thread spinner_thread(boost::bind(&ros::spin, s));

  my_parent_ = dynamic_cast<Model*> (parent);

  if (!my_parent_)
    gzthrow("Gazebo_ROS_Kurt controller requires a Model as its parent");

  Param::Begin(&this->parameters);
  node_namespaceP_ = new ParamT<std::string> ("node_namespace", "", 0);
  joint_nameP_.push_back(new ParamT<std::string> ("left_front_wheel_joint", "body_to_wheel_left_front", 1));
  joint_nameP_.push_back(new ParamT<std::string> ("left_middle_wheel_joint", "body_to_wheel_left_middle", 1));
  joint_nameP_.push_back(new ParamT<std::string> ("left_rear_wheel_joint", "body_to_wheel_left_rear", 1));
  joint_nameP_.push_back(new ParamT<std::string> ("right_front_wheel_joint", "body_to_wheel_right_front", 1));
  joint_nameP_.push_back(new ParamT<std::string> ("right_middle_wheel_joint", "body_to_wheel_right_middle", 1));
  joint_nameP_.push_back(new ParamT<std::string> ("right_rear_wheel_joint", "body_to_wheel_right_rear", 1));
  wheel_sepP_ = new ParamT<float> ("wheel_separation", 0.34, 1);
  wheel_diamP_ = new ParamT<float> ("wheel_diameter", 0.15, 1);
  torqueP_ = new ParamT<float> ("torque", 10.0, 1);
  Param::End();

  wheel_speed_right_ = 0.0;
  wheel_speed_left_ = 0.0;

  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    joints_[i] = NULL;
  }
}

GazeboRosKurt::~GazeboRosKurt()
{
  delete wheel_diamP_;
  delete wheel_sepP_;
  delete torqueP_;
  delete node_namespaceP_;

  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    delete joint_nameP_[i];
  }

  delete rosnode_;
}

void GazeboRosKurt::LoadChild(XMLConfigNode *node)
{
  node_namespaceP_->Load(node);
  wheel_sepP_->Load(node);
  wheel_diamP_->Load(node);
  torqueP_->Load(node);
  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    joint_nameP_[i]->Load(node);
    joints_[i] = my_parent_->GetJoint(**joint_nameP_[i]);
    if (!joints_[i])
      gzthrow("The controller couldn't get joint " << **joint_nameP_[i]);
  }

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_ros_kurt", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  }

  rosnode_ = new ros::NodeHandle(**node_namespaceP_);

  cmd_vel_sub_ = rosnode_->subscribe("cmd_vel", 1, &GazeboRosKurt::OnCmdVel, this);

  odom_pub_ = rosnode_->advertise<nav_msgs::Odometry> ("/odom", 1);

  joint_state_pub_ = rosnode_->advertise<sensor_msgs::JointState> ("/joint_states", 1);

  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    js_.name.push_back(**joint_nameP_[i]);
    js_.position.push_back(0);
    js_.velocity.push_back(0);
    js_.effort.push_back(0);
  }
}

void GazeboRosKurt::InitChild()
{
}

void GazeboRosKurt::FiniChild()
{
  rosnode_->shutdown();
}

void GazeboRosKurt::UpdateChild()
{
  double wd, ws;
  double d1, d2;
  double dr, da;
  Time step_time;

  wd = **(wheel_diamP_);
  ws = **(wheel_sepP_);

  d1 = d2 = 0;
  dr = da = 0;

  step_time = Simulator::Instance()->GetSimTime() - prev_update_time_;
  prev_update_time_ = Simulator::Instance()->GetSimTime();

  // Distance travelled by middle wheels
  d1 = step_time.Double() * (wd / 2) * joints_[LEFT]->GetVelocity(0);
  d2 = step_time.Double() * (wd / 2) * joints_[RIGHT]->GetVelocity(0);

  dr = (d1 + d2) / 2;
  da = (d2 - d1) / ws;

  // Compute odometric pose
  odom_pose_[0] += dr * cos(odom_pose_[2]);
  odom_pose_[1] += dr * sin(odom_pose_[2]);
  odom_pose_[2] += da;

  // Compute odometric instantaneous velocity
  odom_vel_[0] = dr / step_time.Double();
  odom_vel_[1] = 0.0;
  odom_vel_[2] = da / step_time.Double();

  // turn left wheels
  for (unsigned short i = 0; i < NUM_JOINTS/2; i++)
  {
    joints_[i]->SetVelocity(0, wheel_speed_left_ / (wd / 2.0));
    joints_[i]->SetMaxForce(0, **(torqueP_));
  }

  // turn right wheels
  for (unsigned short i = NUM_JOINTS/2; i < NUM_JOINTS; i++)
  {
    joints_[i]->SetVelocity(0, wheel_speed_right_ / (wd / 2.0));
    joints_[i]->SetMaxForce(0, **(torqueP_));
  }

  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";
  odom.pose.pose.position.x = odom_pose_[0];
  odom.pose.pose.position.y = odom_pose_[1];
  odom.pose.pose.position.z = 0;

  btQuaternion qt;
  qt.setRPY(0, 0, odom_pose_[2]);

  odom.pose.pose.orientation.x = qt.getX();
  odom.pose.pose.orientation.y = qt.getY();
  odom.pose.pose.orientation.z = qt.getZ();
  odom.pose.pose.orientation.w = qt.getW();

  double pose_cov[36] = { 1e-3, 0, 0, 0, 0, 0,
                          0, 1e-3, 0, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e3};

  memcpy(&odom.pose.covariance[0], pose_cov, sizeof(double) * 36);
  memcpy(&odom.twist.covariance[0], pose_cov, sizeof(double) * 36);

  odom.twist.twist.linear.x = 0;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.linear.z = 0;

  odom.twist.twist.angular.x = 0;
  odom.twist.twist.angular.y = 0;
  odom.twist.twist.angular.z = 0;

  odom_pub_.publish(odom);

  js_.header.stamp = ros::Time::now();

  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    js_.position[i] = joints_[i]->GetAngle(0).GetAsRadian();
    js_.velocity[i] = joints_[i]->GetVelocity(0);
  }

  joint_state_pub_.publish(js_);
}

void GazeboRosKurt::OnCmdVel(const geometry_msgs::TwistConstPtr &msg)
{
  double vr, va;
  vr = msg->linear.x;
  va = msg->angular.z;

  wheel_speed_left_ = vr - va * **(wheel_sepP_) / 2;
  wheel_speed_right_ = vr + va * **(wheel_sepP_) / 2;
}

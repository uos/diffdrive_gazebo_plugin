#include <diffdrive_gazebo_plugin/gazebo_ros_diffdrive_uos.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <ros/time.h>

#include <tf/transform_broadcaster.h>

using namespace gazebo;

// index of left / right middle wheel joint
enum
{
  LEFT = 1, RIGHT = 4
};

GazeboRosDiffdrive::GazeboRosDiffdrive() :
  wheel_speed_right_(0.0),
  wheel_speed_left_(0.0)
{
  this->spinner_thread_ = new boost::thread( boost::bind( &GazeboRosDiffdrive::spin, this) );

  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    joints_[i].reset();
  }
}

GazeboRosDiffdrive::~GazeboRosDiffdrive()
{
  rosnode_->shutdown();
  this->spinner_thread_->join();
  delete this->spinner_thread_;
  delete rosnode_;
}

void GazeboRosDiffdrive::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  this->my_world_ = _parent->GetWorld();

  this->my_parent_ = _parent;
  if (!this->my_parent_)
  {
    ROS_FATAL("Gazebo_ROS_Create controller requires a Model as its parent");
    return;
  }


  // this MUST be called 'robotNamespace' for proper remapping to work
  // TODO: could be 'node_namespace' now
  this->node_namespace_ = "/";
  if (_sdf->HasElement("robotNamespace"))
    this->node_namespace_ = _sdf->GetElement("robotNamespace")->GetValueString() + "/";


  cmd_vel_topic_name_ = "/cmd_vel";
  if (_sdf->HasElement("cmd_vel_topic_name"))
    cmd_vel_topic_name_ = _sdf->GetElement("cmd_vel_topic_name")->GetValueString();

  odom_topic_name_ = "/odom";
  if (_sdf->HasElement("odom_topic_name"))
    odom_topic_name_ = _sdf->GetElement("odom_topic_name")->GetValueString();

  joint_states_topic_name_ = "/joint_states";
  if (_sdf->HasElement("joint_states_topic_name"))
    joint_states_topic_name_ = _sdf->GetElement("joint_states_topic_name")->GetValueString();

  js_.name.resize(NUM_JOINTS);
  js_.position.resize(NUM_JOINTS);
  js_.velocity.resize(NUM_JOINTS);
  js_.effort.resize(NUM_JOINTS);

  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    js_.position[i] = 0;
    js_.velocity[i] = 0;
    js_.effort[i] = 0;
  }

  js_.name[0] = "left_front_wheel_joint";
  if (_sdf->HasElement("left_front_wheel_joint"))
    js_.name[0] = _sdf->GetElement("left_front_wheel_joint")->GetValueString();

  js_.name[1] = "left_middle_wheel_joint";
  if (_sdf->HasElement("left_middle_wheel_joint"))
    js_.name[1] = _sdf->GetElement("left_middle_wheel_joint")->GetValueString();

  js_.name[2] = "left_rear_wheel_joint";
  if (_sdf->HasElement("left_rear_wheel_joint"))
    js_.name[2] = _sdf->GetElement("left_rear_wheel_joint")->GetValueString();

  js_.name[3] = "right_front_wheel_joint";
  if (_sdf->HasElement("right_front_wheel_joint"))
    js_.name[3] = _sdf->GetElement("right_front_wheel_joint")->GetValueString();

  js_.name[4] = "right_middle_wheel_joint";
  if (_sdf->HasElement("right_middle_wheel_joint"))
    js_.name[5] = _sdf->GetElement("right_middle_wheel_joint")->GetValueString();

  js_.name[5] = "right_rear_wheel_joint";
  if (_sdf->HasElement("right_rear_wheel_joint"))
    js_.name[5] = _sdf->GetElement("right_rear_wheel_joint")->GetValueString();

  wheel_sep_ = 0.34;
  if (_sdf->HasElement("wheel_separation"))
    wheel_sep_ = _sdf->GetElement("wheel_separation")->GetValueDouble();

  turning_adaptation_ = 0.15;
  if (_sdf->HasElement("turning_adaptation"))
    turning_adaptation_ = _sdf->GetElement("turning_adaptation")->GetValueDouble();

  wheel_diam_ = 0.15;
  if (_sdf->HasElement("wheel_diameter"))
    wheel_diam_ = _sdf->GetElement("wheel_diameter")->GetValueDouble();

  torque_ = 4.0;
  if (_sdf->HasElement("torque"))
    torque_ = _sdf->GetElement("torque")->GetValueDouble();

  max_velocity_ = 4.0;
  if (_sdf->HasElement("max_velocity"))
    max_velocity_ = _sdf->GetElement("max_velocity")->GetValueDouble();


  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_ros_diffdrive_uos", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  rosnode_ = new ros::NodeHandle(node_namespace_);

  cmd_vel_sub_ = rosnode_->subscribe(cmd_vel_topic_name_, 1, &GazeboRosDiffdrive::OnCmdVel, this);
  odom_pub_ = rosnode_->advertise<nav_msgs::Odometry> (odom_topic_name_, 1);
  joint_state_pub_ = rosnode_->advertise<sensor_msgs::JointState> (joint_states_topic_name_, 1);

  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    joints_[i] = my_parent_->GetJoint(js_.name[i]);
    if (!joints_[i])
      gzthrow("The controller couldn't get joint " << js_.name[i]);
  }

  //initialize time and odometry position
  prev_update_time_ = last_cmd_vel_time_ = this->my_world_->GetSimTime();
  odom_pose_[0] = 0.0;
  odom_pose_[1] = 0.0;
  odom_pose_[2] = 0.0;

  // Get then name of the parent model
  std::string modelName = _sdf->GetParent()->GetValueString("name");

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateStart(
      boost::bind(&GazeboRosDiffdrive::UpdateChild, this));
  gzdbg << "plugin model name: " << modelName << "\n";

  ROS_INFO("gazebo_ros_diffdrive_uos plugin initialized");
}

void GazeboRosDiffdrive::UpdateChild()
{
  common::Time time_now = this->my_world_->GetSimTime();
  common::Time step_time = time_now - prev_update_time_;
  prev_update_time_ = time_now;

  double wd, ws;
  double d1, d2;
  double dr, da;
  double turning_adaptation;

  wd = wheel_diam_;
  ws = wheel_sep_;
  turning_adaptation = turning_adaptation_;

  d1 = d2 = 0;
  dr = da = 0;

  // Distance travelled by middle wheels
  d1 = step_time.Double() * (wd / 2) * joints_[LEFT]->GetVelocity(0);
  d2 = step_time.Double() * (wd / 2) * joints_[RIGHT]->GetVelocity(0);

  // Can see NaN values here, just zero them out if needed
  if (isnan(d1)) {
    ROS_WARN_THROTTLE(0.1, "gazebo_ros_diffdrive_uos: NaN in d1. Step time: %.2f. WD: %.2f. Velocity: %.2f", step_time.Double(), wd, joints_[LEFT]->GetVelocity(0));
    d1 = 0;
  }

  if (isnan(d2)) {
    ROS_WARN_THROTTLE(0.1, "gazebo_ros_diffdrive_uos: NaN in d2. Step time: %.2f. WD: %.2f. Velocity: %.2f", step_time.Double(), wd, joints_[RIGHT]->GetVelocity(0));
    d2 = 0;
  }

  dr = (d1 + d2) / 2;
  da = (d2 - d1) / ws * turning_adaptation;

  // Compute odometric pose
  odom_pose_[0] += dr * cos(odom_pose_[2]);
  odom_pose_[1] += dr * sin(odom_pose_[2]);
  odom_pose_[2] += da;

  // Compute odometric instantaneous velocity
  odom_vel_[0] = dr / step_time.Double();
  odom_vel_[1] = 0.0;
  odom_vel_[2] = da / step_time.Double();

  if (this->my_world_->GetSimTime() > last_cmd_vel_time_ + common::Time(CMD_VEL_TIMEOUT))
  {
	ROS_DEBUG("gazebo_ros_diffdrive_uos: cmd_vel timeout - current: %f, last cmd_vel: %f, timeout: %f", this->my_world_->GetSimTime().Double(), last_cmd_vel_time_.Double(), common::Time(CMD_VEL_TIMEOUT).Double());
	wheel_speed_left_ = wheel_speed_right_ = 0.0;
  }

  ROS_DEBUG("gazebo_ros_diffdrive_uos: setting wheel speeds (left; %f, right: %f)", wheel_speed_left_ / (wd / 2.0), wheel_speed_right_ / (wd / 2.0));

  // turn left wheels
  for (unsigned short i = 0; i < NUM_JOINTS/2; i++)
  {
    joints_[i]->SetVelocity(0, wheel_speed_left_ / (wd / 2.0));
    joints_[i]->SetMaxForce(0, torque_);
  }

  // turn right wheels
  for (unsigned short i = NUM_JOINTS/2; i < NUM_JOINTS; i++)
  {
    joints_[i]->SetVelocity(0, wheel_speed_right_ / (wd / 2.0));
    joints_[i]->SetMaxForce(0, torque_);
  }

  nav_msgs::Odometry odom;
  odom.header.stamp.sec = time_now.sec;
  odom.header.stamp.nsec = time_now.nsec;
  odom.header.frame_id = "odom_combined";
  odom.child_frame_id = "base_footprint";
  odom.pose.pose.position.x = odom_pose_[0];
  odom.pose.pose.position.y = odom_pose_[1];
  odom.pose.pose.position.z = 0;

  tf::Quaternion qt;
  qt.setEuler(0, 0, odom_pose_[2]);

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

  odom.twist.twist.linear.x = odom_vel_[0];
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.linear.z = 0;

  odom.twist.twist.angular.x = 0;
  odom.twist.twist.angular.y = 0;
  odom.twist.twist.angular.z = odom_vel_[2];

  odom_pub_.publish(odom);

  js_.header.stamp.sec = time_now.sec;
  js_.header.stamp.nsec = time_now.nsec;

  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    js_.position[i] = joints_[i]->GetAngle(0).GetAsRadian();
    js_.velocity[i] = joints_[i]->GetVelocity(0);
  }

  joint_state_pub_.publish(js_);
}

void GazeboRosDiffdrive::OnCmdVel(const geometry_msgs::TwistConstPtr &msg)
{
  last_cmd_vel_time_ = this->my_world_->GetSimTime();
  double vr, va;
  vr = msg->linear.x;
  va = msg->angular.z;

  wheel_speed_left_ = vr - va * wheel_sep_ / 2;
  wheel_speed_right_ = vr + va * wheel_sep_ / 2;

  // limit wheel speed
  if (fabs(wheel_speed_left_) > max_velocity_)
    wheel_speed_left_ = copysign(max_velocity_, wheel_speed_left_);
  if (fabs(wheel_speed_right_) > max_velocity_)
    wheel_speed_right_ = copysign(max_velocity_, wheel_speed_right_);
}

void GazeboRosDiffdrive::spin()
{
  while(ros::ok()) ros::spinOnce();
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosDiffdrive);

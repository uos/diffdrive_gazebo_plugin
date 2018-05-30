#include <diffdrive_gazebo_plugin/gazebo_ros_diffdrive_uos.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <ros/time.h>

#include <tf/transform_broadcaster.h>

using namespace gazebo;

const double gazebo::GazeboRosDiffdrive::CMD_VEL_TIMEOUT = 0.6;

GazeboRosDiffdrive::GazeboRosDiffdrive() :
  wheel_speed_right_(0.0),
  wheel_speed_left_(0.0)
{
  this->spinner_thread_ = new boost::thread( boost::bind( &GazeboRosDiffdrive::spin, this) );
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
    this->node_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";


  cmd_vel_topic_name_ = "/cmd_vel";
  if (_sdf->HasElement("cmd_vel_topic_name"))
    cmd_vel_topic_name_ = _sdf->GetElement("cmd_vel_topic_name")->Get<std::string>();

  odom_topic_name_ = "/odom";
  if (_sdf->HasElement("odom_topic_name"))
    odom_topic_name_ = _sdf->GetElement("odom_topic_name")->Get<std::string>();

  joint_states_topic_name_ = "/joint_states";
  if (_sdf->HasElement("joint_states_topic_name"))
    joint_states_topic_name_ = _sdf->GetElement("joint_states_topic_name")->Get<std::string>();

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

  enum
  {
    LF = 0, // left front wheel
    LM = 1, // left middle wheel
    LR = 2, // left rear wheel
    RF = 3, // right front wheel
    RM = 4, // right middle wheel
    RR = 5  // right rear wheel
  };

  std::vector<std::string> joint_names(6);

  joint_names[LF] = "left_front_wheel_joint";
  joint_names[LM] = "left_middle_wheel_joint";
  joint_names[LR] = "left_rear_wheel_joint";
  joint_names[RF] = "right_front_wheel_joint",
  joint_names[RM] = "right_middle_wheel_joint";
  joint_names[RR] = "right_rear_wheel_joint";

  std::vector<physics::JointPtr> joints_tmp(6);

  for(size_t i=0; i<joint_names.size(); i++)
  {
    // overwrite names, if configured in sdf
    if(_sdf->HasElement(joint_names[i]))
    {
      joint_names[i] = _sdf->GetElement(joint_names[i])->Get<std::string>();
    }

    // get all joint objects, some of them might not exist in the model
    joints_tmp[i] = my_parent_->GetJoint(joint_names[i]);
  }

  //check for availability

  std::vector<int> missing_joints;

  // left front, rear and right front and rear are required for a four wheel diffdrive robot
  if(!joints_tmp[LF]) missing_joints.push_back(LF);
  if(!joints_tmp[LR]) missing_joints.push_back(LR);
  if(!joints_tmp[RF]) missing_joints.push_back(RF);
  if(!joints_tmp[RR]) missing_joints.push_back(RR);

  // if one of the middle joints is available than the other should be available, too.
  if( !joints_tmp[LM] != !joints_tmp[RM] )
  {
    missing_joints.push_back(LM);
    missing_joints.push_back(RM);
  }

  if(!missing_joints.empty())
  {
    // build helpful error message with the missing joints
    std::string missing_err = "The controller couldn't get the joint(s): ";
    for(size_t i=0; i<missing_joints.size(); i++)
    {
      missing_err += joint_names[missing_joints[i]] + (i != missing_joints.size()-1 ? ", " : "");
    }
    gzthrow("This plugin supports either a four or a six wheeled robot, due to that the middle wheels are optional. "
                + missing_err);
  }

  // Determine the number of joints, the type of robot, either four wheeled or six wheeled.
  bool six_wheeled = joints_tmp[LM] && joints_tmp[RM];
  num_joints_ = six_wheeled ? 6 : 4;

  int six_wheels[] = {LF, LM, LR, RF, RM, RR};
  int four_wheels[] = {LF, LR, RF, RR};
  int* wheels = six_wheeled ? six_wheels : four_wheels;

  if ( six_wheeled ) ROS_INFO_STREAM("The robot is six wheeled.");
  else ROS_INFO_STREAM("The robot is four wheeled");

  left = six_wheeled ? 1 : 0;
  right = six_wheeled ? 4 : 2;

  js_.name.resize(num_joints_);
  js_.position.resize(num_joints_);
  js_.velocity.resize(num_joints_);
  js_.effort.resize(num_joints_);
  joints_.resize(num_joints_);

  for (size_t i = 0; i < num_joints_; ++i)
  {
    js_.position[i] = 0;
    js_.velocity[i] = 0;
    js_.effort[i] = 0;
    js_.name[i] = joint_names[wheels[i]];
    joints_[i] = joints_tmp[wheels[i]];
  }

  wheel_sep_ = 0.34;
  if (_sdf->HasElement("wheel_separation"))
    wheel_sep_ = _sdf->GetElement("wheel_separation")->Get<double>();

  turning_adaptation_ = 0.15;
  if (_sdf->HasElement("turning_adaptation"))
    turning_adaptation_ = _sdf->GetElement("turning_adaptation")->Get<double>();

  wheel_diam_ = 0.15;
  if (_sdf->HasElement("wheel_diameter"))
    wheel_diam_ = _sdf->GetElement("wheel_diameter")->Get<double>();

  torque_ = 4.0;
  if (_sdf->HasElement("torque"))
    torque_ = _sdf->GetElement("torque")->Get<double>();

  max_velocity_ = 4.0;
  if (_sdf->HasElement("max_velocity"))
    max_velocity_ = _sdf->GetElement("max_velocity")->Get<double>();

  //initialize time and odometry position
#if GAZEBO_MAJOR_VERSION > 8
  prev_update_time_ = last_cmd_vel_time_ = this->my_world_->SimTime();
#else
  prev_update_time_ = last_cmd_vel_time_ = this->my_world_->GetSimTime();
#endif
  odom_pose_[0] = 0.0;
  odom_pose_[1] = 0.0;
  odom_pose_[2] = 0.0;

  // Get then name of the parent model
  std::string modelName = _sdf->GetParent()->Get<std::string>("name");

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosDiffdrive::UpdateChild, this));
  gzdbg << "plugin model name: " << modelName << "\n";

  ROS_INFO_STREAM("Plugin gazebo_ros_diffdrive_uos_uos initialized.");
}

void GazeboRosDiffdrive::UpdateChild()
{
#if GAZEBO_MAJOR_VERSION > 8
  common::Time time_now = this->my_world_->SimTime();
#else
  common::Time time_now = this->my_world_->GetSimTime();
#endif
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

  // Distance travelled by middle (six wheeled robot) or by front wheels (four wheeled robot)
  d1 = step_time.Double() * (wd / 2) * joints_[left]->GetVelocity(0);
  d2 = step_time.Double() * (wd / 2) * joints_[right]->GetVelocity(0);

  // Can see NaN values here, just zero them out if needed
  if (std::isnan(d1)) {
    ROS_WARN_THROTTLE(0.1, "gazebo_ros_diffdrive_uos: NaN in d1. Step time: %.2f. WD: %.2f. Velocity: %.2f", step_time.Double(), wd, joints_[left]->GetVelocity(0));
    d1 = 0;
  }

  if (std::isnan(d2)) {
    ROS_WARN_THROTTLE(0.1, "gazebo_ros_diffdrive_uos: NaN in d2. Step time: %.2f. WD: %.2f. Velocity: %.2f", step_time.Double(), wd, joints_[right]->GetVelocity(0));
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

#if GAZEBO_MAJOR_VERSION > 8
  if (this->my_world_->SimTime() > last_cmd_vel_time_ + common::Time(CMD_VEL_TIMEOUT))
#else
  if (this->my_world_->GetSimTime() > last_cmd_vel_time_ + common::Time(CMD_VEL_TIMEOUT))
#endif
  {
#if GAZEBO_MAJOR_VERSION > 8
    ROS_DEBUG("gazebo_ros_diffdrive_uos: cmd_vel timeout - current: %f, last cmd_vel: %f, timeout: %f", this->my_world_->SimTime().Double(), last_cmd_vel_time_.Double(), common::Time(CMD_VEL_TIMEOUT).Double());
#else
    ROS_DEBUG("gazebo_ros_diffdrive_uos: cmd_vel timeout - current: %f, last cmd_vel: %f, timeout: %f", this->my_world_->GetSimTime().Double(), last_cmd_vel_time_.Double(), common::Time(CMD_VEL_TIMEOUT).Double());
#endif
    wheel_speed_left_ = wheel_speed_right_ = 0.0;
  }

  ROS_DEBUG("gazebo_ros_diffdrive_uos: setting wheel speeds (left; %f, right: %f)", wheel_speed_left_ / (wd / 2.0), wheel_speed_right_ / (wd / 2.0));

  // turn left wheels
  for (unsigned short i = 0; i < num_joints_/2; i++)
  {
    joints_[i]->SetVelocity(0, wheel_speed_left_ / (wd / 2.0));
#if GAZEBO_MAJOR_VERSION < 5
    joints_[i]->SetMaxForce(0, torque_);
#endif
  }

  // turn right wheels
  for (unsigned short i = num_joints_/2; i < num_joints_; i++)
  {
    joints_[i]->SetVelocity(0, wheel_speed_right_ / (wd / 2.0));
#if GAZEBO_MAJOR_VERSION < 5
    joints_[i]->SetMaxForce(0, torque_);
#endif
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

  for (size_t i = 0; i < num_joints_; ++i)
  {
#if GAZEBO_MAJOR_VERSION > 8
    js_.position[i] = joints_[i]->Position(0);
#else
    js_.position[i] = joints_[i]->GetAngle(0).Radian();
#endif
    js_.velocity[i] = joints_[i]->GetVelocity(0);
  }

  joint_state_pub_.publish(js_);
}

void GazeboRosDiffdrive::OnCmdVel(const geometry_msgs::TwistConstPtr &msg)
{
#if GAZEBO_MAJOR_VERSION > 8
  last_cmd_vel_time_ = this->my_world_->SimTime();
#else
  last_cmd_vel_time_ = this->my_world_->GetSimTime();
#endif
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

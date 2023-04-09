#include "flightros/pilot/flight_pilot.hpp"
#include <cv_bridge/cv_bridge.h>

namespace flightros {

FlightPilot::FlightPilot(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : nh_(nh),
    pnh_(pnh),
    scene_id_(UnityScene::INDUSTRIAL),
    unity_ready_(false),
    unity_render_(false),
    receive_id_(0),
    main_loop_freq_(50.0),
    initialized_(false) {
  // load parameters
  if (!loadParams()) {
    ROS_WARN("[%s] Could not load all parameters.",
             pnh_.getNamespace().c_str());
  } else {
    ROS_INFO("[%s] Loaded all parameters.", pnh_.getNamespace().c_str());
  }

  // quad initialization
  quad_ptr_ = std::make_shared<Quadrotor>();

  // add mono camera
  rgb_camera_ = std::make_shared<RGBCamera>();
  Vector<3> B_r_BC(0.0, 0.0, 0.3);
  Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  std::cout << R_BC << std::endl;
  rgb_camera_->setFOV(90);
  rgb_camera_->setWidth(640);
  rgb_camera_->setHeight(480);
  rgb_camera_->setRelPose(B_r_BC, R_BC);
  quad_ptr_->addRGBCamera(rgb_camera_);

  // initialization
  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);

  // initialize publisher
  setPublish();

  // initialize subscriber
  setSubscribe();
  
  // initialize Timer callback
  timer_main_loop_ = nh_.createTimer(ros::Rate(main_loop_freq_),
                                     &FlightPilot::mainLoopCallback, this);


  // wait until the gazebo and unity are loaded
  ros::Duration(5.0).sleep();

  // connect unity
  setUnity(unity_render_);
  connectUnity();
}

FlightPilot::~FlightPilot() {}

void FlightPilot::ImageCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  // gt
  if (!initialized_) {
    est_pose_pub_.publish(msg);
    // t2pose_.emplace(static_cast<float>(msg->header.stamp.toSec()), *msg);
    ROS_INFO("pub pose to autopilot, timestamp: %lf", msg->header.stamp.toSec());
  }

  // img
  quad_state_.x[QS::POSX] = (Scalar)msg->pose.pose.position.x;
  quad_state_.x[QS::POSY] = (Scalar)msg->pose.pose.position.y;
  quad_state_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  quad_state_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.y;
  quad_state_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;
  quad_ptr_->setState(quad_state_);

  if (unity_render_ && unity_ready_) {
    unity_bridge_ptr_->getRender(0);
    unity_bridge_ptr_->handleOutput();

    // publish image
    cv::Mat img;
    ros::Time timestamp = msg->header.stamp; //TODO: time synchronization
    rgb_camera_->getRGBImage(img);
    sensor_msgs::ImagePtr rgb_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    rgb_msg->header.stamp = timestamp;
    img_obs_pub_.publish(rgb_msg);

    // // collision
    // if (quad_ptr_->getCollision()) {
    //   ROS_INFO("COLLISION");
    // }
  }
}

void FlightPilot::IMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
{ 
  const sensor_msgs::Imu::ConstPtr& imu_gt = msg;

  // noise parameters

  // add noise
  sensor_msgs::Imu imu_obs;

  // publish
  imu_obs_pub_.publish(imu_obs);
  
  return;
}

void FlightPilot::EstPoseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if (initialized_) {
    // T_w = T_w_est * T_set
    Eigen::Quaterniond q_est(msg->pose.pose.orientation.w,
                             msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y,
                             msg->pose.pose.orientation.z);
    Eigen::Vector3d t_est(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Vector3d v_est(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    Eigen::Matrix3d R_est = q_est.toRotationMatrix();

    Eigen::Matrix3d R_w = R_w_est_ * R_est;
    Eigen::Vector3d t_w = R_w_est_ * t_est + t_w_est_;
    Eigen::Vector3d v_w = R_w_est_ * v_est; //TODO: check velocity is correct ?
    Eigen::Quaterniond q_w(R_w);

    // pub pose in world frame
    nav_msgs::Odometry pose_w;
    pose_w.header = msg->header;
    pose_w.header.frame_id = "world";  //TODO: check frame_id
    pose_w.pose.pose.position.x = t_w.x();
    pose_w.pose.pose.position.y = t_w.y();
    pose_w.pose.pose.position.z = t_w.z();
    pose_w.pose.pose.orientation.x = q_w.x();
    pose_w.pose.pose.orientation.y = q_w.y();
    pose_w.pose.pose.orientation.z = q_w.z();
    pose_w.pose.pose.orientation.w = q_w.w();
    pose_w.twist.twist.linear.x = v_w.x();
    pose_w.twist.twist.linear.y = v_w.y();
    pose_w.twist.twist.linear.z = v_w.z();
    est_pose_pub_.publish(pose_w);
  }

  return;
}

void FlightPilot::EstFailedCallback(const std_msgs::Empty::ConstPtr& msg)
{
  initialized_ = false;
  t2pose_.clear();

  ROS_WARN("Estimation Failed !!!");

  return;
}

void FlightPilot::EstInitializeSucceedCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
  if (initialized_) {
    ROS_ERROR("already initialised !");
    return;
  }

  // read world frame & est frame poses
  Eigen::Matrix<double, 3, Eigen::Dynamic> traj_gt;
  Eigen::Matrix<double, 3, Eigen::Dynamic> traj_est;

  int nums_of_poses = msg->channels[0].values.size();
  for (int i = 0; i < nums_of_poses; ++i) {
    float time = msg->channels[0].values[i]; // time
    if (!t2pose_.count(time)) {
      ROS_ERROR("cannot find match pose at time %f", time);
    }

    nav_msgs::Odometry& odom_gt = t2pose_[time];
    
    traj_gt(0, i) = odom_gt.pose.pose.position.x;
    traj_gt(1, i) = odom_gt.pose.pose.position.y;
    traj_gt(2, i) = odom_gt.pose.pose.position.z;
    traj_est(0, i) = msg->channels[1].values[i]; // x
    traj_est(1, i) = msg->channels[2].values[i]; // y
    traj_est(2, i) = msg->channels[3].values[i]; // z
  }

  // set T_w_est_ use Eigen::umeyama     TODO:: the rotation's freedom is strictly bounded ?
  Eigen::Matrix4d cRt = Eigen::umeyama(traj_gt, traj_est, true);
  Eigen::Matrix3d cR= cRt.topLeftCorner(3, 3);
  scale_ = pow(cR.determinant(), 1.0/3);
  R_w_est_ = (1 / scale_) * cR;
  t_w_est_ = cRt.topRightCorner(3, 1);
  
  ROS_INFO("set the T_w_est_ successfully !");
  initialized_ = true;

  return;
}

void FlightPilot::setPublish()
{
  // publisher for autopilot command
  pose_cmd_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("autopilot/pose_command", 1);
  velocity_cmd_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("autopilot/velocity_command", 1);
  start_cmd_pub_ = nh_.advertise<std_msgs::Empty>("autopilot/start", 1);
  hover_cmd_pub_ = nh_.advertise<std_msgs::Empty>("autopilot/force_hover", 1);
  land_cmd_pub_ = nh_.advertise<std_msgs::Empty>("autopilot/land", 1);
  off_cmd_pub_ = nh_.advertise<std_msgs::Empty>("autopilot/off", 1);

  // publisher for vio
  img_obs_pub_ = nh_.advertise<sensor_msgs::Image>("/cam0/image_raw", 1);
  imu_obs_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu0", 1);

  // publisher for autopilot & rotors estimated pose
  est_pose_pub_ = nh_.advertise<nav_msgs::Odometry>("flight_pilot/odometry", 1);

  return;
}

void FlightPilot::setSubscribe()
{
  // subscriber for img & imu, forward the img & imu to vins estimator
  sub_state_gt_ = nh_.subscribe("ground_truth/odometry", 1,
                                 &FlightPilot::ImageCallback, this);
  sub_imu_gt_ = nh_.subscribe("ground_truth/imu", 1,
                                 &FlightPilot::IMUCallback, this);

  // subscriber for gt & est pose, forward to autopilot
  sub_pose_est_ = nh_.subscribe("estimator/imu_propagate", 1, &FlightPilot::EstPoseCallback, this);

  // subscriber for estimator fail & initalize succeed signal
  sub_fail_est_ = nh_.subscribe("estimator/fail", 1, &FlightPilot::EstFailedCallback, this);
  sub_initialize_success_est_ = nh_.subscribe("estimator/initialize_success", 1, &FlightPilot::EstInitializeSucceedCallback, this);

  return;
}

void FlightPilot::mainLoopCallback(const ros::TimerEvent &event) {
  // pub start cmd
  if (!started_) {
    // std_msgs::Empty msg;
    // start_cmd_pub_.publish(msg);
    // ROS_INFO("pub start signal to autopilot");
    // started_ = true;

    return;
  }

  // 1. if not initialized, move around to initialize
  if (!initialized_) {

  }
  // 2. if initialized, pub desired way points
  else if (initialized_) {

  }
  return;
}

bool FlightPilot::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    ROS_INFO("[%s] Unity Bridge is created.", pnh_.getNamespace().c_str());
  }
  return true;
}

bool FlightPilot::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

bool FlightPilot::loadParams(void) {
  // load parameters
  quadrotor_common::getParam("main_loop_freq", main_loop_freq_, pnh_);
  quadrotor_common::getParam("unity_render", unity_render_, pnh_);

  return true;
}

}  // namespace flightros
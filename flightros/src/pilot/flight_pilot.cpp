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
    main_loop_freq_(50.0) {
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
  rgb_camera_->setWidth(720);
  rgb_camera_->setHeight(480);
  rgb_camera_->setRelPose(B_r_BC, R_BC);
  quad_ptr_->addRGBCamera(rgb_camera_);

  // initialization
  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);

  // initialize publisher
  setPublish();

  // initialize subscriber callbacks
  sub_state_gt_ = nh_.subscribe("ground_truth/odometry", 1,
                                 &FlightPilot::ImageCallback, this);
  sub_imu_gt_ = nh_.subscribe("ground_truth/imu", 1,
                                 &FlightPilot::IMUCallback, this);
  
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
    ros::Time timestamp = ros::Time::now(); //TODO: time synchronization
    rgb_camera_->getRGBImage(img);
    sensor_msgs::ImagePtr rgb_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    rgb_msg->header.stamp = timestamp;
    img_obs_pub_.publish(rgb_msg);

    // collision
    if (quad_ptr_->getCollision()) {
      ROS_INFO("COLLISION");
    }
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

void FlightPilot::setPublish()
{
  // publisher for autopilot
  pose_cmd_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("autopilot/pose_command", 1);
  velocity_cmd_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("autopilot/velocity_command", 1);
  start_cmd_pub_ = nh_.advertise<std_msgs::Empty>("autopilot/start", 1);
  hover_cmd_pub_ = nh_.advertise<std_msgs::Empty>("autopilot/force_hover", 1);
  land_cmd_pub_ = nh_.advertise<std_msgs::Empty>("autopilot/land", 1);
  off_cmd_pub_ = nh_.advertise<std_msgs::Empty>("autopilot/off", 1);

  // publisher for vio
  img_obs_pub_ = nh_.advertise<sensor_msgs::Image>("/cam0/image_raw", 1);
  imu_obs_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu0", 1);

  return;
}

void FlightPilot::mainLoopCallback(const ros::TimerEvent &event) {
  // publish way points
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
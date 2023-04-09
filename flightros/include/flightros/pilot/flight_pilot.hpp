
#pragma once

#include <memory>

// ros
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Empty.h>
#include <ros/ros.h>

// rpg quadrotor
#include <autopilot/autopilot_helper.h>
#include <autopilot/autopilot_states.h>
#include <quadrotor_common/parameter_helper.h>
#include <quadrotor_msgs/AutopilotFeedback.h>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

using namespace flightlib;

namespace flightros {

class FlightPilot {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FlightPilot(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~FlightPilot();

  // pulishers
  void setPublish();

  // subscribers
  void setSubscribe();

  // callbacks
  void mainLoopCallback(const ros::TimerEvent& event);
  void Image_GtPoseCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void EstPoseCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void EstFailedCallback(const std_msgs::Empty::ConstPtr& msg);
  void EstInitializeSucceedCallback(const sensor_msgs::PointCloud::ConstPtr& msg);

  bool setUnity(const bool render);
  bool connectUnity(void);
  bool loadParams(void);

  inline double rand1() { return (static_cast<double>(std::rand()) / RAND_MAX - 0.5) * 2; }

  bool initialized_;
  bool started_;

  Eigen::Matrix3d R_w_est_;
  Eigen::Vector3d t_w_est_;
  double scale_;
  std::map<float, nav_msgs::Odometry> t2pose_;

 private:
  // ros nodes
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // publisher
  ros::Publisher pose_cmd_pub_;
  ros::Publisher velocity_cmd_pub_;
  ros::Publisher start_cmd_pub_;
  ros::Publisher hover_cmd_pub_;
  ros::Publisher land_cmd_pub_;
  ros::Publisher off_cmd_pub_;

  ros::Publisher img_obs_pub_;
  ros::Publisher imu_obs_pub_;

  ros::Publisher est_pose_pub_;

  // subscriber
  ros::Subscriber sub_state_gt_;
  ros::Subscriber sub_imu_gt_;
  ros::Subscriber sub_pose_est_;
  ros::Subscriber sub_fail_est_;
  ros::Subscriber sub_initialize_success_est_;

  // main loop timer
  ros::Timer timer_main_loop_;

  // unity quadrotor
  std::shared_ptr<Quadrotor> quad_ptr_;
  std::shared_ptr<RGBCamera> rgb_camera_;
  QuadState quad_state_;

  // Flightmare(Unity3D)
  std::shared_ptr<UnityBridge> unity_bridge_ptr_;
  SceneID scene_id_{UnityScene::WAREHOUSE};
  bool unity_ready_{false};
  bool unity_render_{false};
  RenderMessage_t unity_output_;
  uint16_t receive_id_{0};

  // auxiliary variables
  Scalar main_loop_freq_{50.0};
};
}  // namespace flightros
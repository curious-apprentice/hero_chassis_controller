#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <hero_chassis_controller/HeroChassisControllerConfig.h>
#include <hero_chassis_controller/mecanum_kinematics.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/subscriber.h>
#include <tf/tfMessage.h>

namespace hero_chassis_controller {

class HeroChassisController
    : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  HeroChassisController() = default;
  ~HeroChassisController() override = default;

  bool init(hardware_interface::EffortJointInterface* hardware,
            ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

 private:
  struct CommandData {
    geometry_msgs::Twist twist;
    ros::Time received;
    bool valid{false};
  };

  void commandCallback(const geometry_msgs::TwistConstPtr& message);
  void reconfigureCallback(HeroChassisControllerConfig& config, std::uint32_t level);
  void publishOdometry(const ros::Time& time, const ChassisVelocity& velocity);

  std::array<hardware_interface::JointHandle, 4> wheel_joints_;
  std::array<control_toolbox::Pid, 4> wheel_pids_;
  std::vector<std::string> joint_names_;

  realtime_tools::RealtimeBuffer<CommandData> command_buffer_;
  std::unique_ptr<dynamic_reconfigure::Server<HeroChassisControllerConfig>> reconfigure_server_;
  ros::Subscriber command_subscriber_;
  std::unique_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odometry_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<tf::tfMessage>> transform_publisher_;

  std::string base_frame_id_{"base_link"};
  std::string odom_frame_id_{"odom"};
  double wheel_radius_{0.07625};
  double wheel_track_{0.4};
  double wheel_base_{0.4};
  double max_wheel_speed_{50.0};
  double max_effort_{5.0};
  double command_timeout_{0.5};

  ChassisVelocity odometry_pose_;
};

}  // namespace hero_chassis_controller

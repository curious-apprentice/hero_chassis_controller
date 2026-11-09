#include "hero_chassis_controller/hero_chassis_controller.h"

#include <algorithm>
#include <cmath>

#include <boost/bind/bind.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf/transform_datatypes.h>

namespace hero_chassis_controller {
namespace {
constexpr double kTwoPi = 6.28318530717958647692;
const std::array<const char*, 4> kDefaultJointNames{{
    "left_front_wheel_joint", "right_front_wheel_joint",
    "left_back_wheel_joint", "right_back_wheel_joint"}};

bool isFinite(double value) {
  return std::isfinite(value);
}
}  // namespace

bool HeroChassisController::init(hardware_interface::EffortJointInterface* hardware,
                                 ros::NodeHandle& root_nh,
                                 ros::NodeHandle& controller_nh) {
  if (hardware == nullptr) {
    ROS_ERROR("HeroChassisController received a null EffortJointInterface.");
    return false;
  }

  joint_names_.assign(kDefaultJointNames.begin(), kDefaultJointNames.end());
  controller_nh.getParam("joints", joint_names_);
  if (joint_names_.size() != wheel_joints_.size()) {
    ROS_ERROR_STREAM("Expected four wheel joint names, got " << joint_names_.size());
    return false;
  }

  try {
    for (std::size_t index = 0; index < wheel_joints_.size(); ++index) {
      wheel_joints_[index] = hardware->getHandle(joint_names_[index]);
    }
  } catch (const hardware_interface::HardwareInterfaceException& exception) {
    ROS_ERROR_STREAM("Failed to get a wheel joint handle: " << exception.what());
    return false;
  }

  controller_nh.param("wheel_radius", wheel_radius_, wheel_radius_);
  controller_nh.param("wheel_track", wheel_track_, wheel_track_);
  controller_nh.param("wheel_base", wheel_base_, wheel_base_);
  controller_nh.param("max_wheel_speed", max_wheel_speed_, max_wheel_speed_);
  controller_nh.param("max_effort", max_effort_, max_effort_);
  controller_nh.param("command_timeout", command_timeout_, command_timeout_);
  controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
  controller_nh.param("odom_frame_id", odom_frame_id_, odom_frame_id_);
  if (!isFinite(wheel_radius_) || !isFinite(wheel_track_) || !isFinite(wheel_base_) ||
      !isFinite(max_wheel_speed_) || !isFinite(max_effort_) ||
      !isFinite(command_timeout_) || wheel_radius_ <= 0.0 || wheel_track_ <= 0.0 ||
      wheel_base_ <= 0.0 || max_wheel_speed_ <= 0.0 || max_effort_ <= 0.0 ||
      command_timeout_ <= 0.0 || base_frame_id_.empty() || odom_frame_id_.empty()) {
    ROS_ERROR("Wheel geometry, limits, timeout, and frame IDs must be valid.");
    return false;
  }

  HeroChassisControllerConfig initial_config;
  reconfigure_server_.reset(
      new dynamic_reconfigure::Server<HeroChassisControllerConfig>(controller_nh));
  reconfigure_server_->getConfigDefault(initial_config);
  controller_nh.param("left_front_kp", initial_config.left_front_kp, initial_config.left_front_kp);
  controller_nh.param("right_front_kp", initial_config.right_front_kp, initial_config.right_front_kp);
  controller_nh.param("left_back_kp", initial_config.left_back_kp, initial_config.left_back_kp);
  controller_nh.param("right_back_kp", initial_config.right_back_kp, initial_config.right_back_kp);
  controller_nh.param("left_front_ki", initial_config.left_front_ki, initial_config.left_front_ki);
  controller_nh.param("right_front_ki", initial_config.right_front_ki, initial_config.right_front_ki);
  controller_nh.param("left_back_ki", initial_config.left_back_ki, initial_config.left_back_ki);
  controller_nh.param("right_back_ki", initial_config.right_back_ki, initial_config.right_back_ki);
  controller_nh.param("left_front_kd", initial_config.left_front_kd, initial_config.left_front_kd);
  controller_nh.param("right_front_kd", initial_config.right_front_kd, initial_config.right_front_kd);
  controller_nh.param("left_back_kd", initial_config.left_back_kd, initial_config.left_back_kd);
  controller_nh.param("right_back_kd", initial_config.right_back_kd, initial_config.right_back_kd);
  controller_nh.param("left_front_i_clamp", initial_config.left_front_i_clamp, initial_config.left_front_i_clamp);
  controller_nh.param("right_front_i_clamp", initial_config.right_front_i_clamp, initial_config.right_front_i_clamp);
  controller_nh.param("left_back_i_clamp", initial_config.left_back_i_clamp, initial_config.left_back_i_clamp);
  controller_nh.param("right_back_i_clamp", initial_config.right_back_i_clamp, initial_config.right_back_i_clamp);

  const std::array<double, 4> kp{{initial_config.left_front_kp, initial_config.right_front_kp,
                                  initial_config.left_back_kp, initial_config.right_back_kp}};
  const std::array<double, 4> ki{{initial_config.left_front_ki, initial_config.right_front_ki,
                                  initial_config.left_back_ki, initial_config.right_back_ki}};
  const std::array<double, 4> kd{{initial_config.left_front_kd, initial_config.right_front_kd,
                                  initial_config.left_back_kd, initial_config.right_back_kd}};
  const std::array<double, 4> i_clamp{{initial_config.left_front_i_clamp,
                                       initial_config.right_front_i_clamp,
                                       initial_config.left_back_i_clamp,
                                       initial_config.right_back_i_clamp}};
  for (std::size_t index = 0; index < wheel_pids_.size(); ++index) {
    const double integral_limit = std::abs(i_clamp[index]);
    wheel_pids_[index].initPid(kp[index], ki[index], kd[index],
                               integral_limit, -integral_limit, true);
  }
  reconfigure_server_->updateConfig(initial_config);

  command_buffer_.writeFromNonRT(CommandData{});
  command_subscriber_ = root_nh.subscribe(
      "/cmd_vel", 1, &HeroChassisController::commandCallback, this);

  odometry_publisher_.reset(
      new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(root_nh, "odom", 10));
  transform_publisher_.reset(
      new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 10));
  odometry_publisher_->msg_.header.frame_id = odom_frame_id_;
  odometry_publisher_->msg_.child_frame_id = base_frame_id_;
  transform_publisher_->msg_.transforms.resize(1);
  transform_publisher_->msg_.transforms[0].header.frame_id = odom_frame_id_;
  transform_publisher_->msg_.transforms[0].child_frame_id = base_frame_id_;

  dynamic_reconfigure::Server<HeroChassisControllerConfig>::CallbackType callback =
      boost::bind(&HeroChassisController::reconfigureCallback, this,
                  boost::placeholders::_1, boost::placeholders::_2);
  reconfigure_server_->setCallback(callback);

  ROS_INFO("HeroChassisController initialized with wheel PID, inverse kinematics, and odometry.");
  return true;
}

void HeroChassisController::starting(const ros::Time& /*time*/) {
  command_buffer_.writeFromNonRT(CommandData{});
  odometry_pose_ = ChassisVelocity{};
  for (auto& pid : wheel_pids_) {
    pid.reset();
  }
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period) {
  const double dt = period.toSec();
  if (!isFinite(dt) || dt <= 0.0) {
    for (auto& joint : wheel_joints_) {
      joint.setCommand(0.0);
    }
    return;
  }

  ChassisVelocity command_velocity;
  const CommandData* command = command_buffer_.readFromRT();
  if (command != nullptr && command->valid) {
    const double age = (time - command->received).toSec();
    if (isFinite(age) && age >= 0.0 && age <= command_timeout_) {
      command_velocity.x = command->twist.linear.x;
      command_velocity.y = command->twist.linear.y;
      command_velocity.yaw = command->twist.angular.z;
    }
  }

  std::array<double, 4> measured_wheel_speeds{};
  for (std::size_t index = 0; index < wheel_joints_.size(); ++index) {
    measured_wheel_speeds[index] = wheel_joints_[index].getVelocity();
    if (!isFinite(measured_wheel_speeds[index])) {
      measured_wheel_speeds[index] = 0.0;
    }
  }

  const std::array<double, 4> desired_wheel_speeds =
      inverseKinematics(command_velocity, wheel_radius_, wheel_track_, wheel_base_);
  for (std::size_t index = 0; index < wheel_joints_.size(); ++index) {
    const double desired = std::max(-max_wheel_speed_,
                                    std::min(max_wheel_speed_, desired_wheel_speeds[index]));
    const double error = desired - measured_wheel_speeds[index];
    const double effort = wheel_pids_[index].computeCommand(error, period);
    wheel_joints_[index].setCommand(
        isFinite(effort) ? std::max(-max_effort_, std::min(max_effort_, effort)) : 0.0);
  }

  const ChassisVelocity body_velocity =
      forwardKinematics(measured_wheel_speeds, wheel_radius_, wheel_track_, wheel_base_);
  const double midpoint_yaw = odometry_pose_.yaw + 0.5 * body_velocity.yaw * dt;
  odometry_pose_.x += (body_velocity.x * std::cos(midpoint_yaw) -
                       body_velocity.y * std::sin(midpoint_yaw)) * dt;
  odometry_pose_.y += (body_velocity.x * std::sin(midpoint_yaw) +
                       body_velocity.y * std::cos(midpoint_yaw)) * dt;
  odometry_pose_.yaw = std::remainder(odometry_pose_.yaw + body_velocity.yaw * dt, kTwoPi);
  publishOdometry(time, body_velocity);
}

void HeroChassisController::stopping(const ros::Time& /*time*/) {
  for (std::size_t index = 0; index < wheel_joints_.size(); ++index) {
    wheel_joints_[index].setCommand(0.0);
    wheel_pids_[index].reset();
  }
}

void HeroChassisController::commandCallback(const geometry_msgs::TwistConstPtr& message) {
  if (message == nullptr) {
    return;
  }
  if (!isFinite(message->linear.x) || !isFinite(message->linear.y) ||
      !isFinite(message->angular.z)) {
    ROS_WARN_THROTTLE(2.0, "Ignoring cmd_vel with non-finite planar velocity.");
    return;
  }

  CommandData command;
  command.twist = *message;
  command.received = ros::Time::now();
  command.valid = true;
  command_buffer_.writeFromNonRT(command);
}

void HeroChassisController::reconfigureCallback(HeroChassisControllerConfig& config,
                                                std::uint32_t /*level*/) {
  const std::array<double, 4> kp{{config.left_front_kp, config.right_front_kp,
                                  config.left_back_kp, config.right_back_kp}};
  const std::array<double, 4> ki{{config.left_front_ki, config.right_front_ki,
                                  config.left_back_ki, config.right_back_ki}};
  const std::array<double, 4> kd{{config.left_front_kd, config.right_front_kd,
                                  config.left_back_kd, config.right_back_kd}};
  const std::array<double, 4> i_clamp{{config.left_front_i_clamp, config.right_front_i_clamp,
                                       config.left_back_i_clamp, config.right_back_i_clamp}};
  for (std::size_t index = 0; index < wheel_pids_.size(); ++index) {
    const double integral_limit = std::abs(i_clamp[index]);
    wheel_pids_[index].setGains(kp[index], ki[index], kd[index],
                                integral_limit, -integral_limit, true);
  }
}

void HeroChassisController::publishOdometry(const ros::Time& time,
                                             const ChassisVelocity& velocity) {
  if (odometry_publisher_ && odometry_publisher_->trylock()) {
    nav_msgs::Odometry& odometry = odometry_publisher_->msg_;
    odometry.header.stamp = time;
    odometry.pose.pose.position.x = odometry_pose_.x;
    odometry.pose.pose.position.y = odometry_pose_.y;
    odometry.pose.pose.position.z = 0.0;
    odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odometry_pose_.yaw);
    odometry.twist.twist.linear.x = velocity.x;
    odometry.twist.twist.linear.y = velocity.y;
    odometry.twist.twist.angular.z = velocity.yaw;
    odometry_publisher_->unlockAndPublish();
  }

  if (transform_publisher_ && transform_publisher_->trylock()) {
    geometry_msgs::TransformStamped& transform =
        transform_publisher_->msg_.transforms.front();
    transform.header.stamp = time;
    transform.transform.translation.x = odometry_pose_.x;
    transform.transform.translation.y = odometry_pose_.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = tf::createQuaternionMsgFromYaw(odometry_pose_.yaw);
    transform_publisher_->unlockAndPublish();
  }
}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)

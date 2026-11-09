#pragma once

#include <array>

namespace hero_chassis_controller {

struct ChassisVelocity {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// Wheel order: left-front, right-front, left-back, right-back.
// wheel_track and wheel_base are full center-to-center distances in meters.
inline std::array<double, 4> inverseKinematics(const ChassisVelocity& velocity,
                                               double wheel_radius,
                                               double wheel_track,
                                               double wheel_base) {
  const double lever = 0.5 * (wheel_track + wheel_base);
  const double rotation = lever * velocity.yaw;
  return {{(velocity.x - velocity.y - rotation) / wheel_radius,
           (velocity.x + velocity.y + rotation) / wheel_radius,
           (velocity.x + velocity.y - rotation) / wheel_radius,
           (velocity.x - velocity.y + rotation) / wheel_radius}};
}

inline ChassisVelocity forwardKinematics(const std::array<double, 4>& wheel_speeds,
                                         double wheel_radius,
                                         double wheel_track,
                                         double wheel_base) {
  const double lf = wheel_speeds[0];
  const double rf = wheel_speeds[1];
  const double lb = wheel_speeds[2];
  const double rb = wheel_speeds[3];
  const double lever = 0.5 * (wheel_track + wheel_base);

  ChassisVelocity velocity;
  velocity.x = wheel_radius * 0.25 * (lf + rf + lb + rb);
  velocity.y = wheel_radius * 0.25 * (-lf + rf + lb - rb);
  velocity.yaw = wheel_radius * 0.25 / lever * (-lf + rf - lb + rb);
  return velocity;
}

}  // namespace hero_chassis_controller

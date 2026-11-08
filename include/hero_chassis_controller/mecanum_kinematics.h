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

}  // namespace hero_chassis_controller

#pragma once

#include <cstdint>

#include "sd/types.h"

namespace sd::dynamics
{
  struct StateIdx
  {
    constexpr static int pos_x = 0;   // X position
    constexpr static int pos_y = 1;   // Y position
    constexpr static int pos_z = 2;   // Z position
    constexpr static int angle_r = 3; // Roll angle
    constexpr static int angle_p = 4; // Pitch angle
    constexpr static int angle_y = 5; // Yaw angle
    constexpr static int vel_x = 6;   // X linear velocity
    constexpr static int vel_y = 7;   // Y linear velocity
    constexpr static int vel_z = 8;   // Z linear velocity
    constexpr static int rate_r = 9;  // Roll rate
    constexpr static int rate_p = 10; // Pitch rate
    constexpr static int rate_y = 11; // Yaw rate
  };

  enum class Mode : uint8_t
  {
    Off,
    Stand,
    RecoveryStand,
    Locomotion,
    Vision
  };

  struct CmdLimits
  {
    constexpr static float max_angle_r = 0.4;
    constexpr static float min_angle_r = -0.4;
    constexpr static float max_angle_p = 0.4;
    constexpr static float min_angle_p = -0.4;
    constexpr static float max_vel_x = 3.0;
    constexpr static float min_vel_x = -3.0;
    constexpr static float max_vel_y = 2.0;
    constexpr static float min_vel_y = -2.0;
    constexpr static float max_rate_y = 2.5;
    constexpr static float min_rate_y = -2.5;
    constexpr static float deadband_region = 0.075;
    constexpr static float filter = 0.1;
  };

  template <typename T>
  using StateVec = Eigen::Matrix<T, 12, 1>;

} // namespace sd::dynamics

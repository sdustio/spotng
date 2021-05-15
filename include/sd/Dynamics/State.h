#pragma once

#include <cstdint>

namespace sd::dynamics
{
  namespace StateIdx
  {
    constexpr int PosX = 0;   // X position
    constexpr int PosY = 1;   // Y position
    constexpr int PosZ = 2;   // Z position
    constexpr int AngleR = 3; // Roll angle
    constexpr int AngleP = 4; // Pitch angle
    constexpr int AngleY = 5; // Yaw angle
    constexpr int VelX = 6;   // X linear velocity
    constexpr int VelY = 7;   // Y linear velocity
    constexpr int VelZ = 8;   // Z linear velocity
    constexpr int RateR = 9;  // Roll rate
    constexpr int RateP = 10; // Pitch rate
    constexpr int RateY = 11; // Yaw rate
  }

  enum class Mode : uint8_t
  {
    Off,
    Stand,
    RecoveryStand,
    Locomotion,
    Vision
  };

  namespace CmdLimits
  {
    constexpr float MaxAngleR = 0.4;
    constexpr float MinAngleR = -0.4;
    constexpr float MaxAngleP = 0.4;
    constexpr float MinAngleP = -0.4;
    constexpr float MaxVelX = 3.0;
    constexpr float MinVelX = -3.0;
    constexpr float MaxVelY = 2.0;
    constexpr float MinVelY = -2.0;
    constexpr float MaxRateYaw = 2.5;
    constexpr float MinRateYaw = -2.5;
    constexpr float DeadbandRegion = 0.075;
    constexpr float Filter = 0.1;
  }

} // namespace sd::dynamics

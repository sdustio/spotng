#pragma once

namespace sdrobot::params
{
  constexpr double kZeroEpsilon = 1.e-12;

  namespace cmd
  {
    constexpr double kMaxAngleR = 0.4;
    constexpr double kMinAngleR = -0.4;
    constexpr double kMaxAngleP = 0.4;
    constexpr double kMinAngleP = -0.4;
    constexpr double kMaxVelX = 3.0;
    constexpr double kMinVelX = -3.0;
    constexpr double kMaxVelY = 2.0;
    constexpr double kMinVelY = -2.0;
    constexpr double kMaxRateY = 2.5;
    constexpr double kMinRateY = -2.5;
    constexpr double kMaxVarHeight = 0.2;
    constexpr double kMinVarHeight = -0.2;
    constexpr double kMaxStepHeight = 0.2;
    constexpr double kMinStepHeight = 0.05;
    constexpr double kDeadbandRegion = 0.075;
    constexpr double kFilter = 0.1;
  }
} // namespace sdrobot::params

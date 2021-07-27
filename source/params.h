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

  namespace robot
  {

  } // namespace robot


  namespace ctrl
  {
    constexpr int kActItfmsec = int(1.0 / 0.04); //unit: milliseconds; 0.04kHz
    constexpr int kCtrlmsec = int(1.0 / 0.5);    //unit: milliseconds; 0.5kHz
    constexpr double kActItfsec = kActItfmsec / 1'000;
    constexpr double kCtrlsec = kCtrlmsec / 1'000;

    constexpr double kFootHeightSensorNoise = 0.001;
    constexpr double kFootProcessNoisePosition = 0.002;
    constexpr double kFootSensorNoisePosition = 0.001;
    constexpr double kFootSensorNoiseVelocity = 0.1;
    constexpr double kIMUProcessNoisePosition = 0.02;
    constexpr double kIMUProcessNoiseVelocity = 0.02;
  }
} // namespace sdrobot::params

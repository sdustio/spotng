#pragma once

#include <chrono>
#include <string>

namespace sdrobot::robot
{
  namespace ros
  {
    constexpr char kNodeNs[] = "sd";
    constexpr char kNodeName[] = "RobotRunner";
    constexpr char kTopicCmd[] = "sdrobot/robot_cmd";
    constexpr char kTopicMotion[] = "sdrobot/robot_motion";
    constexpr char kTopicIMU[] = "imu";
  }

  namespace ctrlparams
  {
    constexpr std::chrono::milliseconds kSPIdt(int(1.0 / 0.04));      //0.04kHz
    constexpr std::chrono::milliseconds kCtrldt(int(1.0 / 0.5));      //0.5kHz
    constexpr double kSPIsec = kSPIdt.count() / 1'000;
    constexpr double kCtrlsec = kCtrldt.count() / 1'000;

    constexpr double kFootHeightSensorNoise = 0.001;
    constexpr double kFootProcessNoisePosition = 0.002;
    constexpr double kFootSensorNoisePosition = 0.001;
    constexpr double kFootSensorNoiseVelocity = 0.1;
    constexpr double kIMUProcessNoisePosition = 0.02;
    constexpr double kIMUProcessNoiseVelocity = 0.02;
  }
}

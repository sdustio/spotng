#pragma once

#include <string>
#include <memory>
#include <chrono>
#include <cinttypes>

#include "rclcpp/rclcpp.hpp"
#include "sdrobot_api/msg/driver_cmd.hpp"
#include "sdrobot_api/msg/motion_data.hpp"

#include "sd/Robot/Interface.h"
#include "sd/Robot/Dynamics.h"


namespace sd::robot
{
  const std::string NODE_NS = "sd";
  const std::string NODE_NAME = "Robot";
  const std::string TOPIC_CMD = "sd/robot_cmd";
  const std::string TOPIC_MOTION = "sd/robot_motion";

  namespace ctrldt{
    constexpr std::chrono::milliseconds SPI(int(1.0/0.04)); //0.04kHz
    constexpr std::chrono::microseconds IMU(int(1000 * 1.0/10)); //10kHz
    constexpr std::chrono::milliseconds DYN(int(1.0/0.5)); //0.5kHz
  }

  class Runner : public rclcpp::Node
  {
  public:
    explicit Runner(std::shared_ptr<interface::Interface>);
    bool Init();
    bool Run();

  private:
    void handleDriverCmd(const sdrobot_api::msg::DriverCmd::SharedPtr) const;

    rclcpp::Subscription<sdrobot_api::msg::DriverCmd>::SharedPtr mDriverCmdSub;
    rclcpp::Publisher<sdrobot_api::msg::MotionData>::SharedPtr mMotionDataPub;

    int iter = 0;
    rclcpp::TimerBase::SharedPtr mSPITimer;
    rclcpp::TimerBase::SharedPtr mIMUTimer;
    rclcpp::TimerBase::SharedPtr mDYNTimer;

    std::shared_ptr<interface::Interface> mInterface;
    std::shared_ptr<dynamics::DesiredStateCmd> mDesiredStateCmd;

    interface::SPICmd mSPICmd;
    interface::SPIData mSPIData;
    interface::IMUData mIMUData;
  };

} // namespace sd::robot

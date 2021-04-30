#pragma once

#include <string>
#include <memory>

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

  class Runner : public rclcpp::Node
  {
  public:
    explicit Runner(std::shared_ptr<interface::Interface>);
    bool Init();
    bool Run();

  private:
    void handleDriverCmd(const sdrobot_api::msg::DriverCmd::SharedPtr) const;

    std::shared_ptr<interface::Interface> mInterface;
    std::shared_ptr<dynamics::DesiredStateCmd> mDesiredStateCmd;

    rclcpp::Subscription<sdrobot_api::msg::DriverCmd>::SharedPtr mDriverCmdSub;
    rclcpp::Publisher<sdrobot_api::msg::MotionData>::SharedPtr mMotionDataPub;

    int iter = 0;
    interface::SPICmd mSPICmd;
    interface::SPIData mSPIData;
    interface::IMUData mIMUData;
  };

  int Run(std::shared_ptr<interface::Interface>, int argc, char * argv[]);

} // namespace sd::robot

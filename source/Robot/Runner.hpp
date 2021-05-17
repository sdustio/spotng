#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sdrobot_api/msg/driver_cmd.hpp"
#include "sdrobot_api/msg/motion_data.hpp"

#include "sd/Robot/Runner.hpp"
#include "sd/Robot/Interface.hpp"

#include "Dynamics/DesiredState.hpp"
#include "Robot/Model.hpp"


namespace sd::robot
{

  class Runner : public rclcpp::Node
  {
  public:
    explicit Runner(std::shared_ptr<Interface>);
    bool Init();
    bool Run();

  private:
    void handleDriverCmd(const sdrobot_api::msg::DriverCmd::SharedPtr);

    rclcpp::Subscription<sdrobot_api::msg::DriverCmd>::SharedPtr mDriverCmdSub;
    rclcpp::Publisher<sdrobot_api::msg::MotionData>::SharedPtr mMotionDataPub;

    int iter = 0;
    rclcpp::TimerBase::SharedPtr mSPITimer;
    rclcpp::TimerBase::SharedPtr mIMUTimer;
    rclcpp::TimerBase::SharedPtr mDYNTimer;

    std::shared_ptr<Interface> mInterface;

    dynamics::DesiredStateCmd<double> mDesiredStateCmd;
    Quadruped<double> mQuadruped;
  };

} // namespace sd::robot

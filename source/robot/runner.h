#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sdrobot_api/msg/driver_cmd.hpp"
#include "sdrobot_api/msg/motion_data.hpp"

#include "sd/robot/runner.h"
#include "sd/robot/interface.h"

#include "dynamics/desired_state.h"
#include "robot/model.h"


namespace sd::robot
{

  class Runner : public rclcpp::Node
  {
  public:
    explicit Runner(std::shared_ptr<Interface>);
    bool Init();
    bool Run();

  private:
    void HandleDriverCmd(const sdrobot_api::msg::DriverCmd::SharedPtr);
    void _test();

    rclcpp::Subscription<sdrobot_api::msg::DriverCmd>::SharedPtr driver_cmd_sub_;
    rclcpp::Publisher<sdrobot_api::msg::MotionData>::SharedPtr motion_data_pub_;

    int iter = 0;
    rclcpp::TimerBase::SharedPtr spi_timer_;
    rclcpp::TimerBase::SharedPtr imu_timer_;
    rclcpp::TimerBase::SharedPtr dyn_timer_;

    std::shared_ptr<Interface> interface_;

    dynamics::DesiredStateCmd<double> desired_state_cmd_;
    Quadruped<double> quadruped_;
    dynamics::FBModel<double> fbmodel_;
  };

} // namespace sd::robot

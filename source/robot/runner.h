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

    /**
    * Initializes the robot model, state estimator, leg controller,
    * robot data, and any control logic specific data.
    初始化机器人模型，状态估计器，腿部控制器，
    机器人数据，以及任何控制逻辑特定的数据
    */
    bool Init();

    /**
    * Runs the overall robot control system by calling each of the major components
    * to run each of their respective steps.
    通过调用每个主要组件来运行整个机器人控制系统运行它们各自的步骤。
    */
    bool Run();

  private:
    void HandleDriverCmd(const sdrobot_api::msg::DriverCmd::SharedPtr);

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

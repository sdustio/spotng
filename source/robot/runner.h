#pragma once

#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "sdrobot_api/msg/driver_cmd.hpp"
#include "sdrobot_api/msg/motion_data.hpp"


#include "sd/robot/runner.h"
#include "sd/robot/interface.h"
#include "sd/robot/model.h"

#include "sd/controllers/leg.h"
#include "sd/controllers/jpos_init.h"
#include "sd/controllers/state_cmd.h"

#include "sd/estimators/orientation.h"
#include "sd/estimators/contact.h"


namespace sd::robot
{

  class Runner : public rclcpp::Node
  {
  public:
    explicit Runner(InterfacePtr);

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

    QuadrupedPtr quadruped_;
    dynamics::FBModelPtr fbmodel_;

    ctrl::StateCmdPtr ctrl_state_cmd_;
    ctrl::LegPtr ctrl_leg_;
    ctrl::JPosInitPtr ctrl_jpos_init_;

    est::StateEst est_ret_;
    est::OrientationPtr est_orientation_;
    est::ContactPtr est_contact_;

    Vector4d contact_phase_;
  };

} // namespace sd::robot

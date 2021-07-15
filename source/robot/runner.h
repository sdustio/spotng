#pragma once

#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "sdrobot_interfaces/msg/cmd.hpp"
#include "sensor_msgs/msg/imu.hpp"


#include "sdrobot/robot/runner.h"
#include "sdrobot/robot/interface.h"
#include "sdrobot/robot/model.h"

#include "sdrobot/controllers/leg.h"
#include "sdrobot/controllers/jpos_init.h"
#include "sdrobot/controllers/state_cmd.h"
#include "sdrobot/controllers/fsm.h"

#include "sdrobot/estimators/orientation.h"
#include "sdrobot/estimators/contact.h"
#include "sdrobot/estimators/pos_vel.h"


namespace sdrobot::robot
{

  class Runner : public rclcpp::Node
  {
  public:
    explicit Runner(const InterfacePtr&);

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
    void HandleCmd(const sdrobot_interfaces::msg::Cmd::SharedPtr&);
    void HandleIMU(const sensor_msgs::msg::Imu::SharedPtr&);

    rclcpp::Subscription<sdrobot_interfaces::msg::Cmd>::SharedPtr cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::TimerBase::SharedPtr spi_timer_;
    rclcpp::TimerBase::SharedPtr ctrl_timer_;

    std::shared_ptr<Interface> interface_;

    QuadrupedPtr quadruped_;

    ctrl::StateCmdPtr ctrl_state_cmd_;
    ctrl::LegPtr ctrl_leg_;
    ctrl::JPosInitPtr ctrl_jpos_init_;
    ctrl::FsmPtr ctrl_fsm_;

    est::StateEstPtr est_ret_;
    est::OrientationPtr est_orientation_;
    est::ContactPtr est_contact_;
    est::PosVelPtr est_pos_vel_;

    Vector4 contact_phase_;
    IMUData imu_data_;
  };

} // namespace sdrobot::robot

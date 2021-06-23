#include "sd/controllers/fsm.h"

#include "controllers/fsm/state/off.h"
#include "controllers/fsm/state/ready.h"
#include "controllers/fsm/state/stand_up.h"
#include "controllers/fsm/state/locomotion.h"
#include "controllers/fsm/state/recovery_stand.h"
#include "controllers/fsm/state/balance_stand.h"

namespace sd::ctrl
{
  FSM::FSM()
  {
    state_ctrls_ = {
        std::make_shared<fsm::StateOff>(),
        std::make_shared<fsm::StateReady>(),
        std::make_shared<fsm::StateStandUp>(),
        std::make_shared<fsm::StateLocomotion>(),
        std::make_shared<fsm::StateRecoveryStand>(),
        std::make_shared<fsm::StateBalanceStand>()};

    // Initialize the FSM with the Off FSM State
    Init();
  }

  bool FSM::Init()
  {
    // Initialize a new FSM State with the control data
    current_state_ctrl_ = GetStateCtrl(fsm::State::Off);

    // Enter the new current state cleanly
    current_state_ctrl_->OnEnter();

    // Initialize to not be in transition
    next_state_ctrl_ = current_state_ctrl_;

    // Initialize FSM mode to normal operation
    opmode_ = fsm::OperatingMode::Normal;

    return true;
  }

  fsm::StateCtrlSharedPtr FSM::GetStateCtrl(fsm::State state)
  {
    return state_ctrls_[size_t(state)];
  }

  bool FSM::Run(robot::Mode mode)
  {
    ctrl_mode_ = mode;

    //TODO safetyPreCheck

    // Run the robot control code if operating mode is not unsafe
    //运行机器人控制代码，如果工作模式安全
    if (opmode_ == fsm::OperatingMode::EStop)
    {
      current_state_ctrl_ = GetStateCtrl(fsm::State::Off);
      current_state_ctrl_->OnEnter();
      next_state_ = current_state_ctrl_->GetState();
    }
    else
    {
      //下面为状态机
      // Run normal controls if no transition is detected
      //如果没有检测到过渡，则运行正常控件
      if (opmode_ == fsm::OperatingMode::Normal)
      {
        // Check the current state for any transition 检查任何转换的当前状态
        next_state_ = current_state_ctrl_->CheckTransition();

        // Detect a commanded transition 探测指令转换
        if (next_state_ != current_state_ctrl_->GetState())
        {
          // Set the FSM operating mode to transitioning 将FSM工作模式设置为transitioning
          opmode_ = fsm::OperatingMode::Transitioning;

          // Get the next FSM State by name 按名称获取下一个FSM状态
          next_state_ctrl_ = GetStateCtrl(next_state_);
        }
        else
        {
          //没有状态转换则运行当前状态控制器
          // Run the iteration for the current state normally
          current_state_ctrl_->Run();
        }
      }

      if (opmode_ == fsm::OperatingMode::Transitioning)
      {
        //获得转换数据，进行状态转换操作 有时候是延时切换状态 比如MPClocomotion
        transition_data_ = current_state_ctrl_->Transition();

        // Check the robot state for safe operation
        // TODO post safe check

        // Run the state transition
        if (transition_data_.done) //状态转换完成 延时到
        {
          // Exit the current state cleanly 干净地退出当前状态
          current_state_ctrl_->OnExit();

          // Complete the transition  完成状态转换
          current_state_ctrl_ = next_state_ctrl_;

          // Enter the new current state cleanly 进入新状态
          current_state_ctrl_->OnEnter();

          // Return the FSM to normal operation mode 操作模式设置为一般
          opmode_ = fsm::OperatingMode::Normal;
        }
      }
      else
      {
        //非转换状态 会进行检查 即任何状态下进行检查并限制
        // Check the robot state for safe operation 检查机器人状态，确保操作安全，并限制
        // TODO post safe check
      }
    }

    return true;
  }
} // namespace sd::ctrl

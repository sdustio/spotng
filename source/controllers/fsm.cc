#include "sd/controllers/fsm.h"

#include "controllers/fsm/state/init.h"
#include "controllers/fsm/state/locomotion.h"
#include "controllers/fsm/state/recovery_stand.h"
#include "controllers/fsm/state/balance_stand.h"

namespace sd::ctrl
{
  Fsm::Fsm(
      LegPtr &cleg,
      const StateCmdPtr &cmd,
      const est::StateEstPtr &est) : leg_ctrl_(cleg), state_cmd_(cmd), state_est_(est),
                                     state_ctrls_{
                                         {fsm::State::Init, std::make_shared<fsm::StateInit>(cleg, cmd, est)},
                                         {fsm::State::RecoveryStand, std::make_shared<fsm::StateRecoveryStand>(cleg, cmd, est)},
                                         {fsm::State::Locomotion, std::make_shared<fsm::StateLocomotion>(cleg, cmd, est)},
                                         {fsm::State::BalanceStand, std::make_shared<fsm::StateBalanceStand>(cleg, cmd, est)}}
  {
    // Initialize the Fsm with the Off Fsm State
    Init();
  }

  bool Fsm::Init()
  {
    // Initialize a new Fsm State with the control data
    current_state_ctrl_ = GetStateCtrl(fsm::State::Init);

    // Enter the new current state cleanly
    current_state_ctrl_->OnEnter();

    // Initialize to not be in transition
    next_state_ctrl_ = current_state_ctrl_;

    // Initialize Fsm mode to normal operation
    opmode_ = fsm::OperatingMode::Normal;

    return true;
  }

  fsm::StateCtrlPtr Fsm::GetStateCtrl(fsm::State state)
  {
    return state_ctrls_[state];
  }

  bool Fsm::Run()
  {
    //safetyPreCheck
    if (!PreCheck())
    {
      opmode_ = fsm::OperatingMode::EStop;
    }

    // Run the robot control code if operating mode is not unsafe
    //运行机器人控制代码，如果工作模式安全
    if (opmode_ == fsm::OperatingMode::EStop)
    {
      current_state_ctrl_ = GetStateCtrl(fsm::State::Init);
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
        next_state_ = current_state_ctrl_->CheckTransition(state_cmd_);

        // Detect a commanded transition 探测指令转换
        if (next_state_ != current_state_ctrl_->GetState())
        {
          // Set the Fsm operating mode to transitioning 将FSM工作模式设置为transitioning
          opmode_ = fsm::OperatingMode::Transitioning;

          // Get the next Fsm State by name 按名称获取下一个Fsm状态
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
        transition_data_ = current_state_ctrl_->Transition(next_state_);

        // Check the robot state for safe operation
        PostCheckAndLimit();

        // Run the state transition
        if (transition_data_.done) //状态转换完成 延时到
        {
          // Exit the current state cleanly 干净地退出当前状态
          current_state_ctrl_->OnExit();

          // Complete the transition  完成状态转换
          current_state_ctrl_ = next_state_ctrl_;

          // Enter the new current state cleanly 进入新状态
          current_state_ctrl_->OnEnter();

          // Return the Fsm to normal operation mode 操作模式设置为一般
          opmode_ = fsm::OperatingMode::Normal;
        }
      }
      else
      {
        //非转换状态 会进行检查 即任何状态下进行检查并限制
        // Check the robot state for safe operation 检查机器人状态，确保操作安全，并限制
        PostCheckAndLimit();
      }
    }

    return true;
  }

  bool Fsm::PreCheck()
  {
    if (current_state_ctrl_->NeedCheckSafeOrientation() && state_cmd_->GetMode() != robot::Mode::RecoveryStand)
    {
      return safety_checker_.CheckSafeOrientation(state_est_->GetData());
    }
    return true;
  }

  bool Fsm::PostCheckAndLimit()
  {
    bool c1, c2;
    if (current_state_ctrl_->NeedCheckPDesFoot())
    {
      c1 = safety_checker_.CheckPDesFoot(leg_ctrl_);
    }
    if (current_state_ctrl_->NeedCheckForceFeedForward())
    {
      c2 = safety_checker_.CheckForceFeedForward(leg_ctrl_);
    }
    return c1 && c2;
  }
} // namespace sd::ctrl

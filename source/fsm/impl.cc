#include "fsm/impl.h"

namespace sdrobot::fsm
{

  bool FiniteStateMachineImpl::Init(
      leg::LegCtrl::SharedPtr const &legctrl,
      model::Quadruped::SharedPtr const &mquat,
      drive::DriveCtrl::SharedPtr const &drictrl,
      estimate::EstimateCtrl::SharedPtr const &estctrl)
  {
    // state_ctrls_ =
    // { {State::Init, std::make_shared<StateInit>(cleg, quad, cmd, est)},
    //   {State::RecoveryStand, std::make_shared<StateRecoveryStand>(cleg, quad, cmd, est)},
    //   {State::Locomotion, std::make_shared<StateLocomotion>(cleg, quad, cmd, est)},
    //   {State::BalanceStand, std::make_shared<StateBalanceStand>(cleg, quad, cmd, est)}
    // };

    legctrl_ = legctrl;
    mquat_ = mquat;
    drictrl_ = drictrl;
    estctrl_ = estctrl;

    // Initialize a new Fsm State with the control data
    current_state_ctrl_ = GetStateCtrl(State::Init);

    // Enter the new current state cleanly
    current_state_ctrl_->OnEnter();

    // Initialize to not be in transition
    next_state_ctrl_ = current_state_ctrl_;

    // Initialize Fsm mode to normal operation
    opmode_ = OperatingMode::Normal;

    return true;
  }

  StateCtrl::SharedPtr const &FiniteStateMachineImpl::GetStateCtrl(State state)
  {
    return state_ctrls_[state];
  }

  bool FiniteStateMachineImpl::RunOnce()
  {
    //safetyPreCheck
    if (!PreCheck())
    {
      opmode_ = OperatingMode::EStop;
    }

    // Run the robot control code if operating mode is not unsafe
    //运行机器人控制代码，如果工作模式安全
    if (opmode_ == OperatingMode::EStop)
    {
      current_state_ctrl_ = GetStateCtrl(State::Init);
      current_state_ctrl_->OnEnter();
      next_state_ = current_state_ctrl_->GetState();
    }
    else
    {
      //下面为状态机
      // Run normal controls if no transition is detected
      //如果没有检测到过渡，则运行正常控件
      if (opmode_ == OperatingMode::Normal)
      {
        // Check the current state for any transition 检查任何转换的当前状态
        next_state_ = current_state_ctrl_->CheckTransition();

        // Detect a commanded transition 探测指令转换
        if (next_state_ != current_state_ctrl_->GetState())
        {
          // Set the Fsm operating mode to transitioning 将FSM工作模式设置为transitioning
          opmode_ = OperatingMode::Transitioning;

          // Get the next Fsm State by name 按名称获取下一个Fsm状态
          next_state_ctrl_ = GetStateCtrl(next_state_);
        }
        else
        {
          //没有状态转换则运行当前状态控制器
          // Run the iteration for the current state normally
          current_state_ctrl_->RunOnce();
        }
      }

      if (opmode_ == OperatingMode::Transitioning)
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
          opmode_ = OperatingMode::Normal;
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

  bool FiniteStateMachineImpl::PreCheck()
  {
    if (current_state_ctrl_->NeedCheckSafeOrientation() && drictrl_->GetState() != State::RecoveryStand)
    {
      return safety_checker_.CheckSafeOrientation(estctrl_);
    }
    return true;
  }

  bool FiniteStateMachineImpl::PostCheckAndLimit()
  {
    bool c1, c2;
    if (current_state_ctrl_->NeedCheckPDesFoot())
    {
      c1 = safety_checker_.CheckPDesFoot(legctrl_);
    }
    if (current_state_ctrl_->NeedCheckForceFeedForward())
    {
      c2 = safety_checker_.CheckForceFeedForward(legctrl_);
    }
    return c1 && c2;
  }

}

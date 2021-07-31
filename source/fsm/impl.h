#pragma once

#include <unordered_map>
#include "sdrobot/fsm.h"

namespace sdrobot::fsm
{
  class FiniteStateMachineImpl : public FiniteStateMachine
  {
  public:
    bool Init(
        leg::LegCtrl::SharedPtr const &legctrl,
        model::Quadruped::SharedPtr const &mquat,
        drive::DriveCtrl::SharedPtr const &drictrl,
        estimate::EstimateCtrl::SharedPtr const &estctrl) override;

    StateCtrl::SharedPtr GetStateCtrl(State state) override;

    bool RunOnce() override;

  private:
    /**
    * Initialize the Control Fsm with the default settings. SHould be set to
    * Passive state and Normal operation mode.
    */
    bool PreCheck();
    bool PostCheckAndLimit();

    std::unordered_map<State, StateCtrl::SharedPtr> state_ctrls_;

    leg::LegCtrl::SharedPtr legctrl_;
    model::Quadruped::SharedPtr mquat_;
    drive::DriveCtrl::SharedPtr drictrl_;
    estimate::EstimateCtrl::SharedPtr estctrl_;
  };
}

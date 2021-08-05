#pragma once

#include <unordered_map>

#include "sdrobot/fsm.h"
#include "sdrobot/estimate.h"
#include "sdrobot/leg.h"
#include "sdrobot/model.h"

namespace sdrobot::fsm
{
  class StateInit : public StateCtrl
  {
  public:
    StateInit(
        Options const &opts,
        leg::LegCtrl::SharedPtr const &legctrl,
        model::Quadruped::SharedPtr const &mquat,
        drive::DriveCtrl::SharedPtr const &drictrl,
        estimate::EstimateCtrl::SharedPtr const &estctrl);

    void OnEnter() override;
    void OnExit() override;
    bool RunOnce() override;
    State CheckTransition() override
    {
      return state_trans_[drictrl_->GetState()];
    }
    TransitionData Transition(const State next) override;

    State GetState() const override { return State::Init; }

  private:
    std::unordered_map<drive::State, State> state_trans_;

    leg::LegCtrl::SharedPtr legctrl_;
    model::Quadruped::SharedPtr mquat_;
    drive::DriveCtrl::SharedPtr drictrl_;
    estimate::EstimateCtrl::SharedPtr estctrl_;
  };

}

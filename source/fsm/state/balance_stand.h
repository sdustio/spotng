#pragma once

#include <unordered_map>

#include "sdrobot/fsm.h"
#include "sdrobot/estimate.h"
#include "sdrobot/leg.h"
#include "sdrobot/model.h"

#include "wbc/wbc_ctrl.h"


namespace sdrobot::fsm
{
  class StateBalanceStand : public StateCtrl
  {
  public:
    StateBalanceStand(
        Options const &opts,
        leg::LegCtrl::SharedPtr const &legctrl,
        model::Quadruped::SharedPtr const &mquad,
        drive::DriveCtrl::SharedPtr const &drictrl,
        estimate::EstimateCtrl::SharedPtr const &estctrl);
    bool OnEnter() override;
    bool OnExit() override;
    bool RunOnce() override;

    State CheckTransition() override
    {
      return state_trans_[drictrl_->GetState()];
    }
    TransitionData Transition(const State next) override;

    State GetState() const override { return State::BalanceStand; }

    bool NeedCheckSafeOrientation() const override { return true; }
    bool NeedCheckForceFeedForward() const override { return true; }

  private:
    // Parses contact specific controls to the leg controller
    bool Step();

    std::unordered_map<drive::State, State> state_trans_;

    leg::LegCtrl::SharedPtr legctrl_;
    drive::DriveCtrl::ConstSharedPtr drictrl_;
    estimate::EstimateCtrl::ConstSharedPtr estctrl_;

    wbc::WbcCtrl::Ptr wbc_;
    wbc::InData wbc_data_;

    SdVector3f ini_body_pos_;
    SdVector3f ini_body_rpy_;
    double body_weight_;
  };
}

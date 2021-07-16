#pragma once

#include "sdrobot/controllers/fsm.h"
#include "sdrobot/controllers/wbc.h"

namespace sdrobot::ctrl::fsm
{
  class StateBalanceStand : public StateCtrl
  {
  public:
    StateBalanceStand(const LegPtr &cleg, const robot::QuadrupedPtr &quad, const StateCmdPtr &cmd, const est::StateEstPtr &est);
    void OnEnter() override;
    void OnExit() override;
    bool Run() override;

    State CheckTransition() override
    {
      return state_trans_[state_cmd_->GetMode()];
    }
    TransitionData Transition(const State next) override;

    State GetState() const override { return State::BalanceStand; }

    bool NeedCheckSafeOrientation() const override { return true; }
    bool NeedCheckForceFeedForward() const override { return true; }

  private:
    // Parses contact specific controls to the leg controller
    void Step();

    std::unordered_map<robot::Mode, State> state_trans_;

    WbcPtr wbc_;
    WbcData wbc_data_;

    Vector3 ini_body_pos_;
    Vector3 _ini_body_ori_rpy;
    double body_weight_;
  };

} // namespace sdrobot::ctrl::fsm

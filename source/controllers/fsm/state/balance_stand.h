#pragma once

#include "sd/controllers/fsm.h"
#include "sd/controllers/wbc.h"

namespace sd::ctrl::fsm
{
  class StateBalanceStand : public StateCtrl
  {
  public:
    StateBalanceStand(const LegPtr &cleg, const robot::QuadrupedPtr &quad, const StateCmdPtr &cmd, const est::StateEstPtr &est);
    void OnEnter() override;
    void OnExit() override;
    bool Run() override;

    State CheckTransition(const StateCmdPtr &cmd) override
    {
      return state_trans_[cmd->GetMode()];
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

    Vector3d ini_body_pos_;
    double body_weight_;
  };

} // namespace sd::ctrl::fsm

#pragma once

#include "sd/controllers/fsm.h"

namespace sd::ctrl::fsm
{
  class StateBalanceStand : public StateCtrl
  {
  public:
    StateBalanceStand(LegPtr &cleg, const StateCmdPtr &cmd, const est::StateEstPtr &est);
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

    double last_height_cmd_ = 0.;

    Vector3d ini_body_pos_;
    Vector3d ini_body_ori_rpy_;
    double body_weight_;
  };

} // namespace sd::ctrl::fsm

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

    State CheckTransition(const StateCmdPtr &cmd) override;
    TransitionData Transition(const State next) override;

    State GetState() const override { return State::BalanceStand; }

    bool NeedCheckSafeOrientation() const override { return true; }
    bool NeedCheckForceFeedForward() const override { return true; }
  };

} // namespace sd::ctrl::fsm

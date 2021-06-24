#pragma once

#include "sd/controllers/fsm.h"

namespace sd::ctrl::fsm
{
  class StateBalanceStand : public StateCtrl
  {
  public:
    void OnEnter() override;
    void OnExit() override;
    bool Run() override;

    State CheckTransition(const StateCmdPtr &cmd) override;
    TransitionData Transition(const State next) override;

    State GetState() const override { return State::BalanceStand; }

    // Pre controls safety checks
    bool NeedCheckSafeOrientation() const override;

    // Post control safety checks
    bool NeedCheckPDesFoot() const override;
    bool NeedCheckForceFeedForward() const override;
  };

} // namespace sd::ctrl::fsm

#pragma once

#include "sd/controllers/fsm.h"

namespace sd::ctrl::fsm
{
  class StateStandUp : public StateCtrl
  {
  public:
    void OnEnter() override;
    void OnExit() override;
    bool Run() override;
    State CheckTransition(const StateCmdPtr &cmd) override;
    TransitionData Transition() override;

    State GetState() const override;

    // Pre controls safety checks
    bool NeedCheckSafeOrientation() const override;

    // Post control safety checks
    bool NeedCheckPDesFoot() const override;
    bool NeedCheckForceFeedForward() const override;
    bool NeedCheckLegSingularity() const override;

  };

} // namespace sd::ctrl::fsm

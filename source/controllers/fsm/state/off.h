#pragma once

#include "sd/controllers/fsm.h"

namespace sd::ctrl::fsm
{
  class StateOff : public StateCtrl
  {
  public:
    void OnEnter() override;
    void OnExit() override;
    bool Run() override;
    State CheckTransition() override;
    TransitionData Transition() override;

    State GetState() const override{return State::Off;}

    // Pre controls safety checks
    bool NeedCheckSafeOrientation() const override;

    // Post control safety checks
    bool NeedCheckPDesFoot() const override;
    bool NeedCheckForceFeedForward() const override;
    bool NeedCheckLegSingularity() const override;

  };

} // namespace sd::ctrl::fsm

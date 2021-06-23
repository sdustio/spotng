#pragma once

#include "sd/controllers/fsm.h"

namespace sd::ctrl::fsm
{
  class StateRecoveryStand : public StateCtrl
  {
  public:
    void OnEnter() override;
    void OnExit() override;
    bool Run() override;
    State CheckTransition() override;
    TransitionData Transition() override;
    State GetState() override;
  };

} // namespace sd::ctrl::fsm

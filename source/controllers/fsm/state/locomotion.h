#pragma once

#include "sd/controllers/fsm.h"

namespace sd::ctrl::fsm
{
  class StateLocomotion : public StateCtrl
  {
  public:
    StateLocomotion(LegPtr &cleg, const StateCmdPtr &cmd, const est::StateEstPtr &est);
    void OnEnter() override;
    void OnExit() override;
    bool Run() override;

    State CheckTransition(const StateCmdPtr &cmd) override
    {
      return state_trans_[cmd->GetMode()];
    }

    TransitionData Transition(const State next) override;

    State GetState() const override{return State::Locomotion;}

    bool NeedCheckSafeOrientation() const override { return true; }
    bool NeedCheckForceFeedForward() const override { return true; }

  private:
    std::unordered_map<robot::Mode, State> state_trans_;
  };

} // namespace sd::ctrl::fsm

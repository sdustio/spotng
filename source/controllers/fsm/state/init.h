#pragma once

#include "sd/controllers/fsm.h"

namespace sd::ctrl::fsm
{
  class StateInit : public StateCtrl
  {
  public:
    StateInit(LegPtr &cleg, const StateCmdPtr &cmd, const est::StateEstPtr &est);
    void OnEnter() override;
    void OnExit() override;
    bool Run() override;
    State CheckTransition(const StateCmdPtr &cmd) override;
    TransitionData Transition(const State next) override;

    State GetState() const override { return State::Init; }

  private:
    std::unordered_map<robot::Mode, State> state_trans_;
  };

} // namespace sd::ctrl::fsm

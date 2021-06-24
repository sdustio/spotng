#pragma once

#include "sd/controllers/fsm.h"

namespace sd::ctrl::fsm
{
  class StateInit : public StateCtrl
  {
  public:
    StateInit();
    void OnEnter() override;
    void OnExit() override;
    bool Run() override;
    State CheckTransition(const StateCmdPtr &cmd) override;
    TransitionData Transition(const State next) override;

    State GetState() const override { return State::Init; }

  private:
    std::array<State, size_t(robot::Mode::Count_)> state_trans_;
  };

} // namespace sd::ctrl::fsm

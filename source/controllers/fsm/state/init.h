#pragma once

#include "sdrobot/controllers/fsm.h"

namespace sdrobot::ctrl::fsm
{
  class StateInit : public StateCtrl
  {
  public:
    StateInit(const LegPtr &cleg, const robot::QuadrupedPtr &quad, const StateCmdPtr &cmd, const est::StateEstPtr &est);
    void OnEnter() override;
    void OnExit() override;
    bool Run() override;
    State CheckTransition(const StateCmdPtr &cmd) override
    {
      return state_trans_[cmd->GetMode()];
    }
    TransitionData Transition(const State next) override;

    State GetState() const override { return State::Init; }

  private:
    std::unordered_map<robot::Mode, State> state_trans_;
  };

} // namespace sdrobot::ctrl::fsm

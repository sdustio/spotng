#include "controllers/fsm/state/init.h"

namespace sd::ctrl::fsm
{

  StateInit::StateInit(
      LegPtr &cleg, const StateCmdPtr &cmd,
      const est::StateEstPtr &est) : StateCtrl(cleg, cmd, est),
                                     state_trans_{
                                         {robot::Mode::Init, State::Init},
                                         {robot::Mode::RecoveryStand, State::RecoveryStand},
                                         {robot::Mode::Locomotion, State::RecoveryStand},
                                         {robot::Mode::BalanceStand, State::RecoveryStand}} {}

  void StateInit::OnEnter()
  {
    // nothing
  }
  void StateInit::OnExit()
  {
    // nothing
  }

  bool StateInit::Run()
  {
    return true;
  }

  TransitionData StateInit::Transition([[maybe_unused]] const State next)
  {
    return TransitionData{true};
  }

} // namespace sd::ctrl::fsm

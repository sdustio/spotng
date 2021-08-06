#include "fsm/state/init.h"

namespace sdrobot::fsm
{

  StateInit::StateInit(
      drive::DriveCtrl::SharedPtr const &drictrl) : state_trans_{
                                                        {drive::State::Init, State::Init},
                                                        {drive::State::RecoveryStand, State::RecoveryStand},
                                                        {drive::State::Locomotion, State::RecoveryStand},
                                                        {drive::State::BalanceStand, State::RecoveryStand}},
                                                    drictrl_(drictrl) {}

  bool StateInit::OnEnter()
  {
    // nothing
    return true;
  }
  bool StateInit::OnExit()
  {
    // nothing
    return true;
  }

  bool StateInit::RunOnce()
  {
    return true;
  }

  TransitionData StateInit::Transition([[maybe_unused]] const State next)
  {
    return TransitionData{true};
  }
} // namespace sdrobot::fsm

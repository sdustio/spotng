#include "fsm/state/init.h"

namespace sdrobot::fsm
{

  StateInit::StateInit(
      [[maybe_unused]] Options const &opts,
      [[maybe_unused]] leg::LegCtrl::SharedPtr const &legctrl,
      [[maybe_unused]] model::Quadruped::SharedPtr const &mquat,
      drive::DriveCtrl::SharedPtr const &drictrl,
      [[maybe_unused]] estimate::EstimateCtrl::SharedPtr const &estctrl) : state_trans_{
                                                                               {drive::State::Init, State::Init},
                                                                               {drive::State::RecoveryStand, State::RecoveryStand},
                                                                               {drive::State::Locomotion, State::RecoveryStand},
                                                                               {drive::State::BalanceStand, State::RecoveryStand}},
                                                                           drictrl_(drictrl) {}

  void StateInit::OnEnter()
  {
    // nothing
  }
  void StateInit::OnExit()
  {
    // nothing
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

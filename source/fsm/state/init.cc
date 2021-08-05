#include "fsm/state/init.h"

namespace sdrobot::fsm
{

  StateInit::StateInit(
      [[maybe_unused]] Options const &opts,
      leg::LegCtrl::SharedPtr const &legctrl,
      model::Quadruped::SharedPtr const &mquat,
      drive::DriveCtrl::SharedPtr const &drictrl,
      estimate::EstimateCtrl::SharedPtr const &estctrl) : state_trans_{
                                                              {drive::State::Init, State::Init},
                                                              {drive::State::RecoveryStand, State::RecoveryStand},
                                                              {drive::State::Locomotion, State::RecoveryStand},
                                                              {drive::State::BalanceStand, State::RecoveryStand}},
                                                          legctrl_(legctrl), mquat_(mquat), drictrl_(drictrl), estctrl_(estctrl) {}

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

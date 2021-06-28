#include "controllers/fsm/state/locomotion.h"

namespace sd::ctrl::fsm
{

  StateLocomotion::StateLocomotion(
      LegPtr &cleg, const StateCmdPtr &cmd,
      const est::StateEstPtr &est) : StateCtrl(cleg, cmd, est),
                                     state_trans_{
                                         {robot::Mode::Init, State::Init},
                                         {robot::Mode::RecoveryStand, State::RecoveryStand},
                                         {robot::Mode::Locomotion, State::RecoveryStand},
                                         {robot::Mode::BalanceStand, State::RecoveryStand}} {}

  void StateLocomotion::OnEnter() {}
  void StateLocomotion::OnExit(){}
  bool StateLocomotion::Run(){
    return true;
  }

  TransitionData StateLocomotion::Transition(const State next){
    return TransitionData{true};
  }

} // namespace sd::ctrl::fsm

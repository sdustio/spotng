#include "controllers/fsm/state/locomotion.h"

namespace sd::ctrl::fsm
{

  void StateLocomotion::OnEnter() {}
  void StateLocomotion::OnExit(){}
  bool StateLocomotion::Run(){}
  State StateLocomotion::CheckTransition(const StateCmdPtr &cmd){}
  TransitionData StateLocomotion::Transition(const State next){}

} // namespace sd::ctrl::fsm

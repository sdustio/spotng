#include "controllers/fsm/state/ready.h"

namespace sd::ctrl::fsm
{

  void StateReady::OnEnter() {}
  void StateReady::OnExit(){}
  bool StateReady::Run(){}
  State StateReady::CheckTransition(const StateCmdPtr &cmd){}
  TransitionData StateReady::Transition(){}

} // namespace sd::ctrl::fsm

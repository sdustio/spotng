#include "controllers/fsm/state/off.h"

namespace sd::ctrl::fsm
{

  void StateOff::OnEnter() {}
  void StateOff::OnExit(){}
  bool StateOff::Run(){}
  State StateOff::CheckTransition(const StateCmdPtr &cmd){}
  TransitionData StateOff::Transition(){}

} // namespace sd::ctrl::fsm

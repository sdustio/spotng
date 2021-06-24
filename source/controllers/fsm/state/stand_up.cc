#include "controllers/fsm/state/stand_up.h"

namespace sd::ctrl::fsm
{

  void StateStandUp::OnEnter() {}
  void StateStandUp::OnExit(){}
  bool StateStandUp::Run(){}
  State StateStandUp::CheckTransition(const StateCmdPtr &cmd){}
  TransitionData StateStandUp::Transition(){}

} // namespace sd::ctrl::fsm

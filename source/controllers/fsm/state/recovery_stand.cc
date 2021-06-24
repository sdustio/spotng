#include "controllers/fsm/state/recovery_stand.h"

namespace sd::ctrl::fsm
{

  void StateRecoveryStand::OnEnter() {}
  void StateRecoveryStand::OnExit(){}
  bool StateRecoveryStand::Run(){}
  State StateRecoveryStand::CheckTransition(const StateCmdPtr &cmd){}
  TransitionData StateRecoveryStand::Transition(){}

} // namespace sd::ctrl::fsm

#include "controllers/fsm/state/balance_stand.h"

namespace sd::ctrl::fsm
{

  void StateBalanceStand::OnEnter() {}
  void StateBalanceStand::OnExit(){}
  bool StateBalanceStand::Run(){}
  State StateBalanceStand::CheckTransition(const StateCmdPtr &cmd){}
  TransitionData StateBalanceStand::Transition(){}

} // namespace sd::ctrl::fsm

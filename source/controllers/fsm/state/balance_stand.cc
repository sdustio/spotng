#include "controllers/fsm/state/balance_stand.h"

namespace sd::ctrl::fsm
{

  StateBalanceStand::StateBalanceStand(
      LegPtr &cleg, const StateCmdPtr &cmd,
      const est::StateEstPtr &est) : StateCtrl(cleg, cmd, est),
                                     state_trans_{
                                         {robot::Mode::Init, State::Init},
                                         {robot::Mode::RecoveryStand, State::RecoveryStand},
                                         {robot::Mode::Locomotion, State::RecoveryStand},
                                         {robot::Mode::BalanceStand, State::RecoveryStand}} {}

  void StateBalanceStand::OnEnter() {}
  void StateBalanceStand::OnExit(){}
  bool StateBalanceStand::Run(){
    return true;
  }

  State StateBalanceStand::CheckTransition(const StateCmdPtr &cmd)
  {
    return state_trans_[cmd->GetMode()];
  }

  TransitionData StateBalanceStand::Transition(const State next){
    return TransitionData{true};
  }

} // namespace sd::ctrl::fsm

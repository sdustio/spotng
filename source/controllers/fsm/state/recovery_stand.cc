#include "controllers/fsm/state/recovery_stand.h"

namespace sd::ctrl::fsm
{
  StateRecoveryStand::StateRecoveryStand() : state_trans_{
                                                 State::Init,
                                                 State::RecoveryStand,
                                                 State::Locomotion,
                                                 State::BalanceStand} {}

  void StateRecoveryStand::OnEnter() {}
  void StateRecoveryStand::OnExit() {
    // do nothing
  }
  bool StateRecoveryStand::Run() {}

  State StateRecoveryStand::CheckTransition(const StateCmdPtr &cmd)
  {
    return state_trans_[size_t(cmd->GetMode())];
  }

  TransitionData StateRecoveryStand::Transition([[maybe_unused]] const State next)
  {
    return TransitionData{.done = true};
  }

} // namespace sd::ctrl::fsm

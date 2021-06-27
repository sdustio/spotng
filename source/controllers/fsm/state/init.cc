#include "controllers/fsm/state/init.h"

namespace sd::ctrl::fsm
{

  StateInit::StateInit(
      LegPtr &cleg, const StateCmdPtr &cmd,
      const est::StateEstPtr &est) : StateCtrl(cleg, cmd, est),
                                     state_trans_{
                                         State::Init,
                                         State::RecoveryStand,
                                         State::RecoveryStand,
                                         State::RecoveryStand} {}

  void StateInit::OnEnter()
  {
    // nothing
  }
  void StateInit::OnExit()
  {
    // nothing
  }

  bool StateInit::Run()
  {
    return true;
  }

  State StateInit::CheckTransition(const StateCmdPtr &cmd)
  {
    return state_trans_[size_t(cmd->GetMode())];
  }

  TransitionData StateInit::Transition([[maybe_unused]] const State next)
  {
    return TransitionData{true};
  }

} // namespace sd::ctrl::fsm

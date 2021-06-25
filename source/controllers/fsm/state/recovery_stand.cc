#include "controllers/fsm/state/recovery_stand.h"

namespace sd::ctrl::fsm
{
  StateRecoveryStand::StateRecoveryStand(
      LegPtr &cleg, const StateCmdPtr &cmd,
      const est::StateEstPtr &est) : StateCtrl(cleg, cmd, est),
                                     state_trans_{
                                         State::Init,
                                         State::RecoveryStand,
                                         State::Locomotion,
                                         State::BalanceStand},
                                     flag_dispatch_{
                                         &StateRecoveryStand::_StandUp,
                                         &StateRecoveryStand::_FoldLegs,
                                         &StateRecoveryStand::_RollOver} {}

  void StateRecoveryStand::OnEnter()
  {
    _iter = 0;
    _state_iter = 0;

    // initial configuration, position
    for (size_t i(0); i < 4; ++i)
    {
      initial_jpos[i] = this->leg_ctrl_->GetDatas()[i].q;
    }

    double body_height = state_est_->GetData().position[2];

    _flag = FoldLegs;
    if (!_UpsideDown())
    { // Proper orientation
      if ((0.2 < body_height) && (body_height < 0.45))
      {
        // printf("[Recovery Balance] body height is %f; Stand Up \n", body_height);
        _flag = StandUp;
      }
    }
    _motion_start_iter = 0;
  }

  bool StateRecoveryStand::_UpsideDown()
  {
    return (state_est_->GetData().rot_body(2, 2) < 0);
  }

  void StateRecoveryStand::OnExit()
  {
    // do nothing
  }

  bool StateRecoveryStand::Run()
  {
    (this->*flag_dispatch_[_flag])(_state_iter - _motion_start_iter);
    _state_iter++;
  }

  State StateRecoveryStand::CheckTransition(const StateCmdPtr &cmd)
  {
    return state_trans_[size_t(cmd->GetMode())];
  }

  TransitionData StateRecoveryStand::Transition([[maybe_unused]] const State next)
  {
    return TransitionData{.done = true};
  }

  void StateRecoveryStand::_StandUp(const int iter)
  {
  }
  void StateRecoveryStand::_FoldLegs(const int iter)
  {
  }
  void StateRecoveryStand::_RollOver(const int iter)
  {
  }
  void StateRecoveryStand::_SetJPosInterPts(
      const int curr_iter, size_t max_iter, int leg,
      const Vector3d &ini, const Vector3d &fin)
  {
  }

} // namespace sd::ctrl::fsm

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
                                         &StateRecoveryStand::_RollOver}
  {
    // goal configuration
    // Folding
    fold_jpos[0] << -0.0, -1.4, 2.7;
    fold_jpos[1] << 0.0, -1.4, 2.7;
    fold_jpos[2] << -0.0, -1.4, 2.7;
    fold_jpos[3] << 0.0, -1.4, 2.7;
    // Stand Up
    for (auto &i : stand_jpos)
      i << 0., -.8, 1.6;
    // Rolling
    rolling_jpos[0] << 1.5, -1.6, 2.77;
    rolling_jpos[1] << 1.3, -3.1, 2.77;
    rolling_jpos[2] << 1.5, -1.6, 2.77;
    rolling_jpos[3] << 1.3, -3.1, 2.77;

    f_ff << 0., 0., -25.;

    kpMat << 80, 0, 0, 0, 80, 0, 0, 0, 80;
    kdMat << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    // kpMat << 120, 0, 0, 0, 120, 0, 0, 0, 120;
    // kdMat << 4, 0, 0, 0, 4, 0, 0, 0, 4;
  }

  void StateRecoveryStand::OnEnter()
  {
    _state_iter = 0;

    // initial configuration, position
    for (auto i = 0; i < robot::ModelAttrs::num_leg; ++i)
    {
      initial_jpos[i] = this->leg_ctrl_->GetDatas()[i].q;
    }

    double body_height = state_est_->GetData().position[2];

    _flag = FoldLegs;
    if (!_UpsideDown())
    { // Proper orientation
      if ((0.2 < body_height) && (body_height < 0.45))
      {
        // printf("[Recovery Balance] body height is %; Stand Up \n", body_height);
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

  void StateRecoveryStand::_StandUp(const unsigned long curr_iter)
  {
    double body_height = state_est_->GetData().position[2];
    bool something_wrong = false;

    if (_UpsideDown() || (body_height < 0.1))
    {
      something_wrong = true;
    }

    if ((curr_iter > floor(standup_ramp_iter * 0.7)) && something_wrong)
    {
      // If body height is too low because of some reason
      // even after the stand up motion is almost over
      // (Can happen when E-Stop is engaged in the middle of Other state)
      for (auto i = 0; i < robot::ModelAttrs::num_leg; ++i)
      {
        initial_jpos[i] = leg_ctrl_->GetDatas()[i].q;
      }
      _flag = FoldLegs;
      _motion_start_iter = _state_iter + 1;

      // printf("[Recovery Balance - Warning] body height is still too low (%f) or UpsideDown (%d); Folding legs \n",
      //        body_height, _UpsideDown());
    }
    else
    {
      for (auto leg = 0; leg < robot::ModelAttrs::num_leg; ++leg)
      {
        _SetJPosInterPts(curr_iter, standup_ramp_iter,
                         leg, initial_jpos[leg], stand_jpos[leg]);
      }
    }
    // TODO check contact
    // Vector4d se_contactState(0.5, 0.5, 0.5, 0.5);
    // this->_data->_stateEstimator->setContactPhase(se_contactState);
  }

  void StateRecoveryStand::_FoldLegs(const unsigned long curr_iter)
  {
    for (auto i = 0; i < robot::ModelAttrs::num_leg; ++i)
    {
      _SetJPosInterPts(curr_iter, fold_ramp_iter, i,
                       initial_jpos[i], fold_jpos[i]);
    }
    if (curr_iter >= fold_ramp_iter + fold_settle_iter)
    {
      if (_UpsideDown())
      {
        _flag = RollOver;
        for (auto i = 0; i < robot::ModelAttrs::num_leg; ++i)
          initial_jpos[i] = fold_jpos[i];
      }
      else
      {
        _flag = StandUp;
        for (auto i = 0; i < robot::ModelAttrs::num_leg; ++i)
          initial_jpos[i] = fold_jpos[i];
      }
      _motion_start_iter = _state_iter + 1;
    }
  }

  void StateRecoveryStand::_RollOver(const unsigned long curr_iter)
  {
    for (auto i = 0; i < robot::ModelAttrs::num_leg; ++i)
    {
      _SetJPosInterPts(curr_iter, rollover_ramp_iter, i,
                       initial_jpos[i], rolling_jpos[i]);
    }

    if (curr_iter > rollover_ramp_iter + rollover_settle_iter)
    {
      _flag = FoldLegs;
      for (auto i = 0; i < robot::ModelAttrs::num_leg; ++i)
        initial_jpos[i] = rolling_jpos[i];
      _motion_start_iter = _state_iter + 1;
    }
  }

  void StateRecoveryStand::_SetJPosInterPts(
      const unsigned long curr_iter, unsigned long max_iter, int leg,
      const Vector3d &ini, const Vector3d &fin)
  {
    double a = 0.;
    double b = 1.;

    // if we're done interpolating
    if (curr_iter <= max_iter)
    {
      b = curr_iter / max_iter;
      a = 1. - b;
    }

    // compute setpoints
    Vector3d inter_pos = a * ini + b * fin;

    // do control
    jointPDControl(leg, inter_pos, Vector3d::Zero());
  }

  void StateRecoveryStand::jointPDControl(int leg, const Vector3d &qDes, const Vector3d &qdDes)
  {
    leg_ctrl_->GetCmdsForUpdate()[leg].kp_joint = kpMat;
    leg_ctrl_->GetCmdsForUpdate()[leg].kd_joint = kdMat;

    leg_ctrl_->GetCmdsForUpdate()[leg].q_des = qDes;
    leg_ctrl_->GetCmdsForUpdate()[leg].qd_des = qdDes;
  }

} // namespace sd::ctrl::fsm

#include "controllers/fsm/state/recovery_stand.h"

namespace sdrobot::ctrl::fsm
{
  StateRecoveryStand::StateRecoveryStand(
      const LegPtr &cleg, const robot::QuadrupedPtr &quad, const StateCmdPtr &cmd,
      const est::StateEstPtr &est) : StateCtrl(cleg, quad, cmd, est),
                                     state_trans_{
                                         {robot::Mode::Init, State::Init},
                                         {robot::Mode::RecoveryStand, State::RecoveryStand},
                                         {robot::Mode::Locomotion, State::Locomotion},
                                         {robot::Mode::BalanceStand, State::BalanceStand}},
                                     flag_dispatch_{
                                         {Flag::StandUp, &StateRecoveryStand::StandUp},
                                         {Flag::FoldLegs, &StateRecoveryStand::FoldLegs},
                                         {Flag::RollOver, &StateRecoveryStand::RollOver}}
  {
    // goal configuration
    // Folding
    fold_jpos_[0] << -0.0, -1.4, 2.7;
    fold_jpos_[1] << 0.0, -1.4, 2.7;
    fold_jpos_[2] << -0.0, -1.4, 2.7;
    fold_jpos_[3] << 0.0, -1.4, 2.7;
    // Stand Up
    for (auto &i : stand_jpos_)
      i << 0., -.8, 1.6;
    // Rolling
    rolling_jpos_[0] << 1.5, -1.6, 2.77;
    rolling_jpos_[1] << 1.3, -3.1, 2.77;
    rolling_jpos_[2] << 1.5, -1.6, 2.77;
    rolling_jpos_[3] << 1.3, -3.1, 2.77;

    kp_mat_ << 80, 0, 0, 0, 80, 0, 0, 0, 80;
    kd_mat_ << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    // kp_mat_ << 120, 0, 0, 0, 120, 0, 0, 0, 120;
    // kd_mat_ << 4, 0, 0, 0, 4, 0, 0, 0, 4;
  }

  void StateRecoveryStand::OnEnter()
  {
    iter_ = 0;

    // initial configuration, position
    for (int i = 0; i < robot::ModelAttrs::num_leg; ++i)
    {
      initial_jpos_[i] = this->leg_ctrl_->GetDatas()[i].q;
    }

    double body_height = state_est_->GetData().position[2];

    flag_ = Flag::FoldLegs;
    if (!UpsideDown())
    { // Proper orientation
      if ((0.2 < body_height) && (body_height < 0.45))
      {
        // printf("[Recovery Balance] body height is %; Stand Up \n", body_height);
        flag_ = Flag::StandUp;
      }
    }
  }

  bool StateRecoveryStand::UpsideDown()
  {
    return (state_est_->GetData().rot_body(2, 2) < 0);
  }

  void StateRecoveryStand::OnExit()
  {
    // do nothing
  }

  bool StateRecoveryStand::Run()
  {
    (this->*flag_dispatch_[flag_])(iter_);
    iter_++;
    return true;
  }

  TransitionData StateRecoveryStand::Transition([[maybe_unused]] const State next)
  {
    return TransitionData{true};
  }

  void StateRecoveryStand::StandUp(const int curr_iter)
  {
    double body_height = state_est_->GetData().position[2];
    bool something_wrong = false;

    if (UpsideDown() || (body_height < 0.1))
    {
      something_wrong = true;
    }

    if (curr_iter <= floor(standup_ramp_iter_ * 0.7))
    {
      for (int leg = 0; leg < robot::ModelAttrs::num_leg; ++leg)
      {
        SetJPosInterPts(curr_iter, standup_ramp_iter_,
                        leg, initial_jpos_[leg], stand_jpos_[leg]);
      }
    }
    else if (something_wrong)
    {
      // If body height is too low because of some reason
      // even after the stand up motion is almost over
      // (Can happen when E-Stop is engaged in the middle of Other state)
      for (int i = 0; i < robot::ModelAttrs::num_leg; ++i)
      {
        initial_jpos_[i] = leg_ctrl_->GetDatas()[i].q;
      }
      flag_ = Flag::FoldLegs;
      iter_ = -1;

      // printf("[Recovery Balance - Warning] body height is still too low (%f) or UpsideDown (%d); Folding legs \n",
      //        body_height, UpsideDown());
    }
    else
    {
      iter_ = 0;
    }

    // TODO check contact
    // Vector4 se_contactState(0.5, 0.5, 0.5, 0.5);
    // this->_data->_stateEstimator->setContactPhase(se_contactState);
  }

  void StateRecoveryStand::FoldLegs(const int curr_iter)
  {
    for (int i = 0; i < robot::ModelAttrs::num_leg; ++i)
    {
      SetJPosInterPts(curr_iter, fold_ramp_iter_, i,
                      initial_jpos_[i], fold_jpos_[i]);
    }
    if (curr_iter >= fold_ramp_iter_ + fold_settle_iter_)
    {
      if (UpsideDown())
      {
        flag_ = Flag::RollOver;
        for (int i = 0; i < robot::ModelAttrs::num_leg; ++i)
          initial_jpos_[i] = fold_jpos_[i];
      }
      else
      {
        flag_ = Flag::StandUp;
        for (int i = 0; i < robot::ModelAttrs::num_leg; ++i)
          initial_jpos_[i] = fold_jpos_[i];
      }
      iter_ = -1;
    }
  }

  void StateRecoveryStand::RollOver(const int curr_iter)
  {
    for (int i = 0; i < robot::ModelAttrs::num_leg; ++i)
    {
      SetJPosInterPts(curr_iter, rollover_ramp_iter_, i,
                      initial_jpos_[i], rolling_jpos_[i]);
    }

    if (curr_iter > rollover_ramp_iter_ + rollover_settle_iter_)
    {
      flag_ = Flag::FoldLegs;
      for (int i = 0; i < robot::ModelAttrs::num_leg; ++i)
        initial_jpos_[i] = rolling_jpos_[i];
      iter_ = -1;
    }
  }

  void StateRecoveryStand::SetJPosInterPts(
      const int curr_iter, int max_iter, int leg,
      const Vector3 &ini, const Vector3 &fin)
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
    Vector3 inter_pos = a * ini + b * fin;

    // do control
    JointPDControl(leg, inter_pos, Vector3::Zero());
  }

  void StateRecoveryStand::JointPDControl(int leg, const Vector3 &qDes, const Vector3 &qdDes)
  {
    leg_ctrl_->GetCmdsForUpdate()[leg].kp_joint = kp_mat_;
    leg_ctrl_->GetCmdsForUpdate()[leg].kd_joint = kd_mat_;

    leg_ctrl_->GetCmdsForUpdate()[leg].q_des = qDes;
    leg_ctrl_->GetCmdsForUpdate()[leg].qd_des = qdDes;
  }

} // namespace sdrobot::ctrl::fsm

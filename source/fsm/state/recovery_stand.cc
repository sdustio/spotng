#include "fsm/state/recovery_stand.h"

#include "estimate/contact.h"
#include "externlib/eigen.h"
#include "math/interpolate.h"
#include "sdquadx/consts.h"
#include "spdlog/spdlog.h"

namespace sdquadx::fsm {

namespace params {

constexpr int const fold_ramp_iter = 1000;
constexpr int const fold_settle_iter = 1000;

constexpr int const standup_ramp_iter = 500;
constexpr int const standup_settle_iter = 500;

constexpr int const rollover_ramp_iter = 300;
constexpr int const rollover_settle_iter = 300;

}  // namespace params

StateRecoveryStand::StateRecoveryStand(Options::ConstSharedPtr const &opts, leg::LegCtrl::SharedPtr const &legctrl,
                                       drive::DriveCtrl::SharedPtr const &drictrl,
                                       estimate::EstimateCtrl::SharedPtr const &estctrl)
    : state_trans_{{drive::State::Init, State::Init},
                   {drive::State::RecoveryStand, State::RecoveryStand},
                   {drive::State::Locomotion, State::Locomotion},
                   {drive::State::BalanceStand, State::BalanceStand}},
      flag_dispatch_{{Flag::StandUp, &StateRecoveryStand::StandUp},
                     {Flag::FoldLegs, &StateRecoveryStand::FoldLegs},
                     {Flag::RollOver, &StateRecoveryStand::RollOver}},
      opts_(opts),
      legctrl_(legctrl),
      drictrl_(drictrl),
      estctrl_(estctrl)

{}

bool StateRecoveryStand::OnEnter() {
  spdlog::debug("Enter State Recovery Stand!!!");
  iter_ = 0;

  // initial configuration, position
  for (int i = 0; i < consts::model::kNumLeg; ++i) {
    initial_jpos_[i] = legctrl_->GetDatas()[i].q;
  }

  auto body_height = estctrl_->GetEstState().pos[2];

  flag_ = Flag::FoldLegs;
  if (!UpsideDown()) {  // Proper orientation
    if ((consts::model::kBodyHeight < body_height) &&
        (body_height < consts::model::kMaxLegLength + consts::model::kBodyHeight)) {
      spdlog::debug(" body height is {}; Stand Up!!", body_height);
      flag_ = Flag::StandUp;
    }
  }
  return true;
}

bool StateRecoveryStand::UpsideDown() { return (ToConstEigenTp(estctrl_->GetEstState().rot_mat)(2, 2) < 0); }

bool StateRecoveryStand::OnExit() {
  // do nothing
  return true;
}

bool StateRecoveryStand::RunOnce() {
  if (iter_ < 0) return true;
  (this->*flag_dispatch_[flag_])();
  return true;
}

TransitionData StateRecoveryStand::Transition([[maybe_unused]] const State next) { return TransitionData{true}; }

bool StateRecoveryStand::StandUp() {
  auto body_height = estctrl_->GetEstState().pos[2];
  bool something_wrong = false;

  if (UpsideDown() || (body_height < consts::model::kBodyHeight)) {
    something_wrong = true;
  }

  if (iter_ <= floor((params::standup_ramp_iter + params::standup_settle_iter) * 0.7)) {
    for (int leg = 0; leg < consts::model::kNumLeg; ++leg) {
      SetJPosInterPts(iter_, params::standup_ramp_iter, leg, initial_jpos_[leg], opts_->stand_jpos[leg]);
    }
    iter_++;
  } else if (something_wrong) {
    // If body height is too low because of some reason
    // even after the stand up motion is almost over
    // (Can happen when E-Stop is engaged in the middle of Other state)
    for (int i = 0; i < consts::model::kNumLeg; ++i) {
      initial_jpos_[i] = legctrl_->GetDatas()[i].q;
    }
    flag_ = Flag::FoldLegs;
    iter_ = 0;

    // printf("[Recovery Balance - Warning] body height is still too low (%f) or
    // UpsideDown (%d); Folding legs \n",
    //        body_height, UpsideDown());
  } else {
    iter_ = -1;
  }

  auto est_contact = std::dynamic_pointer_cast<estimate::Contact>(estctrl_->GetEstimator("contact"));
  est_contact->UpdateContact({0.5, 0.5, 0.5, 0.5});

  return true;
}

bool StateRecoveryStand::FoldLegs() {
  for (int i = 0; i < consts::model::kNumLeg; ++i) {
    SetJPosInterPts(iter_, params::fold_ramp_iter, i, initial_jpos_[i], opts_->fold_jpos[i]);
  }
  iter_++;
  if (iter_ >= params::fold_ramp_iter + params::fold_settle_iter) {
    if (UpsideDown())
      flag_ = Flag::RollOver;
    else
      flag_ = Flag::StandUp;
    for (int i = 0; i < consts::model::kNumLeg; ++i) initial_jpos_[i] = opts_->fold_jpos[i];
    iter_ = 0;
  }
  return true;
}

bool StateRecoveryStand::RollOver() {
  for (int i = 0; i < consts::model::kNumLeg; ++i) {
    SetJPosInterPts(iter_, params::rollover_ramp_iter, i, initial_jpos_[i], opts_->rolling_jpos[i]);
  }
  iter_++;
  if (iter_ > params::rollover_ramp_iter + params::rollover_settle_iter) {
    flag_ = Flag::FoldLegs;
    for (int i = 0; i < consts::model::kNumLeg; ++i) initial_jpos_[i] = opts_->rolling_jpos[i];
    iter_ = 0;
  }
  return true;
}

bool StateRecoveryStand::SetJPosInterPts(int const curr_iter, int const max_iter, int const leg, SdVector3f const &ini,
                                         SdVector3f const &fin) {
  auto &cmd = legctrl_->GetCmdsForUpdate()[leg];
  math::interpolate_linear(ToEigenTp(cmd.q_des), ToConstEigenTp(ini), ToConstEigenTp(fin),
                           std::fmin(static_cast<fpt_t>(curr_iter) / max_iter, 1.));
  ToEigenTp(cmd.kp_joint).diagonal() = ToConstEigenTp(opts_->kp_st);
  ToEigenTp(cmd.kd_joint).diagonal() = ToConstEigenTp(opts_->kd_st);
  return true;
}

State StateRecoveryStand::CheckTransition() {
  if (flag_ != Flag::StandUp) return State::RecoveryStand;

  return state_trans_[drictrl_->GetState()];
}
}  // namespace sdquadx::fsm

#include "fsm/state/recovery_stand.h"

#include "externlib/eigen.h"
#include "estimate/contact.h"

namespace sdquadx::fsm {
namespace opts {
constexpr inline int const fold_ramp_iter = 1000;
constexpr inline int const fold_settle_iter = 1000;

constexpr inline int const standup_ramp_iter = 500;
constexpr inline int const standup_settle_iter = 500;

// 0.5 kHz
constexpr inline int const rollover_ramp_iter = 150;
constexpr inline int const rollover_settle_iter = 150;
// iteration setup
// constexpr inline int const rollover_ramp_iter = 300;
// constexpr inline int const rollover_settle_iter = 300;

// JPos
constexpr inline std::array<SdVector3f, 4> const fold_jpos = {SdVector3f{-0.0, -1.4, 2.7}, SdVector3f{0.0, -1.4, 2.7},
                                                              SdVector3f{-0.0, -1.4, 2.7}, SdVector3f{0.0, -1.4, 2.7}};
constexpr inline std::array<SdVector3f, 4> const stand_jpos = {SdVector3f{0., -.8, 1.6}, SdVector3f{0., -.8, 1.6},
                                                               SdVector3f{0., -.8, 1.6}, SdVector3f{0., -.8, 1.6}};
constexpr inline std::array<SdVector3f, 4> const rolling_jpos = {
    SdVector3f{1.5, -1.6, 2.77}, SdVector3f{1.3, -3.1, 2.77}, SdVector3f{1.5, -1.6, 2.77}, SdVector3f{1.3, -3.1, 2.77}};

// 对角线矩阵，row major == column major
constexpr inline SdMatrix3f const kp_mat = {80, 0, 0, 0, 80, 0, 0, 0, 80};
constexpr inline SdMatrix3f const kd_mat = {1, 0, 0, 0, 1, 0, 0, 0, 1};
// kp_mat << 120, 0, 0, 0, 120, 0, 0, 0, 120;
// kd_mat << 4, 0, 0, 0, 4, 0, 0, 0, 4;
}  // namespace opts

StateRecoveryStand::StateRecoveryStand(leg::LegCtrl::SharedPtr const &legctrl,
                                       drive::DriveCtrl::SharedPtr const &drictrl,
                                       estimate::EstimateCtrl::SharedPtr const &estctrl)
    : state_trans_{{drive::State::Init, State::Init},
                   {drive::State::RecoveryStand, State::RecoveryStand},
                   {drive::State::Locomotion, State::Locomotion},
                   {drive::State::BalanceStand, State::BalanceStand}},
      flag_dispatch_{{Flag::StandUp, &StateRecoveryStand::StandUp},
                     {Flag::FoldLegs, &StateRecoveryStand::FoldLegs},
                     {Flag::RollOver, &StateRecoveryStand::RollOver}},
      legctrl_(legctrl),
      drictrl_(drictrl),
      estctrl_(estctrl)

{}

bool StateRecoveryStand::OnEnter() {
  iter_ = 0;

  // initial configuration, position
  for (int i = 0; i < consts::model::kNumLeg; ++i) {
    initial_jpos_[i] = legctrl_->GetDatas()[i].q;
  }

  auto body_height = estctrl_->GetEstState().pos[2];

  flag_ = Flag::FoldLegs;
  if (!UpsideDown()) {  // Proper orientation
    if ((0.2 < body_height) && (body_height < 0.45)) {
      // printf("[Recovery Balance] body height is %; Stand Up \n",
      // body_height);
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
  (this->*flag_dispatch_[flag_])(iter_);
  iter_++;
  return true;
}

TransitionData StateRecoveryStand::Transition([[maybe_unused]] const State next) { return TransitionData{true}; }

bool StateRecoveryStand::StandUp(const int curr_iter) {
  auto body_height = estctrl_->GetEstState().pos[2];
  bool something_wrong = false;

  if (UpsideDown() || (body_height < 0.1)) {
    something_wrong = true;
  }

  if (curr_iter <= floor(opts::standup_ramp_iter * 0.7)) {
    for (int leg = 0; leg < consts::model::kNumLeg; ++leg) {
      SetJPosInterPts(curr_iter, opts::standup_ramp_iter, leg, initial_jpos_[leg], opts::stand_jpos[leg]);
    }
  } else if (something_wrong) {
    // If body height is too low because of some reason
    // even after the stand up motion is almost over
    // (Can happen when E-Stop is engaged in the middle of Other state)
    for (int i = 0; i < consts::model::kNumLeg; ++i) {
      initial_jpos_[i] = legctrl_->GetDatas()[i].q;
    }
    flag_ = Flag::FoldLegs;
    iter_ = -1;

    // printf("[Recovery Balance - Warning] body height is still too low (%f) or
    // UpsideDown (%d); Folding legs \n",
    //        body_height, UpsideDown());
  } else {
    iter_ = 0;
  }

  auto est_contact = std::dynamic_pointer_cast<estimate::Contact>(estctrl_->GetEstimator("contact"));
  est_contact->UpdateContact({0.5, 0.5, 0.5, 0.5});

  return true;
}

bool StateRecoveryStand::FoldLegs(const int curr_iter) {
  for (int i = 0; i < consts::model::kNumLeg; ++i) {
    SetJPosInterPts(curr_iter, opts::fold_ramp_iter, i, initial_jpos_[i], opts::fold_jpos[i]);
  }
  if (curr_iter >= opts::fold_ramp_iter + opts::fold_settle_iter) {
    if (UpsideDown()) {
      flag_ = Flag::RollOver;
      for (int i = 0; i < consts::model::kNumLeg; ++i) initial_jpos_[i] = opts::fold_jpos[i];
    } else {
      flag_ = Flag::StandUp;
      for (int i = 0; i < consts::model::kNumLeg; ++i) initial_jpos_[i] = opts::fold_jpos[i];
    }
    iter_ = -1;
  }
  return true;
}

bool StateRecoveryStand::RollOver(const int curr_iter) {
  for (int i = 0; i < consts::model::kNumLeg; ++i) {
    SetJPosInterPts(curr_iter, opts::rollover_ramp_iter, i, initial_jpos_[i], opts::rolling_jpos[i]);
  }

  if (curr_iter > opts::rollover_ramp_iter + opts::rollover_settle_iter) {
    flag_ = Flag::FoldLegs;
    for (int i = 0; i < consts::model::kNumLeg; ++i) initial_jpos_[i] = opts::rolling_jpos[i];
    iter_ = -1;
  }
  return true;
}

bool StateRecoveryStand::SetJPosInterPts(int const curr_iter, int const max_iter, int const leg, SdVector3f const &ini,
                                         SdVector3f const &fin) {
  double a = 0.;
  double b = 1.;

  // if we're done interpolating
  if (curr_iter <= max_iter) {
    b = curr_iter / max_iter;
    a = 1. - b;
  }

  // compute setpoints
  SdVector3f inter_pos;
  ToEigenTp(inter_pos) = a * ToConstEigenTp(ini) + b * ToConstEigenTp(fin);

  // do control
  return JointPDControl(leg, inter_pos, SdVector3f{});
}

bool StateRecoveryStand::JointPDControl(int const leg, SdVector3f const &qDes, SdVector3f const &qdDes) {
  legctrl_->GetCmdsForUpdate()[leg].kp_joint = opts::kp_mat;
  legctrl_->GetCmdsForUpdate()[leg].kd_joint = opts::kd_mat;

  legctrl_->GetCmdsForUpdate()[leg].q_des = qDes;
  legctrl_->GetCmdsForUpdate()[leg].qd_des = qdDes;
  return true;
}

State StateRecoveryStand::CheckTransition() {
  if (flag_ != Flag::StandUp) return State::RecoveryStand;

  return state_trans_[drictrl_->GetState()];
}
}  // namespace sdquadx::fsm

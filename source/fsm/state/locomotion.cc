#include "fsm/state/locomotion.h"

#include <memory>

#include "drive/drive_ctrl_impl.h"
#include "math/utils.h"
#include "mpc/cmpc.h"
#include "skd/od_gait.h"
#include "spdlog/spdlog.h"
#include "utils/eigen.h"
#include "wbc/wbic.h"

namespace sdquadx::fsm {

using V4i = skd::SdVector4i;
using GaitSkd = skd::OffsetDurationGait;

StateLocomotion::StateLocomotion(Options::ConstSharedPtr const &opts, LegCtrl::SharedPtr const &legctrl,
                                 model::Quadruped::SharedPtr const &mquad,
                                 drive::DriveCtrl::ConstSharedPtr const &drictrl,
                                 estimate::EstimateCtrl::ConstSharedPtr const &estctrl)
    : state_trans_{{drive::State::Init, State::Init},
                   {drive::State::RecoveryStand, State::RecoveryStand},
                   {drive::State::Locomotion, State::Locomotion},
                   {drive::State::BalanceStand, State::BalanceStand}},
      opts_(opts),
      legctrl_(legctrl),
      drictrl_(drictrl),
      estctrl_(estctrl),
      state_des_(std::make_unique<skd::StateDes>(opts)),
      mpc_(std::make_unique<mpc::CMpc>(opts)),
      wbc_(std::make_unique<wbc::Wbic>(opts, mquad)) {
  auto scale = opts->ctrl.mpc_iters;
  gait_skds_[drive::Gait::Trot] =
      std::make_shared<GaitSkd>(10 * scale, V4i{0 * scale, 5 * scale, 5 * scale, 0 * scale},
                                V4i{5 * scale, 5 * scale, 5 * scale, 5 * scale}, opts->ctrl_sec, "Trot");
  gait_skds_[drive::Gait::SlowTrot] =
      std::make_shared<GaitSkd>(12 * scale, V4i{0 * scale, 6 * scale, 6 * scale, 0 * scale},
                                V4i{6 * scale, 6 * scale, 6 * scale, 6 * scale}, opts->ctrl_sec, "SlowTrot");
  gait_skds_[drive::Gait::FlyingTrot] =
      std::make_shared<GaitSkd>(10 * scale, V4i{0 * scale, 5 * scale, 5 * scale, 0 * scale},
                                V4i{4 * scale, 4 * scale, 4 * scale, 4 * scale}, opts->ctrl_sec, "FlyingTrot");
  gait_skds_[drive::Gait::Walk] =
      std::make_shared<GaitSkd>(16 * scale, V4i{0 * scale, 8 * scale, 4 * scale, 12 * scale},
                                V4i{12 * scale, 12 * scale, 12 * scale, 12 * scale}, opts->ctrl_sec, "Walk");
  gait_skds_[drive::Gait::Bound] =
      std::make_shared<GaitSkd>(10 * scale, V4i{5 * scale, 5 * scale, 0 * scale, 0 * scale},
                                V4i{5 * scale, 5 * scale, 5 * scale, 5 * scale}, opts->ctrl_sec, "Bound");

  estcontact_ = std::dynamic_pointer_cast<estimate::Contact>(estctrl_->GetEstimator("contact"));
}

bool StateLocomotion::OnEnter() {
  spdlog::info("Enter State Locomotion!!!");
  iter_counter_ = 0;
  return true;
}

bool StateLocomotion::OnExit() {
  iter_counter_ = 0;
  return true;
}

TransitionData StateLocomotion::Transition(const State next) {
  if (next == State::BalanceStand) {
    RunOnce();
  }

  return TransitionData{true};
}

State StateLocomotion::CheckTransition() {
  if (!SafeCheck()) return State::RecoveryStand;
  return state_trans_[drictrl_->GetState()];
}

// Parses contact specific controls to the leg controller
bool StateLocomotion::RunOnce() {
  legctrl_->ZeroCmds();

  auto const &gait_skd = gait_skds_[drictrl_->GetGait()];
  gait_skd->SetCurrentIter(iter_counter_);

  state_des_->RunOnce(wbc_data_, estctrl_->GetEstState(), drictrl_, gait_skd);

  if ((iter_counter_ % opts_->ctrl.mpc_iters) == 0) {
    mpc_->RunOnce(wbc_data_, estctrl_->GetEstState(), gait_skd->GetNextPeriodStanceStates());
  }

  wbc_->RunOnce(legctrl_->cmds, wbc_data_, estctrl_->GetEstState());

  estcontact_->UpdateContact(wbc_data_.contact_state);

  iter_counter_++;

  return true;
}

bool StateLocomotion::SafeCheck() {
  auto const &seResult = estctrl_->GetEstState();

  if (std::fabs(seResult.rpy[0]) > opts_->model.max_body_roll) {
    spdlog::warn("Unsafe locomotion: roll is {} degrees (max {})", math::RadToDeg(seResult.rpy[0]),
                 opts_->model.max_body_roll);
    return false;
  }

  if (std::fabs(seResult.rpy[1]) > opts_->model.max_body_pitch) {
    spdlog::warn("Unsafe locomotion: pitch is {} degrees (max {})", math::RadToDeg(seResult.rpy[1]),
                 opts_->model.max_body_pitch);
    return false;
  }

  for (int leg = 0; leg < 4; leg++) {
    if (seResult.foot_pos_robot[leg][2] > 0) {
      spdlog::warn("Unsafe locomotion: leg {} is above hip ({} m)", leg, seResult.foot_pos_robot[leg][2]);
      return false;
    }

    if (std::fabs(seResult.foot_pos_robot[leg][1] > 0.346)) {  // 0.18))
      spdlog::warn("Unsafe locomotion: leg {}'s y-position is bad ({} m)", leg, seResult.foot_pos_robot[leg][1]);
      return false;
    }

    auto v_leg = ToConstEigenTp(seResult.foot_vel_robot[leg]).norm();

    if (std::fabs(v_leg) > 19.) {
      spdlog::warn("Unsafe locomotion: leg {} is moving too quickly ({} m/s)", leg, v_leg);
      return false;
    }
  }

  return true;
}
}  // namespace sdquadx::fsm

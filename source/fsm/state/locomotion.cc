#include "fsm/state/locomotion.h"

#include <memory>

#include "drive/drive_ctrl_impl.h"
#include "math/utils.h"
#include "mpc/cmpc.h"
#include "spdlog/spdlog.h"
#include "wbc/wbic.h"

namespace sdquadx::fsm {
namespace params {
constexpr inline fpt_t const max_roll = 80.;   // 40;
constexpr inline fpt_t const max_pitch = 80.;  // 40;
}  // namespace params

StateLocomotion::StateLocomotion(Options::ConstSharedPtr const &opts, LegCtrl::SharedPtr const &legctrl,
                                 model::Quadruped::SharedPtr const &mquad,
                                 drive::DriveCtrl::ConstSharedPtr const &drictrl,
                                 estimate::EstimateCtrl::ConstSharedPtr const &estctrl)
    : state_trans_{{drive::State::Init, State::Init},
                   {drive::State::RecoveryStand, State::RecoveryStand},
                   {drive::State::Locomotion, State::Locomotion},
                   {drive::State::BalanceStand, State::BalanceStand}},
      legctrl_(legctrl),
      drictrl_(drictrl),
      estctrl_(estctrl),
      wbc_(std::make_unique<wbc::Wbic>(opts, mquad)),
      mpc_(std::make_unique<mpc::CMpc>(opts, mquad)) {}

bool StateLocomotion::OnEnter() {
  spdlog::info("Enter State Locomotion!!!");
  return true;
}

bool StateLocomotion::OnExit() { return true; }

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
  // Contact state logic
  // estimateContact();
  legctrl_->ZeroCmds();

  mpc_->RunOnce(legctrl_->cmds, wbc_data_, estctrl_->GetEstState(), drictrl_);

  wbc_->RunOnce(legctrl_->cmds, wbc_data_, estctrl_->GetEstState());

  return true;
}

bool StateLocomotion::SafeCheck() {
  auto const &seResult = estctrl_->GetEstState();

  if (std::fabs(seResult.rpy[0]) > math::DegToRad(params::max_roll)) {
    spdlog::warn("Unsafe locomotion: roll is {} degrees (max {})", math::RadToDeg(seResult.rpy[0]), params::max_roll);
    return false;
  }

  if (std::fabs(seResult.rpy[1]) > math::DegToRad(params::max_pitch)) {
    spdlog::warn("Unsafe locomotion: pitch is {} degrees (max {})", math::RadToDeg(seResult.rpy[1]), params::max_pitch);
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

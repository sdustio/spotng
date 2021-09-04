#include "fsm/state/locomotion.h"

#include <memory>

#include "drive/drive_ctrl_impl.h"
#include "math/utils.h"
#include "mpc/cmpc.h"
#include "spdlog/spdlog.h"

namespace sdquadx::fsm {
namespace opts {
constexpr inline double const max_roll = 80.;   // 40;
constexpr inline double const max_pitch = 80.;  // 40;
}  // namespace opts

StateLocomotion::StateLocomotion(Options::ConstSharedPtr const &opts, leg::LegCtrl::SharedPtr const &legctrl,
                                 model::Quadruped::SharedPtr const &mquad, drive::DriveCtrl::SharedPtr const &drictrl,
                                 estimate::EstimateCtrl::SharedPtr const &estctrl)
    : state_trans_{{drive::State::Init, State::Init},
                   {drive::State::RecoveryStand, State::RecoveryStand},
                   {drive::State::Locomotion, State::Locomotion},
                   {drive::State::BalanceStand, State::BalanceStand}},
      legctrl_(legctrl),
      mquad_(mquad),
      drictrl_(drictrl),
      estctrl_(estctrl),
      wbc_(std::make_unique<wbc::WbcCtrl>(mquad->GetFloatBaseModel(), opts)),
      mpc_(std::make_unique<mpc::CMpc>(opts->ctrl_sec, opts->gravity, 30 / static_cast<int>(1000. * opts->ctrl_sec))) {}

bool StateLocomotion::OnEnter() {
  spdlog::debug("Enter State Locomotion!!!");
  return mpc_->Init();
}

bool StateLocomotion::OnExit() { return true; }

bool StateLocomotion::RunOnce() {
  // Call the locomotion control logic for this iteration
  return LocomotionControlStep();
}

TransitionData StateLocomotion::Transition(const State next) {
  if (next == State::BalanceStand) {
    LocomotionControlStep();
  }

  return TransitionData{true};
}

State StateLocomotion::CheckTransition() {
  if (locomotionSafe()) {
    return state_trans_[drictrl_->GetState()];
  }

  std::dynamic_pointer_cast<drive::DriveCtrlImpl>(std::const_pointer_cast<drive::DriveCtrl>(drictrl_))
      ->UpdateState(drive::State::RecoveryStand);
  return State::RecoveryStand;
}

// Parses contact specific controls to the leg controller
bool StateLocomotion::LocomotionControlStep() {
  // StateEstimate<T> stateEstimate = this->_data->_stateEstimator->getResult();

  // Contact state logic
  // estimateContact();

  mpc_->RunOnce(wbc_data_, legctrl_, mquad_, drictrl_, estctrl_);

  wbc_->RunOnce(wbc_data_, legctrl_, drictrl_, estctrl_);

  return true;
}

bool StateLocomotion::locomotionSafe() {
  auto const &seResult = estctrl_->GetEstState();

  if (std::fabs(seResult.rpy[0]) > math::DegToRad(opts::max_roll)) {
    // printf("Unsafe locomotion: roll is %.3f degrees (max %.3f)\n",
    // math::RadToDeg(seResult.rpy[0]), max_roll);
    return false;
  }

  if (std::fabs(seResult.rpy[1]) > math::DegToRad(opts::max_pitch)) {
    // printf("Unsafe locomotion: pitch is %.3f degrees (max %.3f)\n",
    // math::RadToDeg(seResult.rpy[1]), max_pitch);
    return false;
  }

  for (int leg = 0; leg < 4; leg++) {
    auto const &leg_data = legctrl_->GetDatas()[leg];

    if (leg_data.p[2] > 0) {
      // printf("Unsafe locomotion: leg %d is above hip (%.3f m)\n", leg,
      // leg_data.p[2]);
      return false;
    }

    if (std::fabs(leg_data.p[1] > 0.28)) {  // 0.18))
      // printf("Unsafe locomotion: leg %d's y-position is bad (%.3f m)\n", leg,
      // leg_data.p[1]);
      return false;
    }

    auto v_leg = ToConstEigenTp(leg_data.v).norm();

    if (std::fabs(v_leg) > 19.) {
      // printf("Unsafe locomotion: leg %d is moving too quickly (%.3f m/s)\n",
      // leg, v_leg);
      return false;
    }
  }

  return true;
}
}  // namespace sdquadx::fsm

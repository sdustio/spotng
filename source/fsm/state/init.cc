#include "fsm/state/init.h"

#include "math/interpolate.h"
#include "spdlog/spdlog.h"

namespace forax::fsm {

namespace params {
constexpr int const ramp_iter = 500;
constexpr int const settle_iter = 500;
}  // namespace params

StateInit::StateInit(Options::ConstSharedPtr const &opts, LegCtrl::SharedPtr const &legctrl,
                     drive::DriveCtrl::ConstSharedPtr const &drictrl,
                     estimate::EstimateCtrl::ConstSharedPtr const &estctrl)
    : state_trans_{{drive::State::Init, State::Init},
                   {drive::State::RecoveryStand, State::RecoveryStand},
                   {drive::State::Locomotion, State::RecoveryStand},
                   {drive::State::BalanceStand, State::RecoveryStand}},
      opts_(opts),
      legctrl_(legctrl),
      drictrl_(drictrl),
      estctrl_(estctrl) {}

bool StateInit::OnEnter() {
  // nothing
  spdlog::info("Enter State Init!!!");
  iter_ = 0;
  initial_jpos_ = estctrl_->GetEstState().q;

  return true;
}
bool StateInit::OnExit() {
  // nothing
  return true;
}

bool StateInit::RunOnce() {
  if (iter_ < 0) return true;
  for (int i = 0; i < consts::model::kNumLeg; ++i)
    SetJPosInterPts(legctrl_->cmds[i], iter_, params::ramp_iter, initial_jpos_[i], opts_->ctrl.jpos_init[i]);
  iter_++;
  if (iter_ >= params::ramp_iter + params::settle_iter) {
    iter_ = -1;
    first_run_ = false;
  }

  return true;
}

bool StateInit::SetJPosInterPts(interface::LegCmd &cmd, int const curr_iter, int const max_iter, SdVector3f const &ini,
                                SdVector3f const &fin) {
  math::InterpolateLinear(ToEigenTp(cmd.q_des), ToConstEigenTp(ini), ToConstEigenTp(fin),
                          std::fmin(static_cast<fpt_t>(curr_iter) / max_iter, 1.));
  cmd.kp = opts_->ctrl.kp_jpos;
  cmd.kd = opts_->ctrl.kd_jpos;

  return true;
}

State StateInit::CheckTransition() {
  if (first_run_) return GetState();
  return state_trans_[drictrl_->GetState()];
}

TransitionData StateInit::Transition([[maybe_unused]] const State next) { return TransitionData{true}; }
}  // namespace forax::fsm

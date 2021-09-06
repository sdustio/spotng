#include "fsm/state/init.h"

#include "math/interpolate.h"
#include "spdlog/spdlog.h"

namespace sdquadx::fsm {

namespace params {
constexpr int const ramp_iter = 500;
constexpr int const settle_iter = 500;
}  // namespace params

StateInit::StateInit(Options::ConstSharedPtr const &opts, leg::LegCtrl::SharedPtr const &legctrl,
                     drive::DriveCtrl::SharedPtr const &drictrl)
    : state_trans_{{drive::State::Init, State::Init},
                   {drive::State::RecoveryStand, State::RecoveryStand},
                   {drive::State::Locomotion, State::RecoveryStand},
                   {drive::State::BalanceStand, State::RecoveryStand}},
      opts_(opts),
      legctrl_(legctrl),
      drictrl_(drictrl) {}

bool StateInit::OnEnter() {
  // nothing
  spdlog::debug("Enter State Init!!!");
  iter_ = 0;

  // initial configuration, position
  for (int i = 0; i < consts::model::kNumLeg; ++i) {
    initial_jpos_[i] = legctrl_->GetDatas()[i].q;
  }

  return true;
}
bool StateInit::OnExit() {
  // nothing
  return true;
}

bool StateInit::RunOnce() {
  if (iter_ < 0) return true;
  for (int i = 0; i < consts::model::kNumLeg; ++i)
    SetJPosInterPts(iter_, params::ramp_iter, i, initial_jpos_[i], opts_->init_jpos[i]);
  iter_++;
  if (iter_ >= params::ramp_iter + params::settle_iter) {
    iter_ = -1;
    first_run_ = false;
  }

  return true;
}

bool StateInit::SetJPosInterPts(int const curr_iter, int const max_iter, int const leg, SdVector3f const &ini,
                                SdVector3f const &fin) {
  auto &cmd = legctrl_->GetCmdsForUpdate()[leg];
  math::interpolate_linear(ToEigenTp(cmd.q_des), ToConstEigenTp(ini), ToConstEigenTp(fin),
                           std::fmin(static_cast<fpt_t>(curr_iter) / max_iter, 1.));
  ToEigenTp(cmd.kp_joint).diagonal() = ToConstEigenTp(opts_->kp_joint);
  ToEigenTp(cmd.kd_joint).diagonal() = ToConstEigenTp(opts_->kd_joint);

  spdlog::debug("State Init Set JPos!!\n Leg: {}\n qdes: {}, {}, {}", leg, cmd.q_des[0], cmd.q_des[1], cmd.q_des[2]);

  return true;
}

State StateInit::CheckTransition() {
  if (first_run_) return GetState();
  return state_trans_[drictrl_->GetState()];
}

TransitionData StateInit::Transition([[maybe_unused]] const State next) { return TransitionData{true}; }
}  // namespace sdquadx::fsm

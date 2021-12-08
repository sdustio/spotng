#include "fsm/state/balance_stand.h"

#include <memory>

#include "spdlog/spdlog.h"
#include "wbc/wbic.h"

namespace sdquadx::fsm {

StateBalanceStand::StateBalanceStand(Options::ConstSharedPtr const &opts, LegCtrl::SharedPtr const &legctrl,
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
      wbc_(std::make_unique<wbc::Wbic>(opts, mquad, 1000.)),
      body_weight_(opts->model.mass_total * opts->gravity) {
  estcontact_ = std::dynamic_pointer_cast<estimate::Contact>(estctrl_->GetEstimator("contact"));
}

bool StateBalanceStand::OnEnter() {
  spdlog::info("Enter State Balance Stand!!!");
  auto const &estdata = estctrl_->GetEstState();

  ini_body_pos_ = estdata.pos;

  if (ini_body_pos_[2] < 0.2) {
    ini_body_pos_[2] = 0.25;
  }
  //   ini_body_pos_[2]=0.26;

  ini_body_rpy_ = estdata.rpy;

  estcontact_->UpdateContact({0.5, 0.5, 0.5, 0.5});

  return true;
}

bool StateBalanceStand::OnExit() { /* do nothing*/
  return true;
}

State StateBalanceStand::CheckTransition() {
  if (!SafeCheck()) return State::RecoveryStand;
  return state_trans_[drictrl_->GetState()];
}

bool StateBalanceStand::SafeCheck() {
  auto const &state = estctrl_->GetEstState();
  return (abs(state.rpy[0]) < 0.5 && abs(state.rpy[1]) < 0.5);
}

TransitionData StateBalanceStand::Transition(const State next) {
  if (next == State::Locomotion) {
    RunOnce();
  }
  return TransitionData{true};
}

bool StateBalanceStand::RunOnce() {
  legctrl_->ZeroCmds();

  wbc_data_.body_pos_des = ini_body_pos_;
  wbc_data_.body_lvel_des.fill(0.);
  wbc_data_.body_acc_des.fill(0.);
  wbc_data_.body_rpy_des = ini_body_rpy_;

  auto const &rpy_des = drictrl_->GetRpyDes();
  for (size_t i = 0; i < rpy_des.size(); i++) wbc_data_.body_rpy_des[i] += rpy_des[i];
  if (std::fabs(wbc_data_.body_rpy_des[0]) > opts_->model.max_body_roll)
    wbc_data_.body_rpy_des[0] *= opts_->model.max_body_roll / std::abs(wbc_data_.body_rpy_des[0]);
  if (std::fabs(wbc_data_.body_rpy_des[1]) > opts_->model.max_body_pitch)
    wbc_data_.body_rpy_des[1] *= opts_->model.max_body_pitch / std::abs(wbc_data_.body_rpy_des[1]);
  if (std::fabs(wbc_data_.body_rpy_des[2]) > opts_->model.max_body_yaw)
    wbc_data_.body_rpy_des[2] *= opts_->model.max_body_yaw / std::abs(wbc_data_.body_rpy_des[2]);

  // Height
  wbc_data_.body_pos_des[2] += drictrl_->GetPosDes()[2];
  if (wbc_data_.body_pos_des[2] > opts_->model.max_com_height) wbc_data_.body_pos_des[2] = opts_->model.max_com_height;

  wbc_data_.body_avel_des.fill(0.);

  for (int i = 0; i < consts::model::kNumLeg; ++i) {
    wbc_data_.foot_pos_des[i].fill(0.);
    wbc_data_.foot_lvel_des[i].fill(0.);
    wbc_data_.foot_acc_des[i].fill(0.);
    wbc_data_.Fr_des[i].fill(0.);
    wbc_data_.Fr_des[i][2] = body_weight_ / consts::model::kNumLeg;
    wbc_data_.contact_state[i] = 0.5;
  }

  return wbc_->RunOnce(legctrl_->cmds, wbc_data_, estctrl_->GetEstState());
}
}  // namespace sdquadx::fsm

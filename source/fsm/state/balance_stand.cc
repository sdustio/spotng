#include "fsm/state/balance_stand.h"

#include <memory>

#include "spdlog/spdlog.h"
#include "estimate/contact.h"

namespace sdquadx::fsm {

StateBalanceStand::StateBalanceStand(Options::ConstSharedPtr const &opts, leg::LegCtrl::SharedPtr const &legctrl,
                                     model::Quadruped::SharedPtr const &mquad,
                                     drive::DriveCtrl::SharedPtr const &drictrl,
                                     estimate::EstimateCtrl::SharedPtr const &estctrl)
    : state_trans_{{drive::State::Init, State::Init},
                   {drive::State::RecoveryStand, State::RecoveryStand},
                   {drive::State::Locomotion, State::Locomotion},
                   {drive::State::BalanceStand, State::BalanceStand}},
      legctrl_(legctrl),
      drictrl_(drictrl),
      estctrl_(estctrl),
      wbc_(std::make_unique<wbc::WbcCtrl>(mquad->GetFloatBaseModel(), opts, 1000.)),
      body_weight_(consts::model::kBodyMass * opts->gravity) {}

bool StateBalanceStand::OnEnter() {
  spdlog::debug("Enter State Balance Stand!!!");
  auto const &estdata = estctrl_->GetEstState();

  ini_body_pos_ = estdata.pos;

  if (ini_body_pos_[2] < 0.2) {
    ini_body_pos_[2] = 0.25;
  }
  //   ini_body_pos_[2]=0.26;

  ini_body_rpy_ = estdata.rpy;
  return true;
}

bool StateBalanceStand::OnExit() { /* do nothing*/
  return true;
}
bool StateBalanceStand::RunOnce() {
  auto est_contact = std::dynamic_pointer_cast<estimate::Contact>(estctrl_->GetEstimator("contact"));
  est_contact->UpdateContact({0.5, 0.5, 0.5, 0.5});
  return Step();
}

TransitionData StateBalanceStand::Transition(const State next) {
  if (next == State::Locomotion) {
    Step();
  }
  return TransitionData{true};
}

bool StateBalanceStand::Step() {
  wbc_data_.body_pos_des = ini_body_pos_;
  wbc_data_.body_lvel_des.fill(0.);
  wbc_data_.body_acc_des.fill(0.);
  wbc_data_.body_rpy_des = ini_body_rpy_;

  auto const &rpy_des = drictrl_->GetRpyDes();
  for (size_t i = 0; i < rpy_des.size(); i++) wbc_data_.body_rpy_des[i] += rpy_des[i];

  // Height
  wbc_data_.body_pos_des[2] += drictrl_->GetPosDes()[2];

  wbc_data_.body_avel_des.fill(0.);

  for (int i = 0; i < consts::model::kNumLeg; ++i) {
    wbc_data_.foot_pos_des[i].fill(0.);
    wbc_data_.foot_lvel_des[i].fill(0.);
    wbc_data_.foot_acc_des[i].fill(0.);
    wbc_data_.Fr_des[i].fill(0.);
    wbc_data_.Fr_des[i][2] = body_weight_ / 4.;
    wbc_data_.contact_state[i] = true;
  }

  return wbc_->RunOnce(wbc_data_, legctrl_, drictrl_, estctrl_);
}
}  // namespace sdquadx::fsm

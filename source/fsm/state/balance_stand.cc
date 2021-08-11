#include "fsm/state/balance_stand.h"

namespace sdrobot::fsm
{

  StateBalanceStand::StateBalanceStand(
      Options const &opts,
      leg::LegCtrl::SharedPtr const &legctrl,
      model::Quadruped::SharedPtr const &mquad,
      drive::DriveCtrl::SharedPtr const &drictrl,
      estimate::EstimateCtrl::SharedPtr const &estctrl) : state_trans_{
                                                              {drive::State::Init, State::Init},
                                                              {drive::State::RecoveryStand, State::RecoveryStand},
                                                              {drive::State::Locomotion, State::Locomotion},
                                                              {drive::State::BalanceStand, State::BalanceStand}},
                                                          legctrl_(legctrl), drictrl_(drictrl), estctrl_(estctrl), body_weight_(params::model::kBodyMass * opts.gravity)
  {
    wbc_ = std::make_unique<wbc::WbcCtrl>(mquad->GetFloatBaseModel(), opts, 1000.);
  }

  bool StateBalanceStand::OnEnter()
  {
    auto const &estdata = estctrl_->GetEstState();

    ini_body_pos_ = estdata.pos;

    if (ini_body_pos_[2] < 0.2)
    {
      ini_body_pos_[2] = 0.25;
    }
    //   ini_body_pos_[2]=0.26;

    ini_body_rpy_ = estdata.rpy;
    return true;
  }

  bool StateBalanceStand::OnExit()
  { /* do nothing*/
    return true;
  }
  bool StateBalanceStand::RunOnce()
  {
    return Step();
  }

  TransitionData StateBalanceStand::Transition(const State next)
  {
    if (next == State::Locomotion)
    {
      Step();
    }
    return TransitionData{true};
  }

  bool StateBalanceStand::Step()
  {

    wbc_data_.body_pos_des = ini_body_pos_;
    wbc_data_.body_vel_des.fill(0.);
    wbc_data_.body_acc_des.fill(0.);
    wbc_data_.body_rpy_des = ini_body_rpy_;

    auto const &rpy_des = drictrl_->GetRpyDes();
    for (size_t i = 0; i < rpy_des.size(); i++)
      wbc_data_.body_rpy_des[i] += rpy_des[i];

    // Height
    wbc_data_.body_pos_des[2] += drictrl_->GetPosDes()[2];

    wbc_data_.body_avel_des.fill(0.);

    for (int i = 0; i < params::model::kNumLeg; ++i)
    {
      wbc_data_.foot_pos_des[i].fill(0.);
      wbc_data_.foot_vel_des[i].fill(0.);
      wbc_data_.foot_acc_des[i].fill(0.);
      wbc_data_.Fr_des[i].fill(0.);
      wbc_data_.Fr_des[i][2] = body_weight_ / 4.;
      wbc_data_.contact_state[i] = true;
    }

    return wbc_->RunOnce(wbc_data_, legctrl_, drictrl_, estctrl_);
  }
}

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
                                                          legctrl_(legctrl), drictrl_(drictrl), estctrl_(estctrl),
                                                          body_weight_(params::model::body_mass * opts.gravity)
  {
    wbc_ = std::make_unique<wbc::WbcCtrl>(mquad->GetFloatBaseModel(), opts, 1000.);
  }

  void StateBalanceStand::OnEnter()
  {
    auto const &estdata = estctrl_->GetEstState();

    ini_body_pos_ = estdata.pos;

    if (ini_body_pos_[2] < 0.2)
    {
      ini_body_pos_[2] = 0.25;
    }
    //   ini_body_pos_[2]=0.26;

    ini_body_pos_rpy_ = estdata.pos_rpy;
  }

  void StateBalanceStand::OnExit()
  { /* do nothing*/
  }
  bool StateBalanceStand::RunOnce()
  {
    Step();
    return true;
  }

  TransitionData StateBalanceStand::Transition(const State next)
  {
    if (next == State::Locomotion)
    {
      Step();
    }
    return TransitionData{true};
  }

  void StateBalanceStand::Step()
  {

    wbc_data_.pos_body_des = ini_body_pos_;
    wbc_data_.vel_body_des.fill(0.);
    wbc_data_.acc_body_des.fill(0.);
    wbc_data_.pos_rpy_body_des = ini_body_pos_rpy_;

    wbc_data_.pos_rpy_body_des = drictrl_->GetPosRpyDes();

    // Height
    wbc_data_.pos_body_des[2] += drictrl_->GetPosDes()[2];

    wbc_data_.vel_rpy_body_des.fill(0.);

    for (int i = 0; i < params::model::num_leg; ++i)
    {
      wbc_data_.pos_foot_des[i].fill(0.);
      wbc_data_.vel_foot_des[i].fill(0.);
      wbc_data_.acc_foot_des[i].fill(0.);
      wbc_data_.Fr_des[i].fill(0.);
      wbc_data_.Fr_des[i][2] = body_weight_ / 4.;
      wbc_data_.contact_state[i] = true;
    }

    wbc_->Run(wbc_data_, legctrl_, drictrl_, estctrl_);
  }
}

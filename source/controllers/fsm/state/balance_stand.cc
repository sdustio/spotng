#include "controllers/fsm/state/balance_stand.h"

namespace sd::ctrl::fsm
{

  StateBalanceStand::StateBalanceStand(
      const LegPtr &cleg, const robot::QuadrupedPtr &quad, const StateCmdPtr &cmd,
      const est::StateEstPtr &est) : StateCtrl(cleg, quad, cmd, est),
                                     state_trans_{
                                         {robot::Mode::Init, State::Init},
                                         {robot::Mode::RecoveryStand, State::RecoveryStand},
                                         {robot::Mode::Locomotion, State::Locomotion},
                                         {robot::Mode::BalanceStand, State::BalanceStand}},
                                     body_weight_(robot::ModelAttrs::body_mass * 9.81)
  {
    //TODO  check
    // Initialize GRF to 0s
    // this->footFeedForwardForces = Mat34<T>::Zero();

    wbc_ = std::make_shared<Wbc>(quad_->BuildModel(), 1000.);
  }

  void StateBalanceStand::OnEnter()
  {
    ini_body_pos_ = state_est_->GetData().position;

    if (ini_body_pos_[2] < 0.2)
    {
      ini_body_pos_[2] = 0.25;
    }
    //   ini_body_pos_[2]=0.26;
  }

  void StateBalanceStand::OnExit()
  { /* do nothing*/
  }
  bool StateBalanceStand::Run()
  {
    // TODO check contact
    // Vector4d contactState(0.5, 0.5, 0.5, 0.5);
    // this->_data->_stateEstimator->setContactPhase(contactState);
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

    wbc_data_.pBody_des = ini_body_pos_;
    wbc_data_.vBody_des.setZero();
    wbc_data_.aBody_des.setZero();

    auto &des = state_cmd_->GetStateDes();
    wbc_data_.pBody_RPY_des[0] = des(StateIdx::angle_r);
    wbc_data_.pBody_RPY_des[1] = des(StateIdx::angle_p);
    wbc_data_.pBody_RPY_des[2] = des(StateIdx::angle_y);

    // Height
    wbc_data_.pBody_des[2] += des(StateIdx::pos_z);

    wbc_data_.vBody_Ori_des.setZero();

    for (size_t i = 0; i < robot::ModelAttrs::num_leg; ++i)
    {
      wbc_data_.pFoot_des[i].setZero();
      wbc_data_.vFoot_des[i].setZero();
      wbc_data_.aFoot_des[i].setZero();
      wbc_data_.Fr_des[i].setZero();
      wbc_data_.Fr_des[i][2] = body_weight_ / 4.;
      wbc_data_.contact_state[i] = true;
    }

    wbc_->Run(wbc_data_, state_est_, leg_ctrl_);
  }

} // namespace sd::ctrl::fsm

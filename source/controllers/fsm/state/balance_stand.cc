#include "controllers/fsm/state/balance_stand.h"

namespace sd::ctrl::fsm
{

  StateBalanceStand::StateBalanceStand(
      LegPtr &cleg, const StateCmdPtr &cmd,
      const est::StateEstPtr &est) : StateCtrl(cleg, cmd, est),
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

    // _wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData->_quadruped->buildModel());
    // _wbc_data = new LocomotionCtrlData<T>();

    // _wbc_ctrl->setFloatingBaseWeight(1000.);
  }

  void StateBalanceStand::OnEnter()
  {
    ini_body_pos_ = state_est_->GetData().position;

    if (ini_body_pos_[2] < 0.2)
    {
      ini_body_pos_[2] = 0.25;
    }
    //   ini_body_pos_[2]=0.26;

    last_height_cmd_ = ini_body_pos_[2];

    ini_body_ori_rpy_ = state_est_->GetData().rpy;
  }

  void StateBalanceStand::OnExit() {/* do nothing*/}
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

  void StateBalanceStand::Step() {}

} // namespace sd::ctrl::fsm

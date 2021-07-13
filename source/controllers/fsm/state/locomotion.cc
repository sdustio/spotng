#include "sdrobot/dynamics/math.h"
#include "sdrobot/robot/runner.h"

#include "controllers/fsm/state/locomotion.h"
#include "controllers/mpc/cmpc.h"

namespace sdrobot::ctrl::fsm
{

  StateLocomotion::StateLocomotion(
      const LegPtr &cleg, const robot::QuadrupedPtr &quad, const StateCmdPtr &cmd,
      const est::StateEstPtr &est) : StateCtrl(cleg, quad, cmd, est),
                                     state_trans_{
                                         {robot::Mode::Init, State::Init},
                                         {robot::Mode::RecoveryStand, State::RecoveryStand},
                                         {robot::Mode::Locomotion, State::Locomotion},
                                         {robot::Mode::BalanceStand, State::BalanceStand}}
  {
    cMPCOld = std::make_shared<mpc::CMpc>(robot::ctrlparams::kCtrlsec, 30 / (1000. * robot::ctrlparams::kCtrlsec));

    // Initialize GRF and footstep locations to 0s
    footstepLocations = Matrix3x4d::Zero();
    _wbc_ctrl = std::make_shared<Wbc>(quad_->BuildModel());
  }

  void StateLocomotion::OnEnter() {}
  void StateLocomotion::OnExit() {}
  bool StateLocomotion::Run()
  {
    return true;
  }

  TransitionData StateLocomotion::Transition(const State next)
  {
    if (next == State::BalanceStand)
    {
      LocomotionControlStep();
    }

    return TransitionData{true};
  }

  State StateLocomotion::CheckTransition()
  {
    if (locomotionSafe())
    {
      return state_trans_[state_cmd_->GetMode()];
    }
    auto cmd = state_cmd_;
    cmd->SetMode(robot::Mode::RecoveryStand);
    return State::RecoveryStand;
  }

  // Parses contact specific controls to the leg controller
  void StateLocomotion::LocomotionControlStep() {}

  bool StateLocomotion::locomotionSafe()
  {
    const auto &seResult = state_est_->GetData();

    const double max_roll = 80.;  //40;
    const double max_pitch = 80.; //40;

    if (std::fabs(seResult.rpy[0]) > dynamics::DegToRad(max_roll))
    {
      // printf("Unsafe locomotion: roll is %.3f degrees (max %.3f)\n", dynamics::RadToDeg(seResult.rpy[0]), max_roll);
      return false;
    }

    if (std::fabs(seResult.rpy[1]) > dynamics::DegToRad(max_pitch))
    {
      // printf("Unsafe locomotion: pitch is %.3f degrees (max %.3f)\n", dynamics::RadToDeg(seResult.rpy[1]), max_pitch);
      return false;
    }

    for (size_t leg = 0; leg < 4; leg++)
    {
      const auto &leg_data = leg_ctrl_->GetDatas()[leg];

      if (leg_data.p[2] > 0)
      {
        // printf("Unsafe locomotion: leg %d is above hip (%.3f m)\n", leg, leg_data.p[2]);
        return false;
      }

      if (std::fabs(leg_data.p[1] > 0.28)) //0.18))
      {
        // printf("Unsafe locomotion: leg %d's y-position is bad (%.3f m)\n", leg, leg_data.p[1]);
        return false;
      }

      auto v_leg = leg_data.v.norm();

      if (std::fabs(v_leg) > 19.)
      {
        // printf("Unsafe locomotion: leg %d is moving too quickly (%.3f m/s)\n", leg, v_leg);
        return false;
      }
    }

    return true;
  }

  // Impedance control for the stance legs during locomotion
  void StateLocomotion::StanceLegImpedanceControl(size_t leg) {}

} // namespace sdrobot::ctrl::fsm

#include "fsm/state/locomotion.h"
#include "mpc/cmpc.h"
#include "drive/drive_ctrl_impl.h"
#include "math/utils.h"

namespace sdrobot::fsm
{
  namespace opts
  {
    constexpr inline double const max_roll = 80.;  //40;
    constexpr inline double const max_pitch = 80.; //40;

    constexpr inline SdMatrix3f const kp_stance = {};
    constexpr inline SdMatrix3f const kd_stance = {14, 0, 0, 0, 14, 0, 0, 0, 14};
  } // namespace opts

  StateLocomotion::StateLocomotion(
      Options const &opts,
      leg::LegCtrl::SharedPtr const &legctrl,
      model::Quadruped::SharedPtr const &mquat,
      drive::DriveCtrl::SharedPtr const &drictrl,
      estimate::EstimateCtrl::SharedPtr const &estctrl) : state_trans_{
                                                              {drive::State::Init, State::Init},
                                                              {drive::State::RecoveryStand, State::RecoveryStand},
                                                              {drive::State::Locomotion, State::Locomotion},
                                                              {drive::State::BalanceStand, State::BalanceStand}},
                                                          legctrl_(legctrl), mquat_(mquat), drictrl_(drictrl), estctrl_(estctrl)
  {
    mpc_ = std::make_unique<mpc::CMpc>(opts.ctrl_dt_sec, opts.gravity, 30 / (1000. * opts.ctrl_dt_sec));

    // Initialize GRF and footstep locations to 0s
    // footstepLocations = Matrix3x4::Zero();
    wbc_ = std::make_unique<wbc::WbcCtrl>(mquat->GetFloatBaseModel(), opts);
  }

  void StateLocomotion::OnEnter()
  {
    mpc_->Init();
  }

  void StateLocomotion::OnExit() {}

  bool StateLocomotion::RunOnce()
  {
    // Call the locomotion control logic for this iteration
    LocomotionControlStep();
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
      return state_trans_[drictrl_->GetState()];
    }

    std::dynamic_pointer_cast<drive::DriveCtrlImpl>(drictrl_)->UpdateState(drive::State::RecoveryStand);
    return State::RecoveryStand;
  }

  // Parses contact specific controls to the leg controller
  void StateLocomotion::LocomotionControlStep()
  {
    // StateEstimate<T> stateEstimate = this->_data->_stateEstimator->getResult();

    // Contact state logic
    // estimateContact();

    mpc_->Run(wbc_data_, legctrl_, mquat_, drictrl_, estctrl_);

    std::array<SdVector3f, 4> pDes_backup;
    std::array<SdVector3f, 4> vDes_backup;

    auto &leg_cmd = legctrl_->GetCmdsForUpdate();

    for (int leg(0); leg < 4; ++leg)
    {
      pDes_backup[leg] = leg_cmd[leg].p_des;
      vDes_backup[leg] = leg_cmd[leg].v_des;
    }

    wbc_->Run(wbc_data_, legctrl_, drictrl_, estctrl_);

    for (int leg(0); leg < 4; ++leg)
    {
      leg_cmd[leg].p_des = pDes_backup[leg];
      leg_cmd[leg].v_des = vDes_backup[leg];
      leg_cmd[leg].kp_cartesian = opts::kp_stance;
      leg_cmd[leg].kd_cartesian = opts::kd_stance;
    }
  }

  bool StateLocomotion::locomotionSafe()
  {
    const auto &seResult = estctrl_->GetEstState();

    if (std::fabs(seResult.pos_rpy[0]) > math::DegToRad(opts::max_roll))
    {
      // printf("Unsafe locomotion: roll is %.3f degrees (max %.3f)\n", math::RadToDeg(seResult.rpy[0]), max_roll);
      return false;
    }

    if (std::fabs(seResult.pos_rpy[1]) > math::DegToRad(opts::max_pitch))
    {
      // printf("Unsafe locomotion: pitch is %.3f degrees (max %.3f)\n", math::RadToDeg(seResult.rpy[1]), max_pitch);
      return false;
    }

    for (int leg = 0; leg < 4; leg++)
    {
      auto const &leg_data = legctrl_->GetDatas()[leg];

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

      auto v_leg = ToConstEigenTp(leg_data.v).norm();

      if (std::fabs(v_leg) > 19.)
      {
        // printf("Unsafe locomotion: leg %d is moving too quickly (%.3f m/s)\n", leg, v_leg);
        return false;
      }
    }

    return true;
  }
} // namespace sdrobot::fsm

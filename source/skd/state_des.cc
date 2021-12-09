#include "skd/state_des.h"

#include "dynamics/rotation.h"

namespace sdquadx::skd {

namespace params {
constexpr fpt_t const kMaxFootPosRel = 0.35;
}  // namespace params

StateDes::StateDes(Options::ConstSharedPtr const &opts) : opts_(opts) {}

bool StateDes::RunOnce(wbc::InData &wbcdata, estimate::State const &estdata,
                       drive::DriveCtrl::ConstSharedPtr const &drivectrl, Gait::ConstSharedPtr const &gait_skd) {
  auto rot_mat = ToConstEigenTp(estdata.rot_mat);
  auto lvel = ToConstEigenTp(estdata.lvel);
  auto pos = ToConstEigenTp(estdata.pos);
  auto rpy = ToConstEigenTp(estdata.rpy);

  auto lvel_des_robot = ToConstEigenTp(drivectrl->GetLvelDes());
  Vector3 lvel_des = rot_mat.transpose() * lvel_des_robot;

  // avel_des_robot = [0, 0, wz]，所以 avel_des ~= avel_des_robot
  auto avel_des = ToConstEigenTp(drivectrl->GetAvelDes());

  // some first time initialization
  if (first_run_) {
    pos_des_[0] = estdata.pos[0];
    pos_des_[1] = estdata.pos[1];
    pos_des_[2] = opts_->model.basic_locomotion_height;  // ?? estdata.rpy[2];

    for (int i = 0; i < 4; i++) {
      foot_swing_trajs_[i].UpdateHeight(0.05);
      foot_swing_trajs_[i].UpdateInitialPosition(estdata.foot_pos[i]);
      foot_swing_trajs_[i].UpdateFinalPosition(estdata.foot_pos[i]);
    }
    first_run_ = false;
  }

  auto _body_height = opts_->model.basic_locomotion_height + drivectrl->GetPosDes()[2];
  if (_body_height < 0.02) {
    _body_height = opts_->model.basic_locomotion_height;
  }
  if (drivectrl->GetGait() == drive::Gait::FlyingTrot) _body_height = opts_->model.fast_locomotion_height;
  ToEigenTp(pos_des_) += opts_->ctrl_sec * lvel_des;
  pos_des_[2] = _body_height;

  // Integral-esque pitche and roll compensation
  if (fabs(lvel[0]) > .02) {  // avoid dividing by zero
    rpy_int_[1] += 5 * opts_->ctrl_sec * (0. - rpy[1]) / lvel[0];
  }
  if (fabs(lvel[1]) > 0.01) {
    rpy_int_[0] += opts_->ctrl_sec * (0. - rpy[0]) / lvel[1];
  }
  rpy_int_[0] = std::fmin(std::fmax(rpy_int_[0], -.25), .25);
  rpy_int_[1] = std::fmin(std::fmax(rpy_int_[1], -.25), .25);

  SdVector3f rpy_des = {lvel[1] * rpy_int_[0], lvel[0] * rpy_int_[1], rpy[2] + opts_->ctrl_sec * avel_des[2]};

  wbcdata.body_pos_des = pos_des_;
  wbcdata.body_lvel_des = {lvel_des[0], lvel_des[1], lvel_des[2]};
  wbcdata.body_acc_des.fill(0.);

  wbcdata.body_rpy_des = rpy_des;
  wbcdata.body_avel_des = {avel_des[0], avel_des[1], avel_des[2]};
  gait_skd->CalcStancePhase(wbcdata.contact_state);

  SdVector4f swingStates;
  gait_skd->CalcSwingPhase(swingStates);

  // foot placement
  for (int i = 0; i < consts::model::kNumLeg; i++) {
    swing_times_[i] = gait_skd->GetCurrentSwingTime(i);
    if (first_swing_[i]) {
      swing_time_remaining_[i] = swing_times_[i];
    } else {
      swing_time_remaining_[i] -= opts_->ctrl_sec;
    }
    // if(first_swing_[i]) {
    // foot_swing_trajs_[i].UpdateHeight(.05);

    foot_swing_trajs_[i].UpdateHeight(drivectrl->GetStepHeight());  // .125);

    fpt_t sign_fh = consts::model::kSignFH[i];
    fpt_t sign_lr = consts::model::kSignLR[i];

    fpt_t hx = opts_->model.location_abad_fl[0];
    fpt_t hy = opts_->model.location_abad_fl[1];
    fpt_t l1 = opts_->model.link_length_abad;

    Vector3 pos_hip_robot(sign_fh * hx, sign_lr * (hy + l1), 0.);
    Vector3 offset(0, sign_lr * opts_->model.foot_offsety, 0);

    if (i > 1) offset[0] = opts_->model.foot_offsetx;  // 0.02;//
    if (drivectrl->GetGait() == drive::Gait::Walk) {   // walk gait
      if (i < 2) offset(1) = sign_lr * 0.085 * (1 - fabs(lvel_des_robot[0]) / 2.0);
    }
    pos_hip_robot += offset;

    fpt_t stance_time = gait_skd->GetCurrentStanceTime(i);
    Matrix3 _rot;
    dynamics::CoordinateRot(_rot, dynamics::CoordinateAxis::Z, -avel_des[2] * stance_time / 2);

    Vector3 Pf = pos + rot_mat.transpose() * (lvel_des_robot * swing_time_remaining_[i] + _rot * pos_hip_robot);

    // Using the estimated velocity is correct
    // Vector3 lvel_des = estdata.rot_mat.transpose() * lvel_des_robot;
    fpt_t pfx_rel = lvel[0] * (.5 + opts_->ctrl.footskd_bonus_swing) * stance_time +
                    opts_->ctrl.footskd_vkp * (lvel[0] - lvel_des[0]) +
                    (0.5 * pos[2] / opts_->gravity) * (lvel[1] * avel_des[2]);

    if (fabs(pfx_rel) > params::kMaxFootPosRel) printf("!!!!!!!!!!!!!!!!out of the max step\n");

    fpt_t pfy_rel = lvel[1] * .5 * stance_time /* * dt_mpc_ */ + opts_->ctrl.footskd_vkp * (lvel[1] - lvel_des[1]) +
                    (0.5 * pos[2] / opts_->gravity) * (-lvel[0] * avel_des[2]);
    pfx_rel = std::fmin(std::fmax(pfx_rel, -params::kMaxFootPosRel), params::kMaxFootPosRel);
    pfy_rel = std::fmin(std::fmax(pfy_rel, -params::kMaxFootPosRel), params::kMaxFootPosRel);
    Pf[0] += pfx_rel;
    Pf[1] += pfy_rel;
    // TODO(Michael) 估计俯仰角
    // Pf[2] = -0.01;
    Pf[2] = 0.0;
    foot_swing_trajs_[i].UpdateFinalPosition({Pf[0], Pf[1], Pf[2]});

    fpt_t swingState = swingStates[i];
    if (swingState > consts::math::kZeroEpsilon) {  // foot is in swing
      if (first_swing_[i]) {
        first_swing_[i] = false;
        foot_swing_trajs_[i].UpdateInitialPosition(estdata.foot_pos[i]);
      }

      foot_swing_trajs_[i].ComputeSwingTrajectoryBezier(swingState, swing_times_[i]);

      // Update for WBC
      wbcdata.foot_pos_des[i] = foot_swing_trajs_[i].GetPosition();
      wbcdata.foot_lvel_des[i] = foot_swing_trajs_[i].GetVelocity();
      wbcdata.foot_acc_des[i] = foot_swing_trajs_[i].GetAcceleration();
    } else {  // foot is in stance
      first_swing_[i] = true;
    }
  }

  return true;
}
}  // namespace sdquadx::skd

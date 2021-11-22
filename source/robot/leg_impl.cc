#include "robot/leg_impl.h"

#include <cmath>

#include "externlib/eigen.h"

namespace sdquadx::leg {

namespace {
std::array<fpt_t, 4> const side_signs{-1.0, 1.0, -1.0, 1.0};

/*!
 * Get if the i-th leg is on the left (+) or right (-) of the robot.
 * 判断第i条腿是在机器人的左边(+)还是右边(-)。
 * @param leg : the leg index
 * @return The side sign (-1 for right legs, +1 for left legs)
 */
fpt_t GetSideSign(int leg) { return side_signs.at(leg); }
}  // namespace

void Cmd::Zero() {
  tau_feed_forward.fill(0.);
  q_des.fill(0.);
  qd_des.fill(0.);
  kp_joint.fill(0.);
  kd_joint.fill(0.);
}

void Data::Zero() {
  q.fill(0.);
  qd.fill(0.);
  tau_estimate.fill(0.);
}

LegCtrlImpl::LegCtrlImpl(interface::ActuatorInterface::SharedPtr const &act_itf, Options::ConstSharedPtr const &opts)
    : act_itf_(act_itf), opts_(opts) {}

Datas const &LegCtrlImpl::GetDatas() const { return datas_; }

Cmds &LegCtrlImpl::GetCmdsForUpdate() { return cmds_; }

bool LegCtrlImpl::UpdateCmds(Cmds const &cmds) {
  cmds_ = cmds;
  return true;
}
bool LegCtrlImpl::UpdateCmd(int leg, Cmd const &cmd) {
  cmds_.at(leg) = cmd;
  return true;
}

void LegCtrlImpl::ZeroCmd() {
  for (auto &cmd : cmds_) {
    cmd.Zero();
  }
}

bool LegCtrlImpl::UpdateDatasFromActuatorInterface() {
  for (int leg = 0; leg < consts::model::kNumLeg; leg++) {
    auto const &data = act_itf_->GetActuatorData();
    // q: 关节角
    datas_[leg].q[0] = data.q_abad[leg];
    datas_[leg].q[1] = data.q_hip[leg];
    datas_[leg].q[2] = data.q_knee[leg];

    // qd 关节角速度
    datas_[leg].qd[0] = data.qd_abad[leg];
    datas_[leg].qd[1] = data.qd_hip[leg];
    datas_[leg].qd[2] = data.qd_knee[leg];
  }
  return true;
}

bool LegCtrlImpl::SendCmdsToActuatorInterface() {
  auto &cmd = act_itf_->GetActuatorCmdForUpdate();

  for (int leg = 0; leg < consts::model::kNumLeg; leg++) {
    // tauFF 获得从控制器来的力矩
    auto const &tff = cmds_[leg].tau_feed_forward;
    Vector3 leg_torque(tff[0], tff[1], tff[2]);

    // set command: 命令设置 设置力矩
    cmd.tau_abad_ff[leg] = tff[0];
    cmd.tau_hip_ff[leg] = tff[1];
    cmd.tau_knee_ff[leg] = tff[2];

    // joint space pd
    auto kp_joint = ToConstEigenTp(cmds_[leg].kp_joint);
    cmd.kp_abad[leg] = kp_joint(0, 0);
    cmd.kp_hip[leg] = kp_joint(1, 1);
    cmd.kp_knee[leg] = kp_joint(2, 2);

    auto kd_joint = ToConstEigenTp(cmds_[leg].kd_joint);
    cmd.kd_abad[leg] = kd_joint(0, 0);
    cmd.kd_hip[leg] = kd_joint(1, 1);
    cmd.kd_knee[leg] = kd_joint(2, 2);

    auto q = ToConstEigenTp(datas_[leg].q);
    auto q_des = ToConstEigenTp(cmds_[leg].q_des);
    cmd.q_des_abad[leg] = q_des[0];
    cmd.q_des_hip[leg] = q_des[1];
    cmd.q_des_knee[leg] = q_des[2];

    auto qd = ToConstEigenTp(datas_[leg].qd);
    auto qd_des = ToConstEigenTp(cmds_[leg].qd_des);
    cmd.qd_des_abad[leg] = qd_des[0];
    cmd.qd_des_hip[leg] = qd_des[1];
    cmd.qd_des_knee[leg] = qd_des[2];

    // estimate torque
    ToEigenTp(datas_[leg].tau_estimate) = leg_torque + kp_joint * (q_des - q) + kd_joint * (qd_des - qd);

    cmd.flags[leg] = 1;
  }

  return true;
}
}  // namespace sdquadx::leg

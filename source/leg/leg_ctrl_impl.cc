#include "leg/leg_ctrl_impl.h"

#include <cmath>

#include "eigen/types.h"
#include "sdrobot/consts.h"

namespace sdrobot::leg {

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

LegCtrlImpl::LegCtrlImpl(interface::ActuatorInterface::SharedPtr const &act_itf)
    : act_itf_(act_itf) {}

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

bool LegCtrlImpl::ComputeLegJacobianAndPosition(int leg) {
  fpt_t l1 = consts::model::kAbadLinkLength;
  fpt_t l2 = consts::model::kHipLinkLength;
  fpt_t l3 = consts::model::kKneeLinkLength;
  fpt_t l4 = consts::model::kKneeLinkYOffset;
  fpt_t side_sign = GetSideSign(leg);  // also check bounds

  auto const &q = datas_[leg].q;
  fpt_t s1 = std::sin(q[0]);
  fpt_t s2 = std::sin(q[1]);
  fpt_t s3 = std::sin(q[2]);

  fpt_t c1 = std::cos(q[0]);
  fpt_t c2 = std::cos(q[1]);
  fpt_t c3 = std::cos(q[2]);

  fpt_t c23 = c2 * c3 - s2 * s3;
  fpt_t s23 = s2 * c3 + c2 * s3;

  auto J = ToEigenTp(datas_[leg].J);
  J(0, 0) = 0;
  J(0, 1) = l3 * c23 + l2 * c2;
  J(0, 2) = l3 * c23;
  J(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - (l1 + l4) * side_sign * s1;
  J(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
  J(1, 2) = -l3 * s1 * s23;
  J(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + (l1 + l4) * side_sign * c1;
  J(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
  J(2, 2) = l3 * c1 * s23;

  auto &p = datas_[leg].p;
  p[0] = l3 * s23 + l2 * s2;
  p[1] = (l1 + l4) * side_sign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
  p[2] = (l1 + l4) * side_sign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;

  return true;
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

    // J and p 雅可比和足端位置
    ComputeLegJacobianAndPosition(leg);

    // v 足端速度
    ToEigenTp(datas_[leg].v) =
        ToConstEigenTp(datas_[leg].J) * ToConstEigenTp(datas_[leg].qd);
  }
  return true;
}

bool LegCtrlImpl::SendCmdsToActuatorInterface() {
  auto &cmd = act_itf_->GetActuatorCmdForUpdate();

  for (int leg = 0; leg < consts::model::kNumLeg; leg++) {
    // tauFF 获得从控制器来的力矩
    auto const &tff = cmds_[leg].tau_feed_forward;
    Vector3 leg_torque(tff[0], tff[1], tff[2]);

    // forceFF 获得从控制器来的力矩
    auto const &fff = cmds_[leg].force_feed_forward;
    Vector3 foot_force(fff[0], fff[1], fff[2]);

    // cartesian PD 直角坐标下pd
    foot_force +=
        ToConstEigenTp(cmds_[leg].kp_cartesian) *
        (ToConstEigenTp(cmds_[leg].p_des) - ToConstEigenTp(datas_[leg].p));

    foot_force +=
        ToConstEigenTp(cmds_[leg].kd_cartesian) *
        (ToConstEigenTp(cmds_[leg].v_des) - ToConstEigenTp(datas_[leg].v));

    // Torque 足力转换成力矩
    leg_torque += ToConstEigenTp(datas_[leg].J).transpose() * foot_force;

    // set command: 命令设置 设置力矩
    cmd.tau_abad_ff[leg] = leg_torque(0);
    cmd.tau_hip_ff[leg] = leg_torque(1);
    cmd.tau_knee_ff[leg] = leg_torque(2);

    // joint space pd
    auto kp_joint = ToConstEigenTp(cmds_[leg].kp_joint);
    cmd.kp_abad[leg] = kp_joint(0, 0);
    cmd.kp_hip[leg] = kp_joint(1, 1);
    cmd.kp_knee[leg] = kp_joint(2, 2);

    auto kd_joint = ToConstEigenTp(cmds_[leg].kd_joint);
    cmd.kd_abad[leg] = kd_joint(0, 0);
    cmd.kd_hip[leg] = kd_joint(1, 1);
    cmd.kd_knee[leg] = kd_joint(2, 2);

    auto q_des = ToConstEigenTp(cmds_[leg].q_des);
    auto q = ToConstEigenTp(datas_[leg].q);
    cmd.q_des_abad[leg] = q_des[0];
    cmd.q_des_hip[leg] = q_des[1];
    cmd.q_des_knee[leg] = q_des[2];

    auto qd_des = ToConstEigenTp(cmds_[leg].qd_des);
    auto qd = ToConstEigenTp(datas_[leg].qd);
    cmd.qd_des_abad[leg] = qd_des[0];
    cmd.qd_des_hip[leg] = qd_des[1];
    cmd.qd_des_knee[leg] = qd_des[2];

    // estimate torque
    ToEigenTp(datas_[leg].tau_estimate) =
        leg_torque + kp_joint * (q_des - q) + kd_joint * (qd_des - qd);

    cmd.flags[leg] = 1;
  }

  return true;
}
}  // namespace sdrobot::leg

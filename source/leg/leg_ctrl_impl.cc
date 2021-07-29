#include <cmath>

#include "leg/leg_ctrl_impl.h"
#include "params.h"
#include "eigen.h"

namespace sdrobot::leg
{
  LegCtrlImpl::LegCtrlImpl(
      interface::ActuatorInterface::SharedPtr const &act_itf) : act_itf_(act_itf) {}

  const Datas &LegCtrlImpl::GetDatas() const
  {
    return datas_;
  }

  Cmds &LegCtrlImpl::GetCmdsForUpdate()
  {
    return cmds_;
  }

  bool LegCtrlImpl::UpdateCmds(Cmds const &cmds)
  {
    cmds_ = cmds;
    return true;
  }
  bool LegCtrlImpl::UpdateCmd(int leg, Cmd const &cmd)
  {
    cmds_.at(leg) = cmd;
    return true;
  }

  void LegCtrlImpl::ZeroCmd()
  {
    for (auto &cmd : cmds_)
    {
      cmd.Zero();
    }
  }

  void LegCtrlImpl::ComputeLegJacobianAndPosition(int leg)
  {
    double l1 = params::model::abad_link_length;
    double l2 = params::model::hip_link_length;
    double l3 = params::model::knee_link_length;
    double l4 = params::model::knee_link_y_offset;
    double side_sign = GetSideSign(leg); //also check bounds

    auto const &q = datas_[leg].q;
    double s1 = std::sin(q[0]);
    double s2 = std::sin(q[1]);
    double s3 = std::sin(q[2]);

    double c1 = std::cos(q[0]);
    double c2 = std::cos(q[1]);
    double c3 = std::cos(q[2]);

    double c23 = c2 * c3 - s2 * s3;
    double s23 = s2 * c3 + c2 * s3;

    Eigen::Map<Matrix3> J(datas_[leg].J.data());
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
  }

  void LegCtrlImpl::UpdateDatasFromActuatorInterface()
  {
    for (int leg = 0; leg < params::model::num_leg; leg++)
    {
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
      Eigen::Map<Vector3> v(datas_[leg].v.data());
      Eigen::Map<Matrix3 const> J(datas_[leg].J.data());
      Eigen::Map<Vector3 const> qd(datas_[leg].qd.data());
      v = J * qd;
    }
  }

  void LegCtrlImpl::SendCmdsToActuatorInterface()
  {
    auto &cmd = act_itf_->GetActuatorCmdForUpdate();

    for (int leg = 0; leg < params::model::num_leg; leg++)
    {
      // tauFF 获得从控制器来的力矩
      auto const &tff = cmds_[leg].tau_feed_forward;
      Vector3 leg_torque(tff[0], tff[1], tff[2]);

      // forceFF 获得从控制器来的力矩
      auto const &fff = cmds_[leg].force_feed_forward;
      Vector3 foot_force(fff[0], fff[1], fff[2]);

      // cartesian PD 直角坐标下pd
      Eigen::Map<Vector3 const> p_des(cmds_[leg].p_des.data());
      Eigen::Map<Vector3 const> p(datas_[leg].p.data());
      Eigen::Map<Matrix3 const> kp_cartesian(cmds_[leg].kp_cartesian.data());
      foot_force += kp_cartesian * (p_des - p);

      Eigen::Map<Vector3 const> v_des(cmds_[leg].v_des.data());
      Eigen::Map<Vector3 const> v(datas_[leg].v.data());
      Eigen::Map<Matrix3 const> kd_cartesian(cmds_[leg].kd_cartesian.data());
      foot_force += kd_cartesian * (v_des - v);

      // Torque 足力转换成力矩
      Eigen::Map<Matrix3 const> J(datas_[leg].J.data());
      leg_torque += J.transpose() * foot_force;

      // set command: 命令设置 设置力矩
      cmd.tau_abad_ff[leg] = leg_torque(0);
      cmd.tau_hip_ff[leg] = leg_torque(1);
      cmd.tau_knee_ff[leg] = leg_torque(2);

      // joint space pd
      Eigen::Map<Matrix3 const> kp_joint(cmds_[leg].kp_joint.data());
      cmd.kp_abad[leg] = kp_joint(0, 0);
      cmd.kp_hip[leg] = kp_joint(1, 1);
      cmd.kp_knee[leg] = kp_joint(2, 2);

      Eigen::Map<Matrix3 const> kd_joint(cmds_[leg].kd_joint.data());
      cmd.kd_abad[leg] = kd_joint(0, 0);
      cmd.kd_hip[leg] = kd_joint(1, 1);
      cmd.kd_knee[leg] = kd_joint(2, 2);

      Eigen::Map<Vector3 const> q_des(cmds_[leg].q_des.data());
      Eigen::Map<Vector3 const> q(datas_[leg].q.data());
      cmd.q_des_abad[leg] = q_des[0];
      cmd.q_des_hip[leg] = q_des[1];
      cmd.q_des_knee[leg] = q_des[2];

      Eigen::Map<Vector3 const> qd_des(cmds_[leg].qd_des.data());
      Eigen::Map<Vector3 const> qd(datas_[leg].qd.data());
      cmd.qd_des_abad[leg] = qd_des[0];
      cmd.qd_des_hip[leg] = qd_des[1];
      cmd.qd_des_knee[leg] = qd_des[2];

      // estimate torque
      Eigen::Map<Vector3> tau_estimate(datas_[leg].tau_estimate.data());
      tau_estimate = leg_torque +
                     kp_joint * (q_des - q) +
                     kd_joint * (qd_des - qd);

      cmd.flags[leg] = 1;
    }
  }

  bool LegCtrl::Make(LegCtrl::Ptr &ret, interface::ActuatorInterface::SharedPtr const &act_itf)
  {
    ret = std::make_unique<LegCtrlImpl>(act_itf);
    return true;
  }
  bool LegCtrl::Make(LegCtrl::SharedPtr &ret, interface::ActuatorInterface::SharedPtr const &act_itf)
  {
    ret = std::make_shared<LegCtrlImpl>(act_itf);
    return true;
  }
}

#include "sd/controllers/leg_ctrl.h"

namespace sd::ctrl
{
  void LegCtrl::UpdateData(const robot::SPIData &data)
  {
    for (int leg = 0; leg < 4; leg++)
    {
      // q: 关节角
      datas_.at(leg).q(0) = data.q_abad.at(leg);
      datas_.at(leg).q(1) = data.q_hip.at(leg);
      datas_.at(leg).q(2) = data.q_knee.at(leg);

      // qd 关节角速度？
      datas_.at(leg).qd(0) = data.qd_abad.at(leg);
      datas_.at(leg).qd(1) = data.qd_hip.at(leg);
      datas_.at(leg).qd(2) = data.qd_knee.at(leg);

      // J and p 雅可比和足端位置
      ComputeLegJacobianAndPosition(leg);

      // v 足端速度
      datas_.at(leg).v = datas_.at(leg).J * datas_.at(leg).qd;
    }
  }

  void LegCtrl::UpdateSPICmd(robot::SPICmd &cmd)
  {
    for (int leg = 0; leg < 4; leg++)
    {
      // tauFF 获得从控制器来的力矩
      Vector3d leg_torque = cmds_.at(leg).tau_feed_forward;

      // forceFF 获得从控制器来的力矩
      Vector3d foot_force = cmds_.at(leg).force_feed_forward;

      // cartesian PD 直角坐标下pd
      foot_force +=
          cmds_.at(leg).kp_cartesian * (cmds_.at(leg).p_des - datas_.at(leg).p);
      foot_force +=
          cmds_.at(leg).kd_cartesian * (cmds_.at(leg).v_des - datas_.at(leg).v);

      // Torque 足力转换成力矩
      leg_torque += datas_.at(leg).J.transpose() * foot_force;

      // set command: 命令设置 设置力矩
      cmd.tau_abad_ff.at(leg) = leg_torque(0);
      cmd.tau_hip_ff.at(leg) = leg_torque(1);
      cmd.tau_knee_ff.at(leg) = leg_torque(2);

      // joint space pd
      // joint space PD
      cmd.kd_abad.at(leg) = cmds_.at(leg).kd_joint(0, 0);
      cmd.kd_hip.at(leg) = cmds_.at(leg).kd_joint(1, 1);
      cmd.kd_knee.at(leg) = cmds_.at(leg).kd_joint(2, 2);

      cmd.kp_abad.at(leg) = cmds_.at(leg).kp_joint(0, 0);
      cmd.kp_hip.at(leg) = cmds_.at(leg).kp_joint(1, 1);
      cmd.kp_knee.at(leg) = cmds_.at(leg).kp_joint(2, 2);

      cmd.q_des_abad.at(leg) = cmds_.at(leg).q_des(0);
      cmd.q_des_hip.at(leg) = cmds_.at(leg).q_des(1);
      cmd.q_des_knee.at(leg) = cmds_.at(leg).q_des(2);

      cmd.qd_des_abad.at(leg) = cmds_.at(leg).qd_des(0);
      cmd.qd_des_hip.at(leg) = cmds_.at(leg).qd_des(1);
      cmd.qd_des_knee.at(leg) = cmds_.at(leg).qd_des(2);

      // estimate torque
      datas_.at(leg).tau_estimate =
          leg_torque +
          cmds_.at(leg).kp_joint * (cmds_.at(leg).q_des - datas_.at(leg).q) +
          cmds_.at(leg).kd_joint * (cmds_.at(leg).qd_des - datas_.at(leg).qd);

      cmd.flags.at(leg) = enabled_ ? 1 : 0;
    }
  }

  void LegCtrl::ZeroCmd()
  {
    for (auto &cmd : cmds_)
    {
      cmd.Zero();
    }
    enabled_ = false;
  }

  void LegCtrl::ComputeLegJacobianAndPosition(int leg)
  {
    double l1 = robot::QuadrupedProperties::abad_link_length;
    double l2 = robot::QuadrupedProperties::hip_link_length;
    double l3 = robot::QuadrupedProperties::knee_link_length;
    double l4 = robot::QuadrupedProperties::knee_link_y_offset;
    double side_sign = robot::leg::SideSign::GetSideSign(leg);

    double s1 = std::sin(datas_.at(leg).q(0));
    double s2 = std::sin(datas_.at(leg).q(1));
    double s3 = std::sin(datas_.at(leg).q(2));

    double c1 = std::cos(datas_.at(leg).q(0));
    double c2 = std::cos(datas_.at(leg).q(1));
    double c3 = std::cos(datas_.at(leg).q(2));

    double c23 = c2 * c3 - s2 * s3;
    double s23 = s2 * c3 + c2 * s3;

    datas_.at(leg).J.operator()(0, 0) = 0;
    datas_.at(leg).J.operator()(0, 1) = l3 * c23 + l2 * c2;
    datas_.at(leg).J.operator()(0, 2) = l3 * c23;
    datas_.at(leg).J.operator()(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - (l1 + l4) * side_sign * s1;
    datas_.at(leg).J.operator()(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
    datas_.at(leg).J.operator()(1, 2) = -l3 * s1 * s23;
    datas_.at(leg).J.operator()(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + (l1 + l4) * side_sign * c1;
    datas_.at(leg).J.operator()(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
    datas_.at(leg).J.operator()(2, 2) = l3 * c1 * s23;

    datas_.at(leg).p.operator()(0) = l3 * s23 + l2 * s2;
    datas_.at(leg).p.operator()(1) = (l1 + l4) * side_sign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
    datas_.at(leg).p.operator()(2) = (l1 + l4) * side_sign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;
  }
}

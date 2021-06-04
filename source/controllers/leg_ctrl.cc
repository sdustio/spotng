#include "sd/controllers/leg_ctrl.h"

namespace sd::ctrl
{
  void LegCtrl::UpdateData(const robot::SPIData &data)
  {
    for (int leg = 0; leg < 4; leg++)
    {
      // q: 关节角
      data_[leg].q(0) = data.q_abad[leg];
      data_[leg].q(1) = data.q_hip[leg];
      data_[leg].q(2) = data.q_knee[leg];

      // qd 关节角速度？
      data_[leg].qd(0) = data.qd_abad[leg];
      data_[leg].qd(1) = data.qd_hip[leg];
      data_[leg].qd(2) = data.qd_knee[leg];

      // J and p 雅可比和足端位置
      ComputeLegJacobianAndPosition(leg);

      // v 足端速度
      data_[leg].v = data_[leg].J * data_[leg].qd;
    }
  }

  void LegCtrl::UpdateSPICmd(robot::SPICmd &cmd)
  {
    for (int leg = 0; leg < 4; leg++)
    {
      // tauFF 获得从控制器来的力矩
      Vector3d leg_torque = cmd_[leg].tau_feed_forward;

      // forceFF 获得从控制器来的力矩
      Vector3d foot_force = cmd_[leg].force_feed_forward;

      // cartesian PD 直角坐标下pd
      foot_force +=
          cmd_[leg].kp_cartesian * (cmd_[leg].p_des - data_[leg].p);
      foot_force +=
          cmd_[leg].kd_cartesian * (cmd_[leg].v_des - data_[leg].v);

      // Torque 足力转换成力矩
      leg_torque += data_[leg].J.transpose() * foot_force;

      // set command: 命令设置 设置力矩
      cmd.tau_abad_ff[leg] = leg_torque(0);
      cmd.tau_hip_ff[leg] = leg_torque(1);
      cmd.tau_knee_ff[leg] = leg_torque(2);

      // joint space pd
      // joint space PD
      cmd.kd_abad[leg] = cmd_[leg].kd_joint(0, 0);
      cmd.kd_hip[leg] = cmd_[leg].kd_joint(1, 1);
      cmd.kd_knee[leg] = cmd_[leg].kd_joint(2, 2);

      cmd.kp_abad[leg] = cmd_[leg].kp_joint(0, 0);
      cmd.kp_hip[leg] = cmd_[leg].kp_joint(1, 1);
      cmd.kp_knee[leg] = cmd_[leg].kp_joint(2, 2);

      cmd.q_des_abad[leg] = cmd_[leg].q_des(0);
      cmd.q_des_hip[leg] = cmd_[leg].q_des(1);
      cmd.q_des_knee[leg] = cmd_[leg].q_des(2);

      cmd.qd_des_abad[leg] = cmd_[leg].qd_des(0);
      cmd.qd_des_hip[leg] = cmd_[leg].qd_des(1);
      cmd.qd_des_knee[leg] = cmd_[leg].qd_des(2);

      // estimate torque
      data_[leg].tau_estimate =
          leg_torque +
          cmd_[leg].kp_joint * (cmd_[leg].q_des - data_[leg].q) +
          cmd_[leg].kd_joint * (cmd_[leg].qd_des - data_[leg].qd);

      cmd.flags[leg] = enabled_ ? 1 : 0;
    }
  }

  void LegCtrl::ZeroCmd()
  {
    for (auto &cmd : cmd_)
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

    double s1 = std::sin(data_[leg].q(0));
    double s2 = std::sin(data_[leg].q(1));
    double s3 = std::sin(data_[leg].q(2));

    double c1 = std::cos(data_[leg].q(0));
    double c2 = std::cos(data_[leg].q(1));
    double c3 = std::cos(data_[leg].q(2));

    double c23 = c2 * c3 - s2 * s3;
    double s23 = s2 * c3 + c2 * s3;

    data_[leg].J.operator()(0, 0) = 0;
    data_[leg].J.operator()(0, 1) = l3 * c23 + l2 * c2;
    data_[leg].J.operator()(0, 2) = l3 * c23;
    data_[leg].J.operator()(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - (l1 + l4) * side_sign * s1;
    data_[leg].J.operator()(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
    data_[leg].J.operator()(1, 2) = -l3 * s1 * s23;
    data_[leg].J.operator()(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + (l1 + l4) * side_sign * c1;
    data_[leg].J.operator()(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
    data_[leg].J.operator()(2, 2) = l3 * c1 * s23;

    data_[leg].p.operator()(0) = l3 * s23 + l2 * s2;
    data_[leg].p.operator()(1) = (l1 + l4) * side_sign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
    data_[leg].p.operator()(2) = (l1 + l4) * side_sign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;
  }
}

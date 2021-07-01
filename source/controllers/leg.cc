#include "sd/controllers/leg.h"

namespace sd::ctrl
{
  void Leg::UpdateDatas(const robot::SPIData &data)
  {
    for (size_t leg = 0; leg < robot::ModelAttrs::num_leg; leg++)
    {
      // q: 关节角
      datas_[leg].q(0) = data.q_abad[leg];
      datas_[leg].q(1) = data.q_hip[leg];
      datas_[leg].q(2) = data.q_knee[leg];

      // qd 关节角速度？
      datas_[leg].qd(0) = data.qd_abad[leg];
      datas_[leg].qd(1) = data.qd_hip[leg];
      datas_[leg].qd(2) = data.qd_knee[leg];

      // J and p 雅可比和足端位置
      ComputeLegJacobianAndPosition(leg);

      // v 足端速度
      datas_[leg].v = datas_[leg].J * datas_[leg].qd;
    }
  }

  void Leg::UpdateSPICmd(robot::SPICmd &cmd)
  {
    for (size_t leg = 0; leg < robot::ModelAttrs::num_leg; leg++)
    {
      // tauFF 获得从控制器来的力矩
      Vector3d leg_torque = cmds_[leg].tau_feed_forward;

      // forceFF 获得从控制器来的力矩
      Vector3d foot_force = cmds_[leg].force_feed_forward;

      // cartesian PD 直角坐标下pd
      foot_force +=
          cmds_[leg].kp_cartesian * (cmds_[leg].p_des - datas_[leg].p);
      foot_force +=
          cmds_[leg].kd_cartesian * (cmds_[leg].v_des - datas_[leg].v);

      // Torque 足力转换成力矩
      leg_torque += datas_[leg].J.transpose() * foot_force;

      // set command: 命令设置 设置力矩
      cmd.tau_abad_ff[leg] = leg_torque(0);
      cmd.tau_hip_ff[leg] = leg_torque(1);
      cmd.tau_knee_ff[leg] = leg_torque(2);

      // joint space pd
      // joint space PD
      cmd.kd_abad[leg] = cmds_[leg].kd_joint(0, 0);
      cmd.kd_hip[leg] = cmds_[leg].kd_joint(1, 1);
      cmd.kd_knee[leg] = cmds_[leg].kd_joint(2, 2);

      cmd.kp_abad[leg] = cmds_[leg].kp_joint(0, 0);
      cmd.kp_hip[leg] = cmds_[leg].kp_joint(1, 1);
      cmd.kp_knee[leg] = cmds_[leg].kp_joint(2, 2);

      cmd.q_des_abad[leg] = cmds_[leg].q_des(0);
      cmd.q_des_hip[leg] = cmds_[leg].q_des(1);
      cmd.q_des_knee[leg] = cmds_[leg].q_des(2);

      cmd.qd_des_abad[leg] = cmds_[leg].qd_des(0);
      cmd.qd_des_hip[leg] = cmds_[leg].qd_des(1);
      cmd.qd_des_knee[leg] = cmds_[leg].qd_des(2);

      // estimate torque
      datas_[leg].tau_estimate =
          leg_torque +
          cmds_[leg].kp_joint * (cmds_[leg].q_des - datas_[leg].q) +
          cmds_[leg].kd_joint * (cmds_[leg].qd_des - datas_[leg].qd);

      cmd.flags[leg] = enabled_ ? 1 : 0;
    }
  }

  void Leg::ZeroCmd()
  {
    for (auto &cmd : cmds_)
    {
      cmd.Zero();
    }
    enabled_ = false;
  }

  void Leg::ComputeLegJacobianAndPosition(size_t leg)
  {
    double l1 = robot::ModelAttrs::abad_link_length;
    double l2 = robot::ModelAttrs::hip_link_length;
    double l3 = robot::ModelAttrs::knee_link_length;
    double l4 = robot::ModelAttrs::knee_link_y_offset;
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

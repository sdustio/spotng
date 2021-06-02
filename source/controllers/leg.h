#pragma once

#include "sd/robot/model.h"
#include "sd/robot/interface.h"

namespace sd::ctrl
{

  template <typename T>
  class LegCtrl
  {
  public:
    void SetLegEnabled(bool enabled) { legs_enabled = enabled; }

    /*!
    * Update the "leg data" from a SPIne board message
    * 从spine卡 更新腿部信息
    */
    void UpdateLegData(const robot::SPIData &data)
    {
      for (int leg = 0; leg < 4; leg++)
      {
        // q: 关节角
        leg_data[leg].q(0) = data.q_abad[leg];
        leg_data[leg].q(1) = data.q_hip[leg];
        leg_data[leg].q(2) = data.q_knee[leg];

        // qd 关节角速度？
        leg_data[leg].qd(0) = data.qd_abad[leg];
        leg_data[leg].qd(1) = data.qd_hip[leg];
        leg_data[leg].qd(2) = data.qd_knee[leg];

        // J and p 雅可比和足端位置
        ComputeLegJacobianAndPosition(leg);

        // v 足端速度
        leg_data[leg].v = leg_data[leg].J * leg_data[leg].qd;
      }
    }

    /*!
    * Update the "leg command" for the SPIne board message
    * 向控制器发送控制命令
    */
    void UpdateLegCmd(robot::SPICmd &cmd)
    {
      for (int leg = 0; leg < 4; leg++)
      {
        // tauFF 获得从控制器来的力矩
        Vec3<T> leg_torque = leg_cmd[leg].tau_feed_forward;

        // forceFF 获得从控制器来的力矩
        Vec3<T> foot_force = leg_cmd[leg].force_feed_forward;

        // cartesian PD 直角坐标下pd
        foot_force +=
            leg_cmd[leg].kp_cartesian * (leg_cmd[leg].p_des - leg_data[leg].p);
        foot_force +=
            leg_cmd[leg].kd_cartesian * (leg_cmd[leg].v_des - leg_data[leg].v);

        // Torque 足力转换成力矩
        leg_torque += leg_data[leg].J.transpose() * foot_force;

        // set command: 命令设置 设置力矩
        cmd.tau_abad_ff[leg] = leg_torque(0);
        cmd.tau_hip_ff[leg] = leg_torque(1);
        cmd.tau_knee_ff[leg] = leg_torque(2);

        // joint space pd
        // joint space PD
        cmd.kd_abad[leg] = leg_cmd[leg].kd_joint(0, 0);
        cmd.kd_hip[leg] = leg_cmd[leg].kd_joint(1, 1);
        cmd.kd_knee[leg] = leg_cmd[leg].kd_joint(2, 2);

        cmd.kp_abad[leg] = leg_cmd[leg].kp_joint(0, 0);
        cmd.kp_hip[leg] = leg_cmd[leg].kp_joint(1, 1);
        cmd.kp_knee[leg] = leg_cmd[leg].kp_joint(2, 2);

        cmd.q_des_abad[leg] = leg_cmd[leg].q_des(0);
        cmd.q_des_hip[leg] = leg_cmd[leg].q_des(1);
        cmd.q_des_knee[leg] = leg_cmd[leg].q_des(2);

        cmd.qd_des_abad[leg] = leg_cmd[leg].qd_des(0);
        cmd.qd_des_hip[leg] = leg_cmd[leg].qd_des(1);
        cmd.qd_des_knee[leg] = leg_cmd[leg].qd_des(2);

        // estimate torque
        leg_data[leg].tau_estimate =
            leg_torque +
            leg_cmd[leg].kp_joint * (leg_cmd[leg].q_des - leg_data[leg].q) +
            leg_cmd[leg].kd_joint * (leg_cmd[leg].qd_des - leg_data[leg].qd);

        cmd.flags[leg] = legs_enabled ? 1 : 0;
      }
    }

    /*!
    * Zero all leg commands.  This should be run *before* any control code, so if
    * the control code is confused and doesn't change the leg command, the legs
    * won't remember the last command.
    * 腿部控制命令清零，应运行在任何控制代码之前，否则控制代码混乱，控制命令不会改变，腿部不会记忆上次命令
    */
    void ZeroLegCmd()
    {
      for (auto &cmd : leg_cmd)
      {
        cmd.Zero();
      }
      legs_enabled = false;
    }

    /*!
    * Compute the position of the foot and its Jacobian.  This is done in the local
    * leg coordinate system. If J/p are NULL, the calculation will be skipped.
    */
    void ComputeLegJacobianAndPosition(int leg)
    {
      T l1 = robot::Properties::abad_link_length;
      T l2 = robot::Properties::hip_link_length;
      T l3 = robot::Properties::knee_link_length;
      T l4 = robot::Properties::knee_link_y_offset;
      T side_sign = robot::leg::SideSign<T>::GetSideSign(leg);

      T s1 = std::sin(leg_data[leg].q(0));
      T s2 = std::sin(leg_data[leg].q(1));
      T s3 = std::sin(leg_data[leg].q(2));

      T c1 = std::cos(leg_data[leg].q(0));
      T c2 = std::cos(leg_data[leg].q(1));
      T c3 = std::cos(leg_data[leg].q(2));

      T c23 = c2 * c3 - s2 * s3;
      T s23 = s2 * c3 + c2 * s3;

      leg_data[leg].J.operator()(0, 0) = 0;
      leg_data[leg].J.operator()(0, 1) = l3 * c23 + l2 * c2;
      leg_data[leg].J.operator()(0, 2) = l3 * c23;
      leg_data[leg].J.operator()(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - (l1 + l4) * side_sign * s1;
      leg_data[leg].J.operator()(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
      leg_data[leg].J.operator()(1, 2) = -l3 * s1 * s23;
      leg_data[leg].J.operator()(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + (l1 + l4) * side_sign * c1;
      leg_data[leg].J.operator()(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
      leg_data[leg].J.operator()(2, 2) = l3 * c1 * s23;

      leg_data[leg].p.operator()(0) = l3 * s23 + l2 * s2;
      leg_data[leg].p.operator()(1) = (l1 + l4) * side_sign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
      leg_data[leg].p.operator()(2) = (l1 + l4) * side_sign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;
    }

  private:
    robot::leg::Cmd<T> leg_cmd[4];
    robot::leg::Data<T> leg_data[4];
    bool legs_enabled = false;
  };

}

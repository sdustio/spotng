#pragma once

#include "sd/types.h"

namespace sd::robot
{

  struct Properties
  {
    constexpr static float body_length = 0.19 * 2;
    constexpr static float body_width = 0.049 * 2;
    constexpr static float body_height = 0.05 * 2;
    constexpr static float body_mass = 3.3;

    constexpr static float abad_gear_ratio = 6.0;
    constexpr static float hip_gear_ratio = 6.0;
    constexpr static float knee_gear_ratio = 9.33;

    constexpr static float abad_link_length = 0.062;
    constexpr static float hip_link_length = 0.209;
    constexpr static float knee_link_length = 0.195;

    constexpr static float knee_link_y_offset = 0.004;
    constexpr static float max_leg_length = 0.409;

    constexpr static float motor_kt = 0.05;
    constexpr static float motor_r = 0.173;
    constexpr static float motor_tau_max = 3.0;
    constexpr static float battery_v = 24;

    constexpr static float joint_damping = 0.01;
    constexpr static float joint_dry_friction = 0.2;
  };

  namespace leg
  {

    struct Idx
    {
      constexpr static int fr = 0; // Front Right
      constexpr static int fl = 1; // Front Left
      constexpr static int hr = 2; // Hind Right
      constexpr static int hl = 3; // Hind Left
    };

    /*!
    * Data sent from the control algorithm to the legs.
    * 数据从控制算法发送到腿部。
    */
    template <typename T>
    struct Cmd
    {
      Cmd() { Zero(); }
      Vec3<T> tau_feed_forward, force_feed_forward, q_des, qd_des, p_des, v_des;
      Mat3<T> kp_cartesian, kd_cartesian, kp_joint, kd_joint;

      void Zero()
      {
        tau_feed_forward = force_feed_forward = q_des = qd_des = p_des = v_des = Vec3<T>::Zero();
        kp_cartesian = kd_cartesian = kp_joint, kd_joint = Mat3<T>::Zero();
      }
    };

    /*!
    * Data returned from the legs to the control code.
    * 从腿返回到控制代码的数据
    */
    template <typename T>
    struct Data
    {
      Data() { Zero(); }
      Vec3<T> q, qd, p, v;  //关节角度 关节角速度 足端位置 足端速度
      Mat3<T> J;            //雅可比
      Vec3<T> tau_estimate; //估计力矩反馈
      void Zero()
      {
        q = qd = p = v = Vec3<T>::Zero();
        J = Mat3<T>::Zero();
        tau_estimate = Vec3<T>::Zero();
      }
    };
  }

} // namespace sd::robot::model

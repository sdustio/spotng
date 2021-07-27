#pragma once

#include <memory>

#include "sdrobot/types.h"

namespace sdrobot::leg
{
  namespace idx
  {
    constexpr int fr = 0; // Front Right
    constexpr int fl = 1; // Front Left
    constexpr int hr = 2; // Hind Right
    constexpr int hl = 3; // Hind Left
  }

  /*!
  * Get if the i-th leg is on the left (+) or right (-) of the robot. 判断第i条腿是在机器人的左边(+)还是右边(-)。
  * @param leg : the leg index
  * @return The side sign (-1 for right legs, +1 for left legs)
  */
  double SDROBOT_EXPORT GetSideSign(int leg);

  /*!
  * Flip signs of elements of a vector V depending on which leg it belongs to 一个向量V的元素的翻转符号取决于它属于哪条腿
  */
  void SDROBOT_EXPORT FlipWithSideSigns(Array3f &ret, const Array3f &v, int leg_id);

  /*!
    * Data sent from the control algorithm to the legs.
    * 数据从控制算法发送到腿部。
    */
  struct SDROBOT_EXPORT Cmd
  {
    Array3f tau_feed_forward = {};
    Array3f q_des = {};
    Array3f qd_des = {};
    Matrix3f kp_joint = {};
    Matrix3f kd_joint = {};

    //from mpc; aid coumpter above;
    Array3f force_feed_forward = {};
    Array3f p_des = {};
    Array3f v_des = {};

    //from mpc; aid coumpter above;
    Matrix3f kp_cartesian = {};
    Matrix3f kd_cartesian = {};

    void Zero();
  };

  /*!
    * Data returned from the legs to the control code.
    * 从腿返回到控制代码的数据
    */
  struct SDROBOT_EXPORT Data
  {
    Array3f q = {};            //关节角度
    Array3f qd = {};           //关节角速度
    Array3f p = {};            //足端位置
    Array3f v = {};            //足端速度
    Matrix3f J = {};           //雅可比
    Array3f tau_estimate = {}; //估计力矩反馈

    void Zero();
  };

  using Cmds = std::array<Cmd, 4>;
  using Datas = std::array<Data, 4>;
} // namespace sdrobot::leg

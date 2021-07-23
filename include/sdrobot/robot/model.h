#pragma once

#include <memory>
#include <array>

#include "sdrobot/types.h"
#include "sdrobot/dynamics/fb_model.h"

namespace sdrobot::robot
{

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
    struct Cmd
    {
      Cmd() { Zero(); }
      Vector3 tau_feed_forward, q_des, qd_des;
      Matrix3 kp_joint, kd_joint;
      Vector3 force_feed_forward, p_des, v_des; //from mpc; aid coumpter above;
      Matrix3 kp_cartesian, kd_cartesian; //from mpc; aid coumpter above;

      void Zero()
      {
        tau_feed_forward = force_feed_forward = q_des = qd_des = p_des = v_des = Vector3::Zero();
        kp_cartesian = kd_cartesian = kp_joint, kd_joint = Matrix3::Zero();
      }
    };

    /*!
    * Data returned from the legs to the control code.
    * 从腿返回到控制代码的数据
    */
    struct Data
    {
      Data() { Zero(); }
      Vector3 q, qd, p, v;  //关节角度 关节角速度 足端位置 足端速度
      Matrix3 J;            //雅可比
      Vector3 tau_estimate; //估计力矩反馈
      void Zero()
      {
        q = qd = p = v = Vector3::Zero();
        J = Matrix3::Zero();
        tau_estimate = Vector3::Zero();
      }
    };

    class SideSign
    {
    public:
      /*!
      * Get if the i-th leg is on the left (+) or right (-) of the robot. 判断第i条腿是在机器人的左边(+)还是右边(-)。
      * @param leg : the leg index
      * @return The side sign (-1 for right legs, +1 for left legs)
      */
      static double GetSideSign(int leg)
      {
        return side_signs_.at(leg);
      }

      /*!
      * Flip signs of elements of a vector V depending on which leg it belongs to 一个向量V的元素的翻转符号取决于它属于哪条腿
      */
      static Vector3 WithLegSigns(const Vector3 &v, int leg_id)
      {
        switch (leg_id)
        {
        case Idx::fr:
          return Vector3(v[0], -v[1], v[2]);
        case Idx::fl:
          return Vector3(v[0], v[1], v[2]);
        case Idx::hr:
          return Vector3(-v[0], -v[1], v[2]);
        case Idx::hl:
          return Vector3(-v[0], v[1], v[2]);
        default:
          throw std::runtime_error("Invalid leg id!");
        }
      }

    private:
      constexpr static std::array<double, 4> side_signs_{-1.0, 1.0, -1.0, 1.0};
    };

    using Cmds = std::array<robot::leg::Cmd, 4>;
    using Datas = std::array<robot::leg::Data, 4>;
  } // namespace sdrobot::robot::leg

  struct ModelAttrs
  {
    constexpr static double body_length = 0.19 * 2;
    constexpr static double body_width = 0.049 * 2;
    constexpr static double body_height = 0.05 * 2;
    constexpr static double body_mass = 3.3;

    constexpr static double abad_gear_ratio = 6.0;
    constexpr static double hip_gear_ratio = 6.0;
    constexpr static double knee_gear_ratio = 9.33;

    constexpr static double abad_link_length = 0.062;
    constexpr static double hip_link_length = 0.209;
    constexpr static double knee_link_length = 0.195;

    constexpr static double knee_link_y_offset = 0.004;
    constexpr static double max_leg_length = 0.409;

    constexpr static double motor_kt = 0.05;
    constexpr static double motor_r = 0.173;
    constexpr static double motor_tau_max = 3.0;
    constexpr static double battery_v = 24;

    constexpr static double joint_damping = 0.01;
    constexpr static double joint_dry_friction = 0.2;

    constexpr static int num_act_joint = 12;
    constexpr static int num_q = 19;
    constexpr static int dim_config = 18;
    constexpr static int num_leg = 4;
    constexpr static int num_leg_joint = 3;
  };

  struct DynamicsAttrs
  {
    constexpr static std::array<double, 3> kp_joint = {3, 3, 3};
    constexpr static std::array<double, 3> kd_joint = {1, 0.2, 0.2};

    constexpr static std::array<double, 3> kp_body = {100, 100, 100};
    constexpr static std::array<double, 3> kd_body = {10, 10, 20};

    constexpr static std::array<double, 3> kp_foot = {500, 500, 500};
    constexpr static std::array<double, 3> kd_foot = {60, 60, 60};

    constexpr static std::array<double, 3> kp_ori = {100, 100, 100};
    constexpr static std::array<double, 3> kd_ori = {10, 10, 10};
  };

  struct LinkId
  {
    constexpr static int fr = 9;  // Front Right Foot
    constexpr static int fl = 11; // Front Left Foot
    constexpr static int hr = 13; // Hind Right Foot
    constexpr static int hl = 15; // Hind Left Foot

    constexpr static int fr_abd = 2; // Front Right Abduction
    constexpr static int fl_abd = 0; // Front Left Abduction
    constexpr static int hr_abd = 3; // Hind Right Abduction
    constexpr static int hl_abd = 1; // Hind Left Abduction
  };

  class Quadruped
  {
  public:
    explicit Quadruped();

    /*!
    * Build a FloatingBaseModel of the quadruped 建立一个四足动物的浮动模型
    */
    const dynamics::FBModelPtr &BuildModel();

    /*!
    * Get location of the hip for the given leg in robot frame 在机器人框架中获取给定腿的臀部位置
    * @param leg : the leg index
    */
    Vector3 GetHipLocation(int leg) const
    {
      Vector3 pHip((leg == leg::Idx::fr || leg == leg::Idx::fl) ? abad_location_(0) : -abad_location_(0),
                   (leg == leg::Idx::fl || leg == leg::Idx::hl) ? abad_location_(1) : -abad_location_(1),
                   abad_location_(2));
      return pHip;
    }

  private:
    Vector3 abad_location_, abad_rotor_location_;
    Vector3 hip_location_, hip_rotor_location_;
    Vector3 knee_location_, knee_rotor_location_;
    dynamics::SpatialInertia abad_spatial_inertia_, hip_spatial_inertia_, knee_spatial_inertia_;
    dynamics::SpatialInertia abad_rotor_spatial_inertia_, hip_rotor_spatial_inertia_, knee_rotor_spatial_inertia_;
    dynamics::SpatialInertia body_spatial_inertia_;

    dynamics::FBModelPtr model_;
  };

  using QuadrupedPtr = std::shared_ptr<Quadruped>;
} // namespace sdrobot::robot

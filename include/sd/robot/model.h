#pragma once

#include <memory>
#include <array>

#include "sd/types.h"
#include "sd/dynamics/fb_model.h"

namespace sd::robot
{

  namespace leg
  {

    struct Idx
    {
      constexpr static size_t fr = 0; // Front Right
      constexpr static size_t fl = 1; // Front Left
      constexpr static size_t hr = 2; // Hind Right
      constexpr static size_t hl = 3; // Hind Left
    };

    /*!
    * Data sent from the control algorithm to the legs.
    * 数据从控制算法发送到腿部。
    */
    struct Cmd
    {
      Cmd() { Zero(); }
      Vector3d tau_feed_forward, force_feed_forward, q_des, qd_des, p_des, v_des;
      Matrix3d kp_cartesian, kd_cartesian, kp_joint, kd_joint;

      void Zero()
      {
        tau_feed_forward = force_feed_forward = q_des = qd_des = p_des = v_des = Vector3d::Zero();
        kp_cartesian = kd_cartesian = kp_joint, kd_joint = Matrix3d::Zero();
      }
    };

    /*!
    * Data returned from the legs to the control code.
    * 从腿返回到控制代码的数据
    */
    struct Data
    {
      Data() { Zero(); }
      Vector3d q, qd, p, v;  //关节角度 关节角速度 足端位置 足端速度
      Matrix3d J;            //雅可比
      Vector3d tau_estimate; //估计力矩反馈
      void Zero()
      {
        q = qd = p = v = Vector3d::Zero();
        J = Matrix3d::Zero();
        tau_estimate = Vector3d::Zero();
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
      static double GetSideSign(size_t leg)
      {
        return side_signs_.at(leg);
      }

      /*!
      * Flip signs of elements of a vector V depending on which leg it belongs to 一个向量V的元素的翻转符号取决于它属于哪条腿
      */
      static Vector3d WithLegSigns(const Vector3d &v, size_t leg_id)
      {
        switch (leg_id)
        {
        case Idx::fr:
          return Vector3d(v[0], -v[1], v[2]);
        case Idx::fl:
          return Vector3d(v[0], v[1], v[2]);
        case Idx::hr:
          return Vector3d(-v[0], -v[1], v[2]);
        case Idx::hl:
          return Vector3d(-v[0], v[1], v[2]);
        default:
          throw std::runtime_error("Invalid leg id!");
        }
      }

    private:
      constexpr static std::array<double, 4> side_signs_{-1.0, 1.0, -1.0, 1.0};
    };

    using Cmds = std::array<robot::leg::Cmd, 4>;
    using Datas = std::array<robot::leg::Data, 4>;
  } // namespace sd::robot::leg

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


  class Quadruped
  {
  public:
    explicit Quadruped();

    /*!
    * Build a FloatingBaseModel of the quadruped 建立一个四足动物的浮动模型
    */
    dynamics::FBModelPtr BuildModel();


    /*!
    * Get location of the hip for the given leg in robot frame 在机器人框架中获取给定腿的臀部位置
    * @param leg : the leg index
    */
    Vector3d GetHipLocation(size_t leg) const
    {
      Vector3d pHip((leg == leg::Idx::fr || leg == leg::Idx::fl) ? abad_location_(0) : -abad_location_(0),
                   (leg == leg::Idx::fl || leg == leg::Idx::hl) ? abad_location_(1) : -abad_location_(1),
                   abad_location_(2));
      return pHip;
    }

  private:
    Vector3d abad_location_, abad_rotor_location_;
    Vector3d hip_location_, hip_rotor_location_;
    Vector3d knee_location_, knee_rotor_location_;
    dynamics::SpatialInertia abad_spatial_inertia_, hip_spatial_inertia_, knee_spatial_inertia_;
    dynamics::SpatialInertia abad_rotor_spatial_inertia_, hip_rotor_spatial_inertia_, knee_rotor_spatial_inertia_;
    dynamics::SpatialInertia body_spatial_inertia_;
  };

  using QuadrupedPtr = std::shared_ptr<Quadruped>;
} // namespace sd::robot

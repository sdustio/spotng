#pragma once

#include "sd/robot/model.h"
#include "sd/robot/interface.h"

#include "dynamics/inertia.h"
#include "dynamics/fb_model.h"

namespace sd::robot
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
    void UpdateLegData(const SPIData &data);

    /*!
    * Update the "leg command" for the SPIne board message
    * 向控制器发送控制命令
    */
    void UpdateLegCmd(SPICmd &cmd);

    /*!
    * Zero all leg commands.  This should be run *before* any control code, so if
    * the control code is confused and doesn't change the leg command, the legs
    * won't remember the last command.
    * 腿部控制命令清零，应运行在任何控制代码之前，否则控制代码混乱，控制命令不会改变，腿部不会记忆上次命令
    */
    void ZeroLegCmd();

    /*!
    * Compute the position of the foot and its Jacobian.  This is done in the local
    * leg coordinate system. If J/p are NULL, the calculation will be skipped.
    */
    void ComputeLegJacobianAndPosition(int leg);

  private:
    leg::Cmd<T> leg_cmd[4];
    leg::Data<T> leg_data[4];
    bool legs_enabled = false;
  };

  template <typename T>
  class Quadruped
  {
  public:
    explicit Quadruped();

    /*!
    * Build a FloatingBaseModel of the quadruped 建立一个四足动物的浮动模型
    */
    bool BuildModel(dynamics::FBModel<T> &model);

    /*!
    * Get if the i-th leg is on the left (+) or right (-) of the robot. 判断第i条腿是在机器人的左边(+)还是右边(-)。
    * @param leg : the leg index
    * @return The side sign (-1 for right legs, +1 for left legs)
    */
    static const T GetSideSign(int leg)
    {
      assert(leg >= 0 && leg < 4);
      return side_signs_[leg];
    }

    /*!
    * Flip signs of elements of a vector V depending on which leg it belongs to 一个向量V的元素的翻转符号取决于它属于哪条腿
    */
    template <typename T2>
    static Vec3<T> WithLegSigns(const Eigen::MatrixBase<T2> &v, int leg_id)
    {
      static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                    "Must have 3x1 matrix");
      switch (leg_id)
      {
      case leg::Idx::fr:
        return Vec3<T>(v[0], -v[1], v[2]);
      case leg::Idx::fl:
        return Vec3<T>(v[0], v[1], v[2]);
      case leg::Idx::hr:
        return Vec3<T>(-v[0], -v[1], v[2]);
      case leg::Idx::hl:
        return Vec3<T>(-v[0], v[1], v[2]);
      default:
        throw std::runtime_error("Invalid leg id!");
      }
    }

    /*!
    * Get location of the hip for the given leg in robot frame 在机器人框架中获取给定腿的臀部位置
    * @param leg : the leg index
    */
    Vec3<T> GetHipLocation(int leg) const
    {
      assert(leg >= 0 && leg < 4);
      Vec3<T> pHip((leg == leg::Idx::fr || leg == leg::Idx::fl) ? abad_location_(0) : -abad_location_(0),
                   (leg == leg::Idx::fl || leg == leg::Idx::hl) ? abad_location_(1) : -abad_location_(1),
                   abad_location_(2));
      return pHip;
    }

    LegCtrl<T>& GetLegCtrl() {return leg_ctrl_;}

  private:
    Vec3<T> abad_location_, abad_rotor_location_;
    Vec3<T> hip_location_, hip_rotor_location_;
    Vec3<T> knee_location_, knee_rotor_location_;
    dynamics::SpatialInertia<T> abad_spatial_inertia_, hip_spatial_inertia_, knee_spatial_inertia_;
    dynamics::SpatialInertia<T> abad_rotor_spatial_inertia_, hip_rotor_spatial_inertia_, knee_rotor_spatial_inertia_;
    dynamics::SpatialInertia<T> body_spatial_inertia_;

    LegCtrl<T> leg_ctrl_;

    constexpr static T side_signs_[4] = {-1, 1, -1, 1};
  };

} // namespace sd::robot::model

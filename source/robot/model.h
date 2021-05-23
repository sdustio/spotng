#pragma once

#include "sd/robot/model.h"

#include "dynamics/inertia.h"
#include "dynamics/fb_model.h"

namespace sd::robot
{

  template <typename T>
  class Quadruped
  {
  public:
    explicit Quadruped();

    bool BuildModel(dynamics::FBModel<T> &model);

    /*!
    * Get if the i-th leg is on the left (+) or right (-) of the robot. 判断第i条腿是在机器人的左边(+)还是右边(-)。
    * @param leg : the leg index
    * @return The side sign (-1 for right legs, +1 for left legs)
    */
    T GetSideSign(int leg) const
    {
      assert(leg >= 0 && leg < 4);
      return side_signs_[leg];
    }

    /*!
    * Flip signs of elements of a vector V depending on which leg it belongs to 一个向量V的元素的翻转符号取决于它属于哪条腿
    */
    template <typename T2>
    Vec3<T> WithLegSigns(const Eigen::MatrixBase<T2> &v, int leg_id)
    {
      static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                    "Must have 3x1 matrix");
      switch (leg_id)
      {
      case LegIdx::fr:
        return Vec3<T>(v[0], -v[1], v[2]);
      case LegIdx::fl:
        return Vec3<T>(v[0], v[1], v[2]);
      case LegIdx::hr:
        return Vec3<T>(-v[0], -v[1], v[2]);
      case LegIdx::hl:
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
      Vec3<T> pHip((leg == LegIdx::fr || leg == LegIdx::fl) ? abad_location_(0) : -abad_location_(0),
                   (leg == LegIdx::fl || leg == LegIdx::hl) ? abad_location_(1) : -abad_location_(1),
                   abad_location_(2));
      return pHip;
    }

  private:
    Vec3<T> abad_location_, abad_rotor_location_;
    Vec3<T> hip_location_, hip_rotor_location_;
    Vec3<T> knee_location_, knee_rotor_location_;
    dynamics::SpatialInertia<T> abad_spatial_inertia_, hip_spatial_inertia_, knee_spatial_inertia_;
    dynamics::SpatialInertia<T> abad_rotor_spatial_inertia_, hip_rotor_spatial_inertia_, knee_rotor_spatial_inertia_;
    dynamics::SpatialInertia<T> body_spatial_inertia_;

    const T side_signs_[4] = {-1, 1, -1, 1};
  };
} // namespace sd::robot::model

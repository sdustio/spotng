#pragma once

#include "sd/Robot/Model.hpp"

#include "Dynamics/Inertia.hpp"

namespace sd::robot::model
{

  using sd::dynamics::SpatialInertia;

  template <typename T>
  class Quadruped
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit Quadruped();
    /*!
   * Get location of the hip for the given leg in robot frame 在机器人框架中获取给定腿的臀部位置
   * @param leg : the leg index
   */
    Vec3<T> GetHipLocation(int leg)
    {
      assert(leg >= 0 && leg < 4);
      Vec3<T> pHip((leg == Leg::Idx::FR || leg == Leg::Idx::FL) ? mAbadLocation(0) : -mAbadLocation(0),
                   (leg == Leg::Idx::FL || leg == Leg::Idx::HL) ? mAbadLocation(1) : -mAbadLocation(1),
                   mAbadLocation(2));
      return pHip;
    }

  private:
    Vec3<T> mAbadLocation, mAbadRotorLocation;
    Vec3<T> mHipLocation, mHipRotorLocation;
    Vec3<T> mKneeLocation, mKneeRotorLocation;
    SpatialInertia<T> mAbadSpatialInertia, mHipSpatialInertia, mKneeSpatialInertia;
    SpatialInertia<T> mAbadRotorSpatialInertia, mHipRotorSpatialInertia, mKneeRotorSpatialInertia;
    SpatialInertia<T> mBodySpatialInertia;
  };
} // namespace sd::robot::model

#pragma once

#include "sd/robot/model.h"

#include "dynamics/inertia.h"
#include "dynamics/fb_model.h"
#include "controllers/leg.h"

namespace sd::robot
{

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

    ctrl::LegCtrl<T>& GetLegCtrl() {return leg_ctrl_;}

  private:
    Vec3<T> abad_location_, abad_rotor_location_;
    Vec3<T> hip_location_, hip_rotor_location_;
    Vec3<T> knee_location_, knee_rotor_location_;
    dynamics::SpatialInertia<T> abad_spatial_inertia_, hip_spatial_inertia_, knee_spatial_inertia_;
    dynamics::SpatialInertia<T> abad_rotor_spatial_inertia_, hip_rotor_spatial_inertia_, knee_rotor_spatial_inertia_;
    dynamics::SpatialInertia<T> body_spatial_inertia_;

    ctrl::LegCtrl<T> leg_ctrl_;
  };

} // namespace sd::robot::model

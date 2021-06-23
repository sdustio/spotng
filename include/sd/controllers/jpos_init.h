#pragma once

#include "sd/robot/model.h"
#include "sd/controllers/leg.h"
#include "sd/dynamics/bspline.h"

namespace sd::ctrl
{

  constexpr double kEndTime = 3.;

  class JPosInit
  {
  public:
    JPosInit();
    bool IsInitialized(LegPtr &ctrl);

  private:
    void UpdateInitial(const LegPtr &ctrl);

    double end_time_;
    double curr_time_ = 0.0;
    bool first_visit_ = true;

    std::array<double, dynamics::bspline::kDim> ini_jpos_;
    std::array<double, dynamics::bspline::kDim> target_jpos_;
    std::array<double, dynamics::bspline::kDim> mid_jpos_;

    dynamics::BSpline jpos_trj_;
  };

  using JPosInitPtr = std::shared_ptr<JPosInit>;

}

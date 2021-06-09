#pragma once

#include "sd/robot/model.h"
#include "sd/controllers/leg_ctrl.h"
#include "sd/dynamics/bspline.h"

namespace sd::ctrl
{
  class JPosInit
  {
  public:
    JPosInit(double end_time);
    bool IsInitialized(LegCtrlPtr &ctrl);

  private:
    void UpdateInitial(const LegCtrlPtr &ctrl);

    double end_time_;
    double curr_time_ = 0.0;
    bool first_visit_ = true;

    std::array<double, dynamics::bspline::kDim> ini_jpos_;
    std::array<double, dynamics::bspline::kDim> target_jpos_;
    std::array<double, dynamics::bspline::kDim> mid_jpos_;

    dynamics::BSpline jpos_trj_;
  };

  using JPosInitPtr = std::unique_ptr<JPosInit>;
  using JPosInitSharedPtr = std::shared_ptr<JPosInit>;

}

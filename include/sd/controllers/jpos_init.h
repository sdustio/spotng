#pragma once

#include <vector>

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

    std::vector<double> ini_jpos_;
    std::vector<double> target_jpos_;
    std::vector<double> mid_jpos_;

    dynamics::BSpline jpos_trj_;
  };

}

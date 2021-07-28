#pragma once

#include "sdrobot/leg.h"
#include "params.h"

namespace sdrobot::leg
{
  class JPosInitImpl : public JPosInit
  {
  public:
    JPosInitImpl(double dt, double end_time);
    bool IsInitialized(LegCtrl::SharedPtr &legctrl) override;

  private:
    void UpdateInitial(const LegCtrl::SharedPtr &legctrl);

    double dt_;
    double end_time_;
    double curr_time_ = 0.0;
    bool first_visit_ = true;

    std::array<double, params::model::num_act_joint> ini_jpos_;
    std::array<double, params::model::num_act_joint> target_jpos_;
    std::array<double, params::model::num_act_joint> mid_jpos_;

  };
}

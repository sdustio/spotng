#pragma once

#include "sdrobot/leg.h"
#include "sdrobot/params.h"

namespace sdrobot::leg
{
  class JPosInitImpl : public JPosInit
  {
  public:
    JPosInitImpl(fptype dt, fptype end_time);
    bool IsInitialized(LegCtrl::SharedPtr &legctrl) override;

  private:
    void UpdateInitial(const LegCtrl::SharedPtr &legctrl);

    fptype dt_;
    fptype end_time_;
    fptype curr_time_ = 0.0;
    bool first_visit_ = true;

    std::array<fptype, params::model::num_act_joint> ini_jpos_;
    std::array<fptype, params::model::num_act_joint> target_jpos_;
    std::array<fptype, params::model::num_act_joint> mid_jpos_;

  };
}

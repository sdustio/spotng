#pragma once

#include "sdrobot/leg.h"
#include "sdrobot/params.h"

namespace sdrobot::leg
{
  class JPosInitImpl : public JPosInit
  {
  public:
    JPosInitImpl(fpt_t dt, fpt_t end_time);
    bool IsInitialized(LegCtrl::SharedPtr const &legctrl) override;

  private:
    bool UpdateInitial(LegCtrl::SharedPtr const &legctrl);

    fpt_t dt_;
    fpt_t end_time_;
    fpt_t curr_time_ = 0.0;
    bool first_visit_ = true;

    std::array<fpt_t, params::model::kNumActJoint> ini_jpos_;
  };
}

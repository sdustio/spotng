#pragma once

#include "sdquadx/consts.h"
#include "sdquadx/leg.h"

namespace sdquadx {

namespace leg {

class JPosInitImpl : public JPosInit {
 public:
  JPosInitImpl(fpt_t dt, fpt_t end_time, SdVector3f const &kp, SdVector3f const &kd,
               std::array<SdVector3f, 4> const &jpos);
  bool IsInitialized(LegCtrl::SharedPtr const &legctrl) override;

 private:
  bool UpdateInitial(LegCtrl::SharedPtr const &legctrl);

  fpt_t dt_;
  fpt_t end_time_;
  fpt_t curr_time_ = 0.0;
  bool first_visit_ = true;

  SdVector3f kp_;
  SdVector3f kd_;
  std::array<fpt_t, consts::model::kNumActJoint> target_jpos_ = {};
  std::array<fpt_t, consts::model::kNumActJoint> ini_jpos_ = {};
};
}  // namespace leg

}  // namespace sdquadx

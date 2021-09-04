#pragma once

#include "sdquadx/consts.h"
#include "sdquadx/options.h"
#include "sdquadx/leg.h"

namespace sdquadx {

namespace leg {

class JPosInitImpl : public JPosInit {
 public:
  JPosInitImpl(Options::ConstSharedPtr const &opts);
  bool IsInitialized(LegCtrl::SharedPtr const &legctrl) override;

 private:
  bool UpdateInitial(LegCtrl::SharedPtr const &legctrl);

  Options::ConstSharedPtr const opts_;

  fpt_t curr_time_ = 0.0;
  bool first_visit_ = true;

  std::array<fpt_t, consts::model::kNumActJoint> target_jpos_ = {};
  std::array<fpt_t, consts::model::kNumActJoint> ini_jpos_ = {};
};
}  // namespace leg

}  // namespace sdquadx

#pragma once

#include "sdquadx/consts.h"
#include "sdquadx/estimate.h"
#include "sdquadx/leg.h"
#include "sdquadx/model.h"
#include "sdquadx/types.h"

namespace sdquadx::wbc {
using SdVector12f = std::array<fpt_t, consts::model::kNumActJoint>;
using SdVector18f = std::array<fpt_t, consts::model::kDimConfig>;

struct InData {
  SdVector3f body_pos_des;
  SdVector3f body_lvel_des;
  SdVector3f body_acc_des;
  SdVector3f body_rpy_des;
  SdVector3f body_avel_des;

  std::array<SdVector3f, consts::model::kNumLeg> foot_pos_des;
  std::array<SdVector3f, consts::model::kNumLeg> foot_lvel_des;
  std::array<SdVector3f, consts::model::kNumLeg> foot_acc_des;

  std::array<SdVector3f, consts::model::kNumLeg> Fr_des;

  SdVector4f contact_state;
};

class Wbc {
 public:
  using Ptr = std::unique_ptr<Wbc>;
  using SharedPtr = std::shared_ptr<Wbc>;
  using ConstSharedPtr = std::shared_ptr<Wbc const>;

  virtual ~Wbc() = default;
  virtual bool RunOnce(InData const &, estimate::State const &, leg::LegCtrl::SharedPtr const &) = 0;
};

}  // namespace sdquadx::wbc

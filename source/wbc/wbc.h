#pragma once

#include "sdrobot/types.h"
#include "sdrobot/consts.h"

namespace sdrobot::wbc
{
  using SdVector12f = std::array<fpt_t, consts::model::kNumActJoint>;
  using SdVector18f = std::array<fpt_t, consts::model::kDimConfig>;

  struct InData
  {
    SdVector3f body_pos_des;
    SdVector3f body_lvel_des;
    SdVector3f body_acc_des;
    SdVector3f body_rpy_des;
    SdVector3f body_avel_des;

    std::array<SdVector3f, 4> foot_pos_des;
    std::array<SdVector3f, 4> foot_lvel_des;
    std::array<SdVector3f, 4> foot_acc_des;

    std::array<SdVector3f, 4> Fr_des;

    SdVector4f contact_state;
  };
} // namespace sdrobot::wbc

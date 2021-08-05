#pragma once

#include "sdrobot/types.h"
#include "sdrobot/params.h"

namespace sdrobot::wbc
{
  using SdVector12f = std::array<fpt_t, params::model::num_act_joint>;
  using SdVector18f = std::array<fpt_t, params::model::dim_config>;

  struct InData
  {
    SdVector3f pos_body_des;
    SdVector3f vel_body_des;
    SdVector3f acc_body_des;
    SdVector3f pos_rpy_body_des;
    SdVector3f vel_rpy_body_des;

    std::array<SdVector3f, 4> pos_foot_des;
    std::array<SdVector3f, 4> vel_foot_des;
    std::array<SdVector3f, 4> acc_foot_des;

    std::array<SdVector3f, 4> Fr_des;

    SdVector4f contact_state;
  };
} // namespace sdrobot::wbc

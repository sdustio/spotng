#pragma once

#include "sdrobot/types.h"
#include "sdrobot/params.h"
#include "eigen.h"

namespace sdrobot::wbc
{
  using SdVector12f = std::array<fpt_t, params::model::num_act_joint>;
  using SdVector18f = std::array<fpt_t, params::model::dim_config>;
  using Vector12 = Eigen::Matrix<fpt_t, params::model::num_act_joint, 1>;
  using Vector18 = Eigen::Matrix<fpt_t, params::model::dim_config, 1>;

  struct InData
  {
    // TODO rename
    SdVector3f p_body_des;
    SdVector3f v_body_des;
    SdVector3f a_body_des;
    SdVector3f p_body_rpy_des;
    SdVector3f vbody_ori_des;

    std::array<SdVector3f, 4> p_foot_des;
    std::array<SdVector3f, 4> v_foot_des;
    std::array<SdVector3f, 4> a_foot_des;

    std::array<SdVector3f, 4> Fr_des;

    SdVector4f contact_state;
  };
} // namespace sdrobot::wbc

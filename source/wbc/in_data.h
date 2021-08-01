#pragma once

#include "sdrobot/types.h"

namespace sdrobot::wbc
{
  struct InData
  {
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

#pragma once

#include <memory>

#include "sdrobot/types.h"

namespace sdrobot::interface
{
  struct SDROBOT_EXPORT ActuatorCmd
  {
    SdVector4f q_des_abad = {};
    SdVector4f q_des_hip = {};
    SdVector4f q_des_knee = {};

    SdVector4f qd_des_abad = {};
    SdVector4f qd_des_hip = {};
    SdVector4f qd_des_knee = {};

    SdVector4f kp_abad = {};
    SdVector4f kp_hip = {};
    SdVector4f kp_knee = {};

    SdVector4f kd_abad = {};
    SdVector4f kd_hip = {};
    SdVector4f kd_knee = {};

    SdVector4f tau_abad_ff = {};
    SdVector4f tau_hip_ff = {};
    SdVector4f tau_knee_ff = {};

    SdVector4c flags = {};
  };

  struct SDROBOT_EXPORT ActuatorData
  {
    SdVector4f q_abad = {};
    SdVector4f q_hip = {};
    SdVector4f q_knee = {};
    SdVector4f qd_abad = {};
    SdVector4f qd_hip = {};
    SdVector4f qd_knee = {};
    SdVector4c flags = {};
    std::uint8_t driver_status;
  };

  class SDROBOT_EXPORT ActuatorInterface
  {
  public:
    using SharedPtr = std::shared_ptr<ActuatorInterface>;
    using Ptr = std::unique_ptr<ActuatorInterface>;

    virtual ~ActuatorInterface() = default;
    virtual ActuatorData const &GetActuatorData() const = 0;
    virtual ActuatorCmd &GetActuatorCmdForUpdate() = 0;
    virtual bool UpdateActuatorCmd(ActuatorCmd const &cmd) = 0;
    virtual bool Init() = 0;    // return true if ok
    virtual bool RunOnce() = 0; // return true if ok
  };
}

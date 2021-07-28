#pragma once

#include <memory>

#include "sdrobot/types.h"

namespace sdrobot::interface
{
  struct SDROBOT_EXPORT ActuatorCmd
  {
    SdArray4f q_des_abad = {};
    SdArray4f q_des_hip = {};
    SdArray4f q_des_knee = {};

    SdArray4f qd_des_abad = {};
    SdArray4f qd_des_hip = {};
    SdArray4f qd_des_knee = {};

    SdArray4f kp_abad = {};
    SdArray4f kp_hip = {};
    SdArray4f kp_knee = {};

    SdArray4f kd_abad = {};
    SdArray4f kd_hip = {};
    SdArray4f kd_knee = {};

    SdArray4f tau_abad_ff = {};
    SdArray4f tau_hip_ff = {};
    SdArray4f tau_knee_ff = {};

    SdArray4c flags = {};
  };

  struct SDROBOT_EXPORT ActuatorData
  {
    SdArray4f q_abad = {};
    SdArray4f q_hip = {};
    SdArray4f q_knee = {};
    SdArray4f qd_abad = {};
    SdArray4f qd_hip = {};
    SdArray4f qd_knee = {};
    SdArray4c flags = {};
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

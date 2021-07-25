#pragma once

#include <memory>

#include "sdrobot/export.h"
#include "sdrobot/types.h"

namespace sdrobot::interface
{
  struct SDROBOT_EXPORT ActuatorCmd
  {
    Array4f q_des_abad;
    Array4f q_des_hip;
    Array4f q_des_knee;

    Array4f qd_des_abad;
    Array4f qd_des_hip;
    Array4f qd_des_knee;

    Array4f kp_abad;
    Array4f kp_hip;
    Array4f kp_knee;

    Array4f kd_abad;
    Array4f kd_hip;
    Array4f kd_knee;

    Array4f tau_abad_ff;
    Array4f tau_hip_ff;
    Array4f tau_knee_ff;

    Array4c flags;
  };

  struct SDROBOT_EXPORT ActuatorData
  {
    Array4f q_abad;
    Array4f q_hip;
    Array4f q_knee;
    Array4f qd_abad;
    Array4f qd_hip;
    Array4f qd_knee;
    Array4c flags;
    uint8_t spi_driver_status;
  };

  class SDROBOT_EXPORT ActuatorInterface
  {
  public:
    virtual ~ActuatorInterface() = default;
    virtual bool GetActuatorData(ActuatorData &) const = 0;
    virtual bool UpdateActuatorCmd(ActuatorCmd const &) = 0;
    virtual bool Init() = 0;   // return true if ok
    virtual bool RunSPI() = 0; // return true if ok
  }
  using ActuatorInterfacePtr = std::shared_ptr<ActuatorInterface>;
}

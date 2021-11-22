#pragma once

#include <cstdint>
#include <memory>

#include "sdquadx/consts.h"
#include "sdquadx/types.h"

namespace sdquadx::interface {
struct SDQUADX_EXPORT ActuatorCmd {
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

  std::array<std::uint8_t, consts::model::kNumLeg> flags = {};
};

struct SDQUADX_EXPORT ActuatorData {
  SdVector4f q_abad = {};
  SdVector4f q_hip = {};
  SdVector4f q_knee = {};

  SdVector4f qd_abad = {};
  SdVector4f qd_hip = {};
  SdVector4f qd_knee = {};

  std::array<std::uint8_t, consts::model::kNumLeg> flags = {};
  std::uint8_t driver_status = 0;
};

class SDQUADX_EXPORT ActuatorInterface {
 public:
  using Ptr = std::unique_ptr<ActuatorInterface>;
  using SharedPtr = std::shared_ptr<ActuatorInterface>;
  using ConstSharedPtr = std::shared_ptr<ActuatorInterface const>;

  virtual ~ActuatorInterface() = default;
  virtual ActuatorData const &GetActuatorData() const = 0;
  virtual ActuatorCmd &GetActuatorCmdForUpdate() = 0;
  virtual bool UpdateActuatorCmd(ActuatorCmd const &cmd) = 0;
  virtual bool RunOnce() = 0;  // return true if ok
};
}  // namespace sdquadx::interface

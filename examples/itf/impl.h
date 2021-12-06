#include "sdquadx/interface.h"

using sdquadx::SdVector4f;

namespace sdquadx::interface {
struct JointsCmd {
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

struct JointsData {
  SdVector4f q_abad = {};
  SdVector4f q_hip = {};
  SdVector4f q_knee = {};

  SdVector4f qd_abad = {};
  SdVector4f qd_hip = {};
  SdVector4f qd_knee = {};

  std::array<std::uint8_t, consts::model::kNumLeg> flags = {};
  std::uint8_t driver_status = 0;
};

class LegImpl : public Leg {
 public:
  bool ReadTo(sensor::LegDatas &data) const override;
  bool WriteFrom(LegCmds const &cmds) override;
  bool RunOnce() override;  // return true if ok
  bool PrintCmd() const;

 private:
  bool PrintArray4f(SdVector4f const &) const;

  JointsData data_;
  JointsCmd cmd_;
};

class ImuImpl : public Imu {
 public:
  bool ReadTo(sensor::ImuData &data) const override;
  bool RunOnce() override;

 private:
  sensor::ImuData imu_;
};

}  // namespace sdquadx::interface

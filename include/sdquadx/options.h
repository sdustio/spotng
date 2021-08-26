#pragma once

#include "sdquadx/types.h"

namespace sdquadx {
enum class DriveMode : uint8_t {
  kAuto,
  kManual,
};

struct SDQUADX_EXPORT Options {
  DriveMode drive_mode = DriveMode::kAuto;

  fpt_t ctrl_dt_sec = 1.0 / (0.5 * 1'000);   // 0.5kHz
  fpt_t act_itf_sec = 1.0 / (0.04 * 1'000);  // 0.04kHz

  fpt_t jpos_init_sec = 3.;
  fpt_t gravity = 9.81;  // gravity scalar

  SdVector3f kp_joint = {3, 3, 3};
  SdVector3f kd_joint = {1, 0.2, 0.2};

  SdVector3f kp_body = {100, 100, 100};
  SdVector3f kd_body = {10, 10, 20};

  SdVector3f kp_foot = {500, 500, 500};
  SdVector3f kd_foot = {60, 60, 60};

  SdVector3f kp_ori = {100, 100, 100};
  SdVector3f kd_ori = {10, 10, 10};
};

}  // namespace sdquadx

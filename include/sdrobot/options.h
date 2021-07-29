#pragma once

#include "sdrobot/types.h"

namespace sdrobot
{
  enum class DriveMode : uint8_t
  {
    kAutoAll,   // auto gait and auto state
    kManualAll, // manual gait and manual state
    kAutoGait,  // auto gait and manual state
    kAutoState  // manual gait and auto state
  };

  struct SDROBOT_EXPORT Options
  {
    DriveMode drive_mode = DriveMode::kAutoAll;

    double ctrl_dt_sec = 1.0 / (0.5 * 1'000);  // 0.5kHz
    double act_itf_sec = 1.0 / (0.04 * 1'000); // 0.04kHz

    double jpos_init_sec = 3.;
    double gravity = 9.81; // gravity scalar
  };

} // namespace sdrobot

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

    fptype ctrl_dt_sec = 1.0 / (0.5 * 1'000);  // 0.5kHz
    fptype act_itf_sec = 1.0 / (0.04 * 1'000); // 0.04kHz

    fptype jpos_init_sec = 3.;
    fptype gravity = 9.81; // gravity scalar
  };

} // namespace sdrobot

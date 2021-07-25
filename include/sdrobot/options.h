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

    double ctrl_dt_sec = 0.002;
  };
} // namespace sdrobot

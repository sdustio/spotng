#pragma once

#include "sdrobot/robot/interface.h"

namespace sdrobot::hardware
{
  class RobotInterface: public robot::Interface
  {
    public:
      bool Init() override;
      bool RunSPI() override;
  };
} // namespace sdrobot::hardware

#pragma once

#include "sd/robot/interface.h"

namespace sd::hardware
{
  class RobotInterface: public robot::Interface
  {
    public:
      bool Init() override;
      bool RunSPI() override;
  };
} // namespace sd::hardware

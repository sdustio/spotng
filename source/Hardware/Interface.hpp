#pragma once

#include "sd/Robot/Interface.hpp"

namespace sd::hardware
{
  class RobotInterface: public robot::Interface
  {
    public:
      bool Init() override;
      bool RunSPI() override;
      bool RunIMU() override;
  };
} // namespace sd::hardware

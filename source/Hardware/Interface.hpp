#pragma once

#include "sd/Robot/Interface.hpp"

namespace sd::hardware
{
  class RobotInterface: public sd::robot::interface::Interface
  {
    public:
      bool Init() override;
      bool RunSPI() override;
      bool RunIMU() override;
  };
} // namespace sd::robot::interface::hardware

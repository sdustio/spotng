#include "sd/Robot/Interface.h"

namespace sd::robot::interface::hardware
{
  class Interface: public sd::robot::interface::Interface
  {
    public:
      bool Init() override;
      bool RunSPI() override;
      bool RunIMU() override;
  };
} // namespace sd::robot::interface::hardware

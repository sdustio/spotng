#include "sd/Robot/Interface.h"

namespace sd::robot::interface::hardware
{
  class Interface: public sd::robot::interface::Interface
  {
    public:
      bool Init();
      bool RunSPI();
      bool RunIMU();
  };
} // namespace sd::robot::interface::hardware

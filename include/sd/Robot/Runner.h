#pragma once

#include "sd/Robot/Interface.h"

namespace sd
{
  namespace robot
  {


    class Runner
    {
      public:
        Runner(Interface*);
        int init();
        void run();

      private:
        void handleDriverCmd();
        Interface* mInterface = nullptr;

        int iter = 0;
        interface::SPICmd mSPICmd;
        interface::SPIData mSPIData;
        interface::IMUData mIMUData;
    }

  } // namespace robot

} // namespace sd

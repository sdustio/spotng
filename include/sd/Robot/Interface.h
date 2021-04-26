#pragma once

#include <cstdint>
#include "sd/Types.h"

namespace sd
{
  namespace robot
  {
    namespace interface
    {

      struct SPICmd
      {
        float q_des_abad[4];
        float q_des_hip[4];
        float q_des_knee[4];

        float qd_des_abad[4];
        float qd_des_hip[4];
        float qd_des_knee[4];

        float kp_abad[4];
        float kp_hip[4];
        float kp_knee[4];

        float kd_abad[4];
        float kd_hip[4];
        float kd_knee[4];

        float tau_abad_ff[4];
        float tau_hip_ff[4];
        float tau_knee_ff[4];

        uint8_t flags[4];
      };

      /*!
      * Data from spine board
      */
      struct SPIData
      {
        float q_abad[4];
        float q_hip[4];
        float q_knee[4];
        float qd_abad[4];
        float qd_hip[4];
        float qd_knee[4];
        uint8_t flags[4];
        uint8_t spi_driver_status;
      };

      /*!
      * IMU
      */
      struct IMUData
      {
        Vec3<float> acc;
        Vec3<float> gyro;
        Quat<float> quat;
        // todo is there status for the vectornav?
      };
    } // namespace interface

    class Interface
    {
      public:
        interface::SPICmd* getSPICommand(){return &mSPICmd;}
        interface::SPIData* getSPIData(){return &mSPIData;}
        interface::IMUData* getIMUData(){return &mIMUData;}
        virtual bool init(); // return true if ok
        virtual bool runSPI(); // return true if ok
        virtual bool runIMU(); // return true if ok
      private:
        interface::SPICmd mSPICmd;
        interface::SPIData mSPIData;
        interface::IMUData mIMUData;
    }
  } // namespace robot

} // namespace sd

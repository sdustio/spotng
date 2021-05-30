#pragma once

#include <cstdint>

#include "sd/types.h"

namespace sd::robot
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
    Vec4<float> quat;
    // todo is there status for the vectornav?
  };

  class Interface
  {
  public:
    SPICmd& GetSPICmd() { return spi_cmd_; }
    const SPIData& GetSPIData() const { return spi_data_; }
    const IMUData& GetIMUData() const { return imu_data_; }
    virtual bool Init() = 0;   // return true if ok
    virtual bool RunSPI() = 0; // return true if ok
    virtual bool RunIMU() = 0; // return true if ok
  private:
    SPICmd spi_cmd_;
    SPIData spi_data_;
    IMUData imu_data_;
  };

} // namespace sd::robot::interface

#pragma once

#include <cstdint>
#include <memory>

#include "sd/types.h"

namespace sd::robot
{
  enum class Mode : uint8_t
  {
    Off,
    Stand,
    RecoveryStand,
    Locomotion
  };

  struct SPICmd
  {
    double q_des_abad[4];
    double q_des_hip[4];
    double q_des_knee[4];

    double qd_des_abad[4];
    double qd_des_hip[4];
    double qd_des_knee[4];

    double kp_abad[4];
    double kp_hip[4];
    double kp_knee[4];

    double kd_abad[4];
    double kd_hip[4];
    double kd_knee[4];

    double tau_abad_ff[4];
    double tau_hip_ff[4];
    double tau_knee_ff[4];

    uint8_t flags[4];
  };

  /*!
      * Data from spine board
      */
  struct SPIData
  {
    double q_abad[4];
    double q_hip[4];
    double q_knee[4];
    double qd_abad[4];
    double qd_hip[4];
    double qd_knee[4];
    uint8_t flags[4];
    uint8_t spi_driver_status;
  };

  /*!
      * IMU
      */
  struct IMUData
  {
    Vector3d acc;
    Vector3d gyro;
    Vector4d quat;
    // todo is there status for the vectornav?
  };

  class Interface
  {
  public:
    SPICmd& GetSPICmdForUpdate() { return spi_cmd_; }
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

  using InterfacePtr = std::unique_ptr<Interface>;
  using InterfaceSharedPtr = std::shared_ptr<Interface>;

} // namespace sd::robot::interface

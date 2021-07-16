#pragma once

#include <cstdint>
#include <memory>
#include <array>

#include "sdrobot/types.h"

namespace sdrobot::robot
{
  enum class Mode : uint8_t
  {
    Init,
    RecoveryStand,
    Locomotion,
    BalanceStand
  };

  struct SPICmd
  {
    std::array<double, 4> q_des_abad;
    std::array<double, 4> q_des_hip;
    std::array<double, 4> q_des_knee;

    std::array<double, 4> qd_des_abad;
    std::array<double, 4> qd_des_hip;
    std::array<double, 4> qd_des_knee;

    std::array<double, 4> kp_abad;
    std::array<double, 4> kp_hip;
    std::array<double, 4> kp_knee;

    std::array<double, 4> kd_abad;
    std::array<double, 4> kd_hip;
    std::array<double, 4> kd_knee;

    std::array<double, 4> tau_abad_ff;
    std::array<double, 4> tau_hip_ff;
    std::array<double, 4> tau_knee_ff;

    std::array<uint8_t, 4> flags;
  };

  /*!
      * Data from spine board
      */
  struct SPIData
  {
    std::array<double, 4> q_abad;
    std::array<double, 4> q_hip;
    std::array<double, 4> q_knee;
    std::array<double, 4> qd_abad;
    std::array<double, 4> qd_hip;
    std::array<double, 4> qd_knee;
    std::array<uint8_t, 4> flags;
    uint8_t spi_driver_status;
  };

  /*!
      * IMU
      */
  struct IMUData
  {
    Vector3 acc;
    Vector3 gyro;
    Vector4 quat;
    // todo is there status for the vectornav?
  };

  class Interface
  {
  public:
    SPICmd& GetSPICmdForUpdate() { return spi_cmd_; }
    const SPIData& GetSPIData() const { return spi_data_; }
    virtual bool Init() = 0;   // return true if ok
    virtual bool RunSPI() = 0; // return true if ok
  protected:
    SPICmd spi_cmd_;
    SPIData spi_data_;
  };

  using InterfacePtr = std::shared_ptr<Interface>;

} // namespace sdrobot::robot::interface

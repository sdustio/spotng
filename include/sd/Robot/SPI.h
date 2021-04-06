#pragma once

#include <cstdint>

namespace sd
{
  namespace robot
  {
    struct SpiCommand
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

      int32_t flags[4];
    };

    /*!
 * Data from spine board
 */
    struct SpiData
    {
      float q_abad[4];
      float q_hip[4];
      float q_knee[4];
      float qd_abad[4];
      float qd_hip[4];
      float qd_knee[4];
      int32_t flags[4];
      int32_t spi_driver_status;
    };

  } // namespace robot

} // namespace sd

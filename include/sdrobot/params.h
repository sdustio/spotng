#pragma once

#include "sdrobot/types.h"

namespace sdrobot::params
{
  namespace math
  {
    constexpr inline fpt_t const kZeroEpsilon = 1.e-12;
  }

  namespace drive
  {
    constexpr inline fpt_t const kMaxAngleR = 0.4;
    constexpr inline fpt_t const kMinAngleR = -0.4;
    constexpr inline fpt_t const kMaxAngleP = 0.4;
    constexpr inline fpt_t const kMinAngleP = -0.4;
    constexpr inline fpt_t const kMaxVelX = 3.0;
    constexpr inline fpt_t const kMinVelX = -3.0;
    constexpr inline fpt_t const kMaxVelY = 2.0;
    constexpr inline fpt_t const kMinVelY = -2.0;
    constexpr inline fpt_t const kMaxRateY = 2.5;
    constexpr inline fpt_t const kMinRateY = -2.5;
    constexpr inline fpt_t const kMaxVarHeight = 0.2;
    constexpr inline fpt_t const kMinVarHeight = -0.2;
    constexpr inline fpt_t const kMaxStepHeight = 0.2;
    constexpr inline fpt_t const kMinStepHeight = 0.05;
    constexpr inline fpt_t const kDeadbandRegion = 0.075;
    constexpr inline fpt_t const kFilter = 0.1;
  }

  namespace model
  {
    constexpr inline fpt_t const body_length = 0.19 * 2;
    constexpr inline fpt_t const body_width = 0.049 * 2;
    constexpr inline fpt_t const body_height = 0.05 * 2;
    constexpr inline fpt_t const body_mass = 3.3;

    constexpr inline fpt_t const abad_gear_ratio = 6.0;
    constexpr inline fpt_t const hip_gear_ratio = 6.0;
    constexpr inline fpt_t const knee_gear_ratio = 9.33;

    constexpr inline fpt_t const abad_link_length = 0.062;
    constexpr inline fpt_t const hip_link_length = 0.209;
    constexpr inline fpt_t const knee_link_length = 0.195;

    constexpr inline fpt_t const knee_link_y_offset = 0.004;
    constexpr inline fpt_t const max_leg_length = 0.409;

    constexpr inline fpt_t const motor_kt = 0.05;
    constexpr inline fpt_t const motor_r = 0.173;
    constexpr inline fpt_t const motor_tau_max = 3.0;
    constexpr inline fpt_t const battery_v = 24;

    constexpr inline fpt_t const joint_damping = 0.01;
    constexpr inline fpt_t const joint_dry_friction = 0.2;

    constexpr inline int const num_act_joint = 12;
    constexpr inline int const num_q = 19;
    constexpr inline int const num_leg = 4;
    constexpr inline int const num_leg_joint = 3;
    constexpr inline int const dim_config = 18; //TODO num_dof
    constexpr inline int const dim_floating = 6;

  } // namespace model

  namespace noise
  {
    constexpr inline fpt_t const kFootHeightSensorNoise = 0.001;
    constexpr inline fpt_t const kFootProcessNoisePosition = 0.002;
    constexpr inline fpt_t const kFootSensorNoisePosition = 0.001;
    constexpr inline fpt_t const kFootSensorNoiseVelocity = 0.1;
    constexpr inline fpt_t const kIMUProcessNoisePosition = 0.02;
    constexpr inline fpt_t const kIMUProcessNoiseVelocity = 0.02;
  }
} // namespace sdrobot::params

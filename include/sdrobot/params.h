#pragma once

#include "sdrobot/types.h"

namespace sdrobot::params
{
  namespace math
  {
    constexpr fpt_t kZeroEpsilon = 1.e-12;
  }

  namespace drive
  {
    constexpr fpt_t kMaxAngleR = 0.4;
    constexpr fpt_t kMinAngleR = -0.4;
    constexpr fpt_t kMaxAngleP = 0.4;
    constexpr fpt_t kMinAngleP = -0.4;
    constexpr fpt_t kMaxVelX = 3.0;
    constexpr fpt_t kMinVelX = -3.0;
    constexpr fpt_t kMaxVelY = 2.0;
    constexpr fpt_t kMinVelY = -2.0;
    constexpr fpt_t kMaxRateY = 2.5;
    constexpr fpt_t kMinRateY = -2.5;
    constexpr fpt_t kMaxVarHeight = 0.2;
    constexpr fpt_t kMinVarHeight = -0.2;
    constexpr fpt_t kMaxStepHeight = 0.2;
    constexpr fpt_t kMinStepHeight = 0.05;
    constexpr fpt_t kDeadbandRegion = 0.075;
    constexpr fpt_t kFilter = 0.1;
  }

  namespace model
  {
    constexpr fpt_t body_length = 0.19 * 2;
    constexpr fpt_t body_width = 0.049 * 2;
    constexpr fpt_t body_height = 0.05 * 2;
    constexpr fpt_t body_mass = 3.3;

    constexpr fpt_t abad_gear_ratio = 6.0;
    constexpr fpt_t hip_gear_ratio = 6.0;
    constexpr fpt_t knee_gear_ratio = 9.33;

    constexpr fpt_t abad_link_length = 0.062;
    constexpr fpt_t hip_link_length = 0.209;
    constexpr fpt_t knee_link_length = 0.195;

    constexpr fpt_t knee_link_y_offset = 0.004;
    constexpr fpt_t max_leg_length = 0.409;

    constexpr fpt_t motor_kt = 0.05;
    constexpr fpt_t motor_r = 0.173;
    constexpr fpt_t motor_tau_max = 3.0;
    constexpr fpt_t battery_v = 24;

    constexpr fpt_t joint_damping = 0.01;
    constexpr fpt_t joint_dry_friction = 0.2;

    constexpr int num_act_joint = 12;
    constexpr int num_q = 19;
    constexpr int dim_config = 18; //TODO num_dof
    constexpr int num_leg = 4;
    constexpr int num_leg_joint = 3;

  } // namespace model

  namespace noise
  {
    constexpr fpt_t kFootHeightSensorNoise = 0.001;
    constexpr fpt_t kFootProcessNoisePosition = 0.002;
    constexpr fpt_t kFootSensorNoisePosition = 0.001;
    constexpr fpt_t kFootSensorNoiseVelocity = 0.1;
    constexpr fpt_t kIMUProcessNoisePosition = 0.02;
    constexpr fpt_t kIMUProcessNoiseVelocity = 0.02;
  }
} // namespace sdrobot::params

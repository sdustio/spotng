#pragma once

namespace sdrobot::params
{
  constexpr fptype kZeroEpsilon = 1.e-12;

  namespace drive
  {
    constexpr fptype kMaxAngleR = 0.4;
    constexpr fptype kMinAngleR = -0.4;
    constexpr fptype kMaxAngleP = 0.4;
    constexpr fptype kMinAngleP = -0.4;
    constexpr fptype kMaxVelX = 3.0;
    constexpr fptype kMinVelX = -3.0;
    constexpr fptype kMaxVelY = 2.0;
    constexpr fptype kMinVelY = -2.0;
    constexpr fptype kMaxRateY = 2.5;
    constexpr fptype kMinRateY = -2.5;
    constexpr fptype kMaxVarHeight = 0.2;
    constexpr fptype kMinVarHeight = -0.2;
    constexpr fptype kMaxStepHeight = 0.2;
    constexpr fptype kMinStepHeight = 0.05;
    constexpr fptype kDeadbandRegion = 0.075;
    constexpr fptype kFilter = 0.1;
  }

  namespace model
  {
    constexpr fptype body_length = 0.19 * 2;
    constexpr fptype body_width = 0.049 * 2;
    constexpr fptype body_height = 0.05 * 2;
    constexpr fptype body_mass = 3.3;

    constexpr fptype abad_gear_ratio = 6.0;
    constexpr fptype hip_gear_ratio = 6.0;
    constexpr fptype knee_gear_ratio = 9.33;

    constexpr fptype abad_link_length = 0.062;
    constexpr fptype hip_link_length = 0.209;
    constexpr fptype knee_link_length = 0.195;

    constexpr fptype knee_link_y_offset = 0.004;
    constexpr fptype max_leg_length = 0.409;

    constexpr fptype motor_kt = 0.05;
    constexpr fptype motor_r = 0.173;
    constexpr fptype motor_tau_max = 3.0;
    constexpr fptype battery_v = 24;

    constexpr fptype joint_damping = 0.01;
    constexpr fptype joint_dry_friction = 0.2;

    constexpr int num_act_joint = 12;
    constexpr int num_q = 19;
    constexpr int dim_config = 18; //TODO num_dof
    constexpr int num_leg = 4;
    constexpr int num_leg_joint = 3;

  } // namespace model


  namespace noise
  {
    constexpr fptype kFootHeightSensorNoise = 0.001;
    constexpr fptype kFootProcessNoisePosition = 0.002;
    constexpr fptype kFootSensorNoisePosition = 0.001;
    constexpr fptype kFootSensorNoiseVelocity = 0.1;
    constexpr fptype kIMUProcessNoisePosition = 0.02;
    constexpr fptype kIMUProcessNoiseVelocity = 0.02;
  }
} // namespace sdrobot::params

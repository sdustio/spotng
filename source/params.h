#pragma once

namespace sdrobot::params
{
  constexpr double kZeroEpsilon = 1.e-12;

  namespace cmd
  {
    constexpr double kMaxAngleR = 0.4;
    constexpr double kMinAngleR = -0.4;
    constexpr double kMaxAngleP = 0.4;
    constexpr double kMinAngleP = -0.4;
    constexpr double kMaxVelX = 3.0;
    constexpr double kMinVelX = -3.0;
    constexpr double kMaxVelY = 2.0;
    constexpr double kMinVelY = -2.0;
    constexpr double kMaxRateY = 2.5;
    constexpr double kMinRateY = -2.5;
    constexpr double kMaxVarHeight = 0.2;
    constexpr double kMinVarHeight = -0.2;
    constexpr double kMaxStepHeight = 0.2;
    constexpr double kMinStepHeight = 0.05;
    constexpr double kDeadbandRegion = 0.075;
    constexpr double kFilter = 0.1;
  }

  namespace model
  {
    constexpr double body_length = 0.19 * 2;
    constexpr double body_width = 0.049 * 2;
    constexpr double body_height = 0.05 * 2;
    constexpr double body_mass = 3.3;

    constexpr double abad_gear_ratio = 6.0;
    constexpr double hip_gear_ratio = 6.0;
    constexpr double knee_gear_ratio = 9.33;

    constexpr double abad_link_length = 0.062;
    constexpr double hip_link_length = 0.209;
    constexpr double knee_link_length = 0.195;

    constexpr double knee_link_y_offset = 0.004;
    constexpr double max_leg_length = 0.409;

    constexpr double motor_kt = 0.05;
    constexpr double motor_r = 0.173;
    constexpr double motor_tau_max = 3.0;
    constexpr double battery_v = 24;

    constexpr double joint_damping = 0.01;
    constexpr double joint_dry_friction = 0.2;

    constexpr int num_act_joint = 12;
    constexpr int num_q = 19;
    constexpr int dim_config = 18;
    constexpr int num_leg = 4;
    constexpr int num_leg_joint = 3;

  } // namespace model


  namespace ctrl
  {
    constexpr int kActItfmsec = int(1.0 / 0.04); //unit: milliseconds; 0.04kHz
    constexpr int kCtrlmsec = int(1.0 / 0.5);    //unit: milliseconds; 0.5kHz
    constexpr double kActItfsec = kActItfmsec / 1'000;
    constexpr double kCtrlsec = kCtrlmsec / 1'000;

    constexpr double kFootHeightSensorNoise = 0.001;
    constexpr double kFootProcessNoisePosition = 0.002;
    constexpr double kFootSensorNoisePosition = 0.001;
    constexpr double kFootSensorNoiseVelocity = 0.1;
    constexpr double kIMUProcessNoisePosition = 0.02;
    constexpr double kIMUProcessNoiseVelocity = 0.02;
  }
} // namespace sdrobot::params

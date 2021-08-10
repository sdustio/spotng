#pragma once

#include "sdrobot/types.h"

namespace sdrobot::params
{
  namespace math
  {
    constexpr inline fpt_t const kZeroEpsilon = 1.e-12;
    constexpr inline fpt_t const kPI = 3.14159265358979323846;
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
    constexpr inline fpt_t const kBodyLength = 0.19 * 2;
    constexpr inline fpt_t const kBodyWidth = 0.049 * 2;
    constexpr inline fpt_t const kBodyHeight = 0.05 * 2;
    constexpr inline fpt_t const kBodyMass = 3.3;

    constexpr inline fpt_t const kAbadGearRatio = 6.0;
    constexpr inline fpt_t const kHipGearRatio = 6.0;
    constexpr inline fpt_t const kKneeGearRatio = 9.33;

    constexpr inline fpt_t const kAbadLinkLength = 0.062;
    constexpr inline fpt_t const kHipLinkLength = 0.209;
    constexpr inline fpt_t const kKneeLinkLength = 0.195;

    constexpr inline fpt_t const kKneeLinkYOffset = 0.004;
    constexpr inline fpt_t const kMaxLegLength = 0.409;

    constexpr inline fpt_t const kMotorKt = 0.05;
    constexpr inline fpt_t const kMotorR = 0.173;
    constexpr inline fpt_t const kMotorTauMax = 3.0;
    constexpr inline fpt_t const kBatteryV = 24;

    constexpr inline fpt_t const kJointDamping = 0.01;
    constexpr inline fpt_t const kJointDryFriction = 0.2;

    constexpr inline int const kNumActJoint = 12;
    constexpr inline int const kNumQ = 19;
    constexpr inline int const kNumLeg = 4;
    constexpr inline int const kNumLegJoint = 3;
    constexpr inline int const kDimConfig = 18;
    constexpr inline int const kDimFloating = 6;

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

  namespace interface
  {
    constexpr inline fpt_t const kMaxTorque = 24.0;
    constexpr inline fpt_t const kMaxAngle = 1.0472; // 60 degrees (should be changed)
    constexpr inline fpt_t const kMaxLateralForce = 350.;
    constexpr inline fpt_t const kMaxVerticalForce = 350.;
  }

} // namespace sdrobot::params

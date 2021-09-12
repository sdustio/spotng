#pragma once

#include "sdquadx/types.h"

namespace sdquadx::consts {
namespace math {
constexpr inline fpt_t const kZeroEpsilon = 1.e-12;
constexpr inline fpt_t const kPI = 3.14159265358979323846;
}  // namespace math

namespace drive {
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
constexpr inline fpt_t const kDefaultStepHeight = 0.15;
}  // namespace drive

namespace model {
constexpr inline fpt_t const kMaxLegLength = 0.55;  //?

constexpr inline int const kNumActJoint = 12;
constexpr inline int const kNumQ = 19;
constexpr inline int const kNumLeg = 4;
constexpr inline int const kNumLegJoint = 3;
constexpr inline int const kDimConfig = 18;
constexpr inline int const kDimFloating = 6;

}  // namespace model

namespace noise {
constexpr inline fpt_t const kFootHeightSensorNoise = 0.001;
constexpr inline fpt_t const kFootProcessNoisePosition = 0.002;
constexpr inline fpt_t const kFootSensorNoisePosition = 0.001;
constexpr inline fpt_t const kFootSensorNoiseVelocity = 0.1;
constexpr inline fpt_t const kIMUProcessNoisePosition = 0.02;
constexpr inline fpt_t const kIMUProcessNoiseVelocity = 0.02;
}  // namespace noise

namespace interface {
constexpr inline fpt_t const kMaxTorque = 40.0;
constexpr inline fpt_t const kMaxAngle = 1.0472;  // 60 degrees (should be changed)
constexpr inline fpt_t const kMaxLateralForce = 350.;
constexpr inline fpt_t const kMaxVerticalForce = 350.;
}  // namespace interface

}  // namespace sdquadx::consts

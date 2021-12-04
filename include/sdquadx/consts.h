#pragma once

#include "sdquadx/types.h"

namespace sdquadx::consts {
namespace math {
constexpr inline fpt_t const kZeroEpsilon = 1.e-12;
constexpr inline fpt_t const kPI = 3.14159265358979323846;
constexpr inline fpt_t const kBigNum = 5e10;
}  // namespace math

namespace legidx {
constexpr inline int const fr = 0;  // Front Right
constexpr inline int const fl = 1;  // Front Left
constexpr inline int const hr = 2;  // Hind Right
constexpr inline int const hl = 3;  // Hind Left
}  // namespace legidx

namespace drive {
constexpr inline fpt_t const kMaxAngleR = 0.4;
constexpr inline fpt_t const kMinAngleR = -0.4;
constexpr inline fpt_t const kMaxAngleP = 0.4;
constexpr inline fpt_t const kMinAngleP = -0.4;
constexpr inline fpt_t const kMaxAngleY = 0.4;
constexpr inline fpt_t const kMinAngleY = -0.4;
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

constexpr inline int const kNumJoint = 12;
constexpr inline int const kNumQ = 19;
constexpr inline int const kNumLeg = 4;
constexpr inline int const kNumLegJoint = 3;
constexpr inline int const kDimConfig = 18;
constexpr inline int const kDimFloating = 6;

constexpr inline std::array<fpt_t, kNumLeg> const kSignLR = {-1., 1., -1., 1.};
constexpr inline std::array<fpt_t, kNumLeg> const kSignFH = {1., 1., -1., -1.};

}  // namespace model

namespace interface {
constexpr inline fpt_t const kMaxTorque = 40.0;
constexpr inline fpt_t const kMaxAngle = 1.0472;  // 60 degrees (should be changed)
constexpr inline fpt_t const kMaxLateralForce = 350.;
constexpr inline fpt_t const kMaxVerticalForce = 350.;
}  // namespace interface

}  // namespace sdquadx::consts

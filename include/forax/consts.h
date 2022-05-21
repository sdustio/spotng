#pragma once

#include "forax/types.h"

namespace forax::consts {
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

namespace ctrl {
constexpr inline int const kPredLength = 10;
}  // namespace ctrl

}  // namespace forax::consts

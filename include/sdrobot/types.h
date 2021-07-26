#pragma once

#include <array>
#include <cstdint>

#include "sdrobot/export.h"

namespace sdrobot
{
  using fptype = double;
  using Array3f = std::array<fptype, 3>;
  using Array3c = std::array<std::uint8_t, 3>;
  using Array3i = std::array<int, 3>;

  using Array4f = std::array<fptype, 4>;
  using Array4c = std::array<std::uint8_t, 4>;
  using Array4i = std::array<int, 4>;

  using Matrix3f = std::array<fptype, 3 * 3>;
  using Matrix4f = std::array<fptype, 4 * 4>;

  constexpr double kZeroEpsilon = 1.e-12;
}

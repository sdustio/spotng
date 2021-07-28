#pragma once

#include <array>
#include <cstdint>

#include "sdrobot/export.h"

namespace sdrobot
{
  using fptype = double;
  using SdArray3f = std::array<fptype, 3>;
  using SdArray3c = std::array<std::uint8_t, 3>;
  using SdArray3i = std::array<int, 3>;

  using SdArray4f = std::array<fptype, 4>;
  using SdArray4c = std::array<std::uint8_t, 4>;
  using SdArray4i = std::array<int, 4>;

  using SdMatrix3f = std::array<fptype, 3 * 3>;
  using SdMatrix4f = std::array<fptype, 4 * 4>;
}

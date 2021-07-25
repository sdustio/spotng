#pragma once

#include <array>
#include <cinttypes>

namespace sdrobot
{
  using fptype = double;
  using Array3f = std::array<fptype, 3>;
  using Array3c = std::array<std::uint8_t, 3>;
  using Array3i = std::array<int, 3>;

  using Array4f = std::array<fptype, 4>;
  using Array4c = std::array<std::uint8_t, 4>;
  using Array4i = std::array<int, 4>;
}

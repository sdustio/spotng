#pragma once

#include <array>
#include <vector>
#include <cstdint>

#include "sdrobot/export.h"

namespace sdrobot
{
  using fptype = double;
  using SdVector3f = std::array<fptype, 3>;
  using SdVector3c = std::array<std::uint8_t, 3>;
  using SdVector3i = std::array<int, 3>;

  using SdVector4f = std::array<fptype, 4>;
  using SdVector4c = std::array<std::uint8_t, 4>;
  using SdVector4i = std::array<int, 4>;

  using SdMatrix3f = std::array<fptype, 3 * 3>;
  using SdMatrix4f = std::array<fptype, 4 * 4>;
  using SdMatrix6f = std::array<fptype, 6 * 6>;

  using SdVectorXf = std::vector<fptype>;
  using SdMatrixXf = std::vector<fptype>;
}

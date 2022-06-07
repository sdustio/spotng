#pragma once

#include <array>
#include <vector>

#include "spotng/export.h"

namespace spotng {
using fpt_t = double;

using SdVector3f = std::array<fpt_t, 3>;
using SdVector4f = std::array<fpt_t, 4>;
using SdVector6f = std::array<fpt_t, 6>;

using SdMatrix3f = std::array<fpt_t, 3 * 3>;
using SdMatrix4f = std::array<fpt_t, 4 * 4>;
using SdMatrix6f = std::array<fpt_t, 6 * 6>;

using SdVectorXf = std::vector<fpt_t>;
using SdMatrixXf = std::vector<fpt_t>;
}  // namespace spotng

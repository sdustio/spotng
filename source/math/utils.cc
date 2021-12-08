#include "math/utils.h"

namespace sdquadx::math {
bool HasNaN(fpt_t const* begin, fpt_t const* end) {
  while (begin != end) {
    if (std::isnan(*begin)) return true;
    begin++;
  }
  return false;
}
}  // namespace sdquadx::math

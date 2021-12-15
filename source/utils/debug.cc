#include "utils/debug.h"

#include "spdlog/spdlog.h"

namespace sdquadx {
void DebugVector(std::string const &prefix, SdVector3f const &v) {
  spdlog::debug(prefix + ": [{}, {}, {}]", v[0], v[1], v[2]);
}

void DebugVector(std::string const &prefix, SdVector4f const &v) {
  spdlog::debug(prefix + ": [{}, {}, {}, {}]", v[0], v[1], v[2], v[3]);
}
}  // namespace sdquadx

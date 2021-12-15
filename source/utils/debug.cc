#include "utils/debug.h"

#include <string>

#include "spdlog/spdlog.h"

namespace sdquadx {
void DebugVector(char const *prefix, SdVector3f const &v) {
  spdlog::debug(std::string(prefix) + ": [{}, {}, {}]", v[0], v[1], v[2]);
}

void DebugVector(char const *prefix, SdVector4f const &v) {
  spdlog::debug(std::string(prefix) + ": [{}, {}, {}, {}]", v[0], v[1], v[2], v[3]);
}
}  // namespace sdquadx

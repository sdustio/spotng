#pragma once

#include <string>

#include "sdquadx/types.h"

namespace sdquadx {
void DebugVector(std::string const &prefix, SdVector3f const &v);
void DebugVector(std::string const &prefix, SdVector4f const &v);
}  // namespace sdquadx

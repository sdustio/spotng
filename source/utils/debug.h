#pragma once

#include <string>

#include "spotng/types.h"

namespace spotng {
void DebugVector(std::string const &prefix, SdVector3f const &v);
void DebugVector(std::string const &prefix, SdVector4f const &v);
}  // namespace spotng

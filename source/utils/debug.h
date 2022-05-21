#pragma once

#include <string>

#include "forax/types.h"

namespace forax {
void DebugVector(std::string const &prefix, SdVector3f const &v);
void DebugVector(std::string const &prefix, SdVector4f const &v);
}  // namespace forax

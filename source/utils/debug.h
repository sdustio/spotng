#pragma once

#include <string>

#include "sdengine/types.h"

namespace sdengine {
void DebugVector(std::string const &prefix, SdVector3f const &v);
void DebugVector(std::string const &prefix, SdVector4f const &v);
}  // namespace sdengine

#pragma once

namespace spotng {
namespace logging {
enum class Level : uint8_t { Debug, Info, Warn, Err, Critical };

enum class Target : uint8_t { Console, File, RotateFile };
}  // namespace logging

}  // namespace spotng

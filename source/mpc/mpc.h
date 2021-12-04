#pragma once

#include <memory>

#include "sdquadx/drive.h"
#include "sdquadx/estimate.h"
#include "sdquadx/model.h"
#include "wbc/wbc.h"

namespace sdquadx::mpc {
namespace params {
constexpr inline int const cmpc_x_drag = 3;
}  // namespace params

using SdVector4i = std::array<int, 4>;

class Mpc {
 public:
  using Ptr = std::unique_ptr<Mpc>;
  using SharedPtr = std::shared_ptr<Mpc>;
  using ConstSharedPtr = std::shared_ptr<Mpc const>;

  virtual ~Mpc() = default;
  virtual bool RunOnce(wbc::InData &, estimate::State const &, std::vector<int> const &) = 0;
};

}  // namespace sdquadx::mpc

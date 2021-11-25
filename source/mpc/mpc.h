#pragma once

#include <memory>

#include "sdquadx/drive.h"
#include "sdquadx/estimate.h"
#include "sdquadx/model.h"
#include "wbc/wbc.h"

namespace sdquadx::mpc {
namespace opts {
constexpr inline int const bonus_swing = 0;
constexpr inline int const horizon_len = 10;
constexpr inline fpt_t const big_num = 5e10;
constexpr inline int const cmpc_x_drag = 3;
}  // namespace opts

using SdVector4i = std::array<int, 4>;

class Mpc {
 public:
  using Ptr = std::unique_ptr<Mpc>;
  using SharedPtr = std::shared_ptr<Mpc>;
  using ConstSharedPtr = std::shared_ptr<Mpc const>;

  virtual ~Mpc() = default;
  virtual bool RunOnce(interface::LegCmds &, wbc::InData &, estimate::State const &, drive::DriveCtrl::ConstSharedPtr const &) = 0;
};

}  // namespace sdquadx::mpc

#pragma once

#include <memory>

#include "sdrobot/drive.h"
#include "sdrobot/estimate.h"
#include "sdrobot/leg.h"
#include "sdrobot/model.h"
#include "wbc/wbc.h"

namespace sdrobot::mpc {
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
  virtual bool Init() = 0;
  virtual bool RunOnce(
      wbc::InData &wbcdata, leg::LegCtrl::SharedPtr const &legctrl,
      model::Quadruped::ConstSharedPtr const &quad,
      drive::DriveCtrl::ConstSharedPtr const &drivectrl,
      estimate::EstimateCtrl::ConstSharedPtr const &estctrl) = 0;
};

}  // namespace sdrobot::mpc

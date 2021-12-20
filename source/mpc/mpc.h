#pragma once

#include <memory>

#include "sdquadx/drive.h"
#include "sdquadx/estimate.h"
#include "sdquadx/model.h"
#include "skd/gait.h"
#include "wbc/wbc.h"

namespace sdquadx::mpc {

using SdVector4i = std::array<int, 4>;

class Mpc {
 public:
  using Ptr = std::unique_ptr<Mpc>;
  using SharedPtr = std::shared_ptr<Mpc>;
  using ConstSharedPtr = std::shared_ptr<Mpc const>;

  virtual ~Mpc() = default;
  virtual bool RunOnce(wbc::InData &, estimate::State const &, skd::PredStanceVector const &) = 0;
};

}  // namespace sdquadx::mpc

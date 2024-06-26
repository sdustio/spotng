#pragma once

#include <memory>

#include "skd/gait.h"
#include "spotng/drive.h"
#include "spotng/estimate.h"
#include "spotng/model.h"
#include "wbc/wbc.h"

namespace spotng::mpc {

using SdVector4i = std::array<int, 4>;

class Mpc {
 public:
  using Ptr = std::unique_ptr<Mpc>;
  using SharedPtr = std::shared_ptr<Mpc>;
  using ConstSharedPtr = std::shared_ptr<Mpc const>;

  virtual ~Mpc() = default;
  virtual bool RunOnce(wbc::InData &, estimate::State const &, skd::PredStanceVector const &) = 0;
};

}  // namespace spotng::mpc

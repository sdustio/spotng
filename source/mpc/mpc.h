#pragma once

#include "sdrobot/drive.h"
#include "sdrobot/estimate.h"
#include "sdrobot/leg.h"
#include "sdrobot/model.h"
#include "wbc/in_data.h"

namespace sdrobot::mpc
{
  namespace params
  {
    constexpr inline int const bonus_swing = 0;
    constexpr inline int const horizon_len = 10;
    constexpr inline fpt_t const big_num = 5e10;
    constexpr inline int const cmpc_x_drag = 3;
  }

  using SdVector4i = std::array<int, 4>;

  class Mpc
  {
  public:
    using Ptr = std::unique_ptr<Mpc>;
    using SharedPtr = std::shared_ptr<Mpc>;

    virtual ~Mpc() = default;
    virtual bool Init() = 0;
    virtual bool Run(wbc::InData &wbcdata,
                     leg::LegCtrl::SharedPtr &legctrl,
                     model::Quadruped::SharedPtr const &quad,
                     drive::DriveCtrl::SharedPtr const &drivectrl,
                     estimate::EstimateCtrl::SharedPtr const &estctrl) = 0;
  };

}

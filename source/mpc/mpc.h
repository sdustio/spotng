#pragma once

#include "sdrobot/drive.h"
#include "sdrobot/estimate.h"
#include "sdrobot/leg.h"
#include "sdrobot/model.h"


namespace sdrobot::mpc
{

  class Mpc
  {
  public:
    using Ptr = std::unique_ptr<Mpc>;
    using SharedPtr = std::shared_ptr<Mpc>;

    virtual ~Mpc() = default;
    virtual bool Init() = 0;
    virtual bool Run(WbcData &data, LegPtr &cleg, const robot::QuadrupedPtr &quad, const StateCmdPtr &cmd, const est::StateEstPtr &est) = 0;
  };

}

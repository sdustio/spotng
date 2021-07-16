#pragma once

#include <memory>
#include <array>

#include "sdrobot/types.h"
#include "sdrobot/estimators/state_est.h"
#include "sdrobot/controllers/leg.h"
#include "sdrobot/controllers/state_cmd.h"
#include "sdrobot/controllers/wbc.h"


namespace sdrobot::ctrl
{

  class Mpc
  {
  public:
    virtual bool Init() = 0;
    virtual bool Run(WbcData &data, LegPtr &cleg, const robot::QuadrupedPtr &quad, const StateCmdPtr &cmd, const est::StateEstPtr &est) = 0;
  };

  using MpcPtr = std::shared_ptr<Mpc>;
}

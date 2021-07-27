#pragma once

#include "sdrobot/estimate.h"

namespace sdrobot::estimate
{
  class Orientation : public Estimator
  {
  public:
    bool Update();
    bool RunOnce(State &ret) override;

  };
}

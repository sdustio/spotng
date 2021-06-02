#pragma once

#include "sd/dynamics/state.h"

namespace sd::est
{
  template <typename T>
  class StateEstimate
  {
  public:
    StateVec<T> state;
  };
}

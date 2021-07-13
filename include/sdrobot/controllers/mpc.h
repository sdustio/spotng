#pragma once

#include <memory>

namespace sdrobot::ctrl
{
  class Mpc
  {
  public:
    virtual bool Init() = 0;
    virtual bool Run() = 0;
  };

  using MpcPtr = std::shared_ptr<Mpc>;
}

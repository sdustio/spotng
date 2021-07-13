#pragma once

#include "sdrobot/controllers/mpc.h"

namespace sdrobot::ctrl::mpc
{

  class CMpc : public Mpc
  {
  public:
    CMpc(double _dt, int _iterations_between_mpc);
    bool Init() override { return true; }
    bool Run() override { return true; }

  private:
    double dt_;
    int iterationsBetweenMPC;
  };
}

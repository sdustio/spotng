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
    const MpcData &GetData() override { return data_; }

  private:
    double dt_;
    int iterationsBetweenMPC;
    MpcData data_;
  };
}

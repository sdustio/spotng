#pragma once

#include "sd/Types.h"
#include "sd/Dynamics/State.h"

namespace sd::dynamics
{
  template <typename T>
  class DesiredStateCmd
  {

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit DesiredStateCmd(float dt) : mDt(dt){mStateDes = Vec12<T>::Zero();}

    bool UpdateCmd(float mv_x, float mv_y, float tr, float pa, Mode m)
    {
      mCmdMode = m;
      if (mCmdMode == Mode::Stand || mCmdMode == Mode::RecoveryStand)
      {
        mv_x = 0.0;
        mv_y = 0.0;
      }
      mCmdMvX = mCmdMvX * (T(1) - CmdLimits::Filter) + mv_x * CmdLimits::Filter;
      mCmdMvY = mCmdMvY * (T(1) - CmdLimits::Filter) + mv_y * CmdLimits::Filter;
      mCmdTr = mCmdTr * (T(1) - CmdLimits::Filter) + tr * CmdLimits::Filter;
      mCmdPa = mCmdPa * (T(1) - CmdLimits::Filter) + pa * CmdLimits::Filter;
      return true;
    }

  private:
    bool CmdtoStateData()
    {

      mStateDes.setZero();
      mStateDes(StateIdx::VelX) = Deadband(mCmdMvX, CmdLimits::MinVelX, CmdLimits::MaxVelX);
      mStateDes(StateIdx::VelY) = Deadband(mCmdMvY, CmdLimits::MinVelY, CmdLimits::MaxVelY);
      mStateDes(StateIdx::VelZ) = 0.0;
      mStateDes(StateIdx::PosX) = mDt * mStateDes(StateIdx::VelX);
      mStateDes(StateIdx::PosY) = mDt * mStateDes(StateIdx::VelY);
      mStateDes(StateIdx::PosZ) = 0.26;
      mStateDes(StateIdx::RateR) = 0.0;
      mStateDes(StateIdx::RateP) = 0.0;
      mStateDes(StateIdx::RateY) = Deadband(mCmdTr, CmdLimits::MinRateYaw, CmdLimits::MaxRateYaw);
      mStateDes(StateIdx::AngleR) = 0.0;
      mStateDes(StateIdx::AngleP) = Deadband(mCmdPa, CmdLimits::MinAngleP, CmdLimits::MaxAngleP);
      mStateDes(StateIdx::AngleY) = mDt * mStateDes(StateIdx::RateY);

      return true;
    }

    float Deadband(float v, float minVal, float maxVal)
    {
      if (v < CmdLimits::DeadbandRegion && v > -CmdLimits::DeadbandRegion)
      {
        return 0.0;
      }
      else
      {
        return (v / 2) * (maxVal - minVal);
      }
    }

    float mCmdMvX;
    float mCmdMvY;
    float mCmdTr;
    float mCmdPa;
    Mode mCmdMode;

    // Dynamics matrix for discrete time approximation
    Mat12<T> mA;
    Vec12<T> mStateDes;
    T mDt;
  };

}

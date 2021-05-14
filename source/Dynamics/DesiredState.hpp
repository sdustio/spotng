#pragma once

#include "sd/Types.h"
#include "sd/Dynamics/State.h"

namespace sd::dynamics
{
  template <typename T>
  class DesiredStateData
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    DesiredStateData() { zero(); }

    void zero()
    {
      mStateDes = Vec12<T>::Zero();
      mPreStateDes = Vec12<T>::Zero();
      mStateTrajDes = Eigen::Matrix<T, 12, 10>::Zero();
    }

    // Instantaneous desired state command 瞬时期望状态指令
    Vec12<T> mStateDes;
    Vec12<T> mPreStateDes;
    // Desired future state trajectory (for up to 10 timestep MPC) 期望的未来状态轨迹(最多10个时间步长MPC)
    Eigen::Matrix<T, 12, 10> mStateTrajDes;
  };

  template <typename T>
  class DesiredStateCmd
  {

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit DesiredStateCmd(float dt) : mDt(dt){};

    bool UpdateCmd(float mv_x, float mv_y, float tr, float pa, Mode m)
    {
      {
        mCmdMvX = mv_x;
        mCmdMvY = mv_y;
        mCmdTr = tr;
        mCmdPa = pa;
        mCmdMode = m;
        return true;
      }
    }

  private:
    float mCmdMvX;
    float mCmdMvY;
    float mCmdTr;
    float mCmdPa;
    Mode mCmdMode;

    // Dynamics matrix for discrete time approximation
    Mat12<T> mA;
    DesiredStateData<T> mData;
    float mDt;
  };

}

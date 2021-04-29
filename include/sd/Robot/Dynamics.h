#pragma once

#include "sd/Types.h"

namespace sd::robot::dynamics
{
  enum DesiredStateIdx
  {
    P_X, // X position
    P_Y, // Y position
    P_Z, // Z position
    A_R, // Roll angle
    A_P, // Pitch angle
    A_Y, // Yaw angle
    V_X, // X linear velocity
    V_Y, // Y linear velocity
    V_Z, // Z linear velocity
    R_R, // Roll rate
    R_P, // Pitch rate
    R_Y  // Yaw rate
  };

  enum Mode
  {
    Off,
    Stand,
    RecoveryStand,
    Locomotion,
    Vision
  };

  class DesiredStateData
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    DesiredStateData() { zero(); }

    void zero();

    // Instantaneous desired state command 瞬时期望状态指令
    Vec12<float> mStateDes;
    Vec12<float> mPreStateDes;
    // Desired future state trajectory (for up to 10 timestep MPC) 期望的未来状态轨迹(最多10个时间步长MPC)
    Eigen::Matrix<float, 12, 10> mStateTrajDes;
  };

  class DesiredStateCmd
  {

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit DesiredStateCmd(/* args */);

    int UpdateCmd(float mv_x, float mv_y, float tr, float pa, Mode m);

  private:
    // Dynamics matrix for discrete time approximation
    Mat12<float> mA;
    float mCmdMvX;
    float mCmdMvY;
    float mCmdTr;
    float mCmdPa;
    Mode mCmdMode;
  };

} // namespace sd::robot::dynamics

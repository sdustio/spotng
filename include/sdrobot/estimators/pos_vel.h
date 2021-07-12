#pragma once

#include <memory>

#include "sdrobot/robot/model.h"
#include "sdrobot/estimators/state_est.h"

namespace sdrobot::est
{

  class PosVel
  {
  public:
    bool Setup();
    bool Run(StateData &ret, const robot::leg::Datas &datas, const robot::QuadrupedPtr &quad);

  private:
    Vector18d xhat_;                  //状态估计值 [p v p1 p2 p3 p4] 世界坐标下
    Vector12d ps_;                    //储存状态p
    Vector12d vs_;                    //储存状态v
    Matrix18d A_;                     //状态转移阵
    Matrix18d Q0_;                    //初始状态估计噪声
    Matrix18d P_;                     //初始不确定性
    Matrix28d R0_;                    //初始观测噪声
    Eigen::Matrix<double, 18, 3> B_;  //输入阵
    Eigen::Matrix<double, 28, 18> C_; //观测阵
  };

  using PosVelPtr = std::shared_ptr<PosVel>;

}
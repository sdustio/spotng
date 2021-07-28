#pragma once

#include "sdrobot/estimate.h"
#include "sdrobot/leg.h"
#include "sdrobot/model.h"

namespace sdrobot::estimate
{
  class PosVel : public Estimator
  {
  public:
    PosVel(double dt, leg::LegCtrl::SharedPtr const &legctrl, model::Quadruped::SharedPtr const &quad);
    bool Init() override;
    bool RunOnce(State &ret) override;

  private:
    double dt_;
    leg::LegCtrl::SharedPtr const legctrl_;
    model::Quadruped::SharedPtr const quad_;

    std::array<fptype, 18> xhat_;    //状态估计值 [p v p1 p2 p3 p4] 世界坐标下
    std::array<fptype, 12> ps_;      //储存状态p
    std::array<fptype, 12> vs_;      //储存状态v
    std::array<fptype, 18 * 18> A_;  //状态转移阵
    std::array<fptype, 18 * 18> Q0_; //初始状态估计噪声
    std::array<fptype, 18 * 18> P_;  //初始不确定性
    std::array<fptype, 28 * 28> R0_; //初始观测噪声
    std::array<fptype, 18 * 3> B_;   //输入阵
    std::array<fptype, 28 * 18> C_;  //观测阵
  };
}

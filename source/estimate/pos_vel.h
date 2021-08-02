#pragma once

#include "sdrobot/estimate.h"
#include "sdrobot/leg.h"
#include "sdrobot/model.h"

namespace sdrobot::estimate
{
  class PosVel : public Estimator
  {
  public:
    PosVel(fpt_t dt, fpt_t gravity, leg::LegCtrl::SharedPtr const &legctrl, model::Quadruped::SharedPtr const &quad);
    bool Init() override;
    bool RunOnce(State &ret) override;

  private:
    fpt_t dt_;
    fpt_t gravity_;
    leg::LegCtrl::SharedPtr const legctrl_;
    model::Quadruped::SharedPtr const quad_;

    std::array<fpt_t, 18> xhat_;    //状态估计值 [p v p1 p2 p3 p4] 世界坐标下
    std::array<fpt_t, 12> ps_;      //储存状态p
    std::array<fpt_t, 12> vs_;      //储存状态v
    std::array<fpt_t, 18 * 18> A_;  //状态转移阵
    std::array<fpt_t, 18 * 18> Q0_; //初始状态估计噪声
    std::array<fpt_t, 18 * 18> P_;  //初始不确定性
    std::array<fpt_t, 28 * 28> R0_; //初始观测噪声
    std::array<fpt_t, 18 * 3> B_;   //输入阵
    std::array<fpt_t, 28 * 18> C_;  //观测阵
  };
}

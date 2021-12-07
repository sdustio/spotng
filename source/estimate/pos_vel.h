#pragma once

#include "utils/eigen.h"
#include "sdquadx/estimate.h"
#include "sdquadx/options.h"

namespace sdquadx::estimate {
class PosVel : public Estimator {
 public:
  PosVel(Options::ConstSharedPtr const &opts);
  bool RunOnce(State &ret) override;

 private:
  using Vector12 = Eigen::Matrix<fpt_t, 12, 1>;
  using Vector18 = Eigen::Matrix<fpt_t, 18, 1>;
  using Vector28 = Eigen::Matrix<fpt_t, 28, 1>;
  using Matrix18 = Eigen::Matrix<fpt_t, 18, 18>;
  using Matrix28 = Eigen::Matrix<fpt_t, 28, 28>;

  bool CalcFootPosVelRobot(SdVector3f &pos, SdVector3f &vel, int leg, State const &data);

  Options::ConstSharedPtr const opts_;

  std::array<fpt_t, 18> xhat_;     // 状态估计值 [p v p1 p2 p3 p4] 世界坐标下
  std::array<fpt_t, 12> ps_;       // 储存状态p
  std::array<fpt_t, 12> vs_;       // 储存状态v
  std::array<fpt_t, 18 * 18> A_;   // 状态转移阵
  std::array<fpt_t, 18 * 18> Q0_;  // 初始状态估计噪声
  std::array<fpt_t, 18 * 18> P_;   // 初始不确定性
  std::array<fpt_t, 28 * 28> R0_;  // 初始观测噪声
  std::array<fpt_t, 18 * 3> B_;    // 输入阵
  std::array<fpt_t, 28 * 18> C_;   // 观测阵
};
}  // namespace sdquadx::estimate

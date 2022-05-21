#pragma once

#include "forax/estimate.h"
#include "forax/options.h"
#include "utils/eigen.h"

namespace forax::estimate {
class PosVel : public Estimator {
 public:
  explicit PosVel(Options::ConstSharedPtr const &opts);
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
  std::array<fpt_t, 18 * 18> Q0_;  // 初始状态转移协方差
  std::array<fpt_t, 18 * 18> Q_;   // 状态转移协方差
  std::array<fpt_t, 18 * 18> P_;   // 估计协方差矩阵
  std::array<fpt_t, 28 * 28> R0_;  // 初始观测噪声协方差
  std::array<fpt_t, 28 * 28> R_;   // 观测噪声协方差
  std::array<fpt_t, 18 * 3> B_;    // 输入阵
  std::array<fpt_t, 28 * 18> C_;   // 观测阵
};
}  // namespace forax::estimate

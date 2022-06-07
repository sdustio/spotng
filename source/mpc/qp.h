#pragma once
#include <array>
#include <qpOASES.hpp>

#include "spotng/consts.h"

namespace spotng::mpc {

using consts::ctrl::kPredLength;
constexpr inline int const kDimXD = 13 * kPredLength;
constexpr inline int const kNumConstraints = 20 * kPredLength;
constexpr inline int const kNumVariables = 12 * kPredLength;

struct QpData {
  QpData();
  void Zero();
  std::array<fpt_t, kNumVariables> qsoln = {};

  std::array<fpt_t, (kDimXD * 13)> A_qp = {};
  std::array<fpt_t, (kDimXD * kNumVariables)> B_qp = {};
  std::array<fpt_t, (kDimXD * kDimXD)> S = {};
  std::array<fpt_t, (kNumVariables * kNumVariables)> eye_12h = {};
  std::array<fpt_t, 13> x_0 = {};
  std::array<fpt_t, kDimXD> X_d = {};

  std::array<fpt_t, (kNumVariables * kNumVariables)> qH = {};
  std::array<fpt_t, (kNumConstraints * kNumVariables)> qA = {};
  std::array<fpt_t, kNumConstraints> qub = {};
  std::array<fpt_t, kNumConstraints> qlb = {};
  std::array<fpt_t, kNumVariables> qg = {};
};

class QpSolver {
 public:
  QpSolver() = default;
  bool Solve(QpData &data);

 private:
  void Reset();

  std::array<int, kNumVariables> var_elim_ = {};
  std::array<int, kNumConstraints> con_elim_ = {};

  std::array<int, kNumVariables> var_ind_ = {};
  std::array<int, kNumConstraints> con_ind_ = {};

  std::array<qpOASES::real_t, (kNumVariables * kNumVariables)> H_red_ = {};
  std::array<qpOASES::real_t, kNumConstraints *kNumVariables> A_red_ = {};
  std::array<qpOASES::real_t, kNumConstraints> ub_red_ = {};
  std::array<qpOASES::real_t, kNumConstraints> lb_red_ = {};
  std::array<qpOASES::real_t, kNumVariables> g_red_ = {};

  std::array<qpOASES::real_t, kNumVariables> qsoln_red_ = {};
};

}  // namespace spotng::mpc

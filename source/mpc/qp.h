#pragma once
#include <qpOASES.hpp>
#include <vector>

#include "sdquadx/types.h"

namespace sdquadx::mpc {
struct QpData {
  QpData(int horizon_len);
  void Zero();
  std::vector<fpt_t> qsoln_;

  std::vector<fpt_t> A_qp_;
  std::vector<fpt_t> B_qp_;
  std::vector<fpt_t> S_;
  std::vector<fpt_t> eye_12h_;
  std::vector<fpt_t> x_0_;
  std::vector<fpt_t> X_d_;

  std::vector<fpt_t> qH_;
  std::vector<fpt_t> qA_;
  std::vector<fpt_t> qub_;
  std::vector<fpt_t> qlb_;
  std::vector<fpt_t> qg_;
};

class QpSolver {
 public:
  QpSolver(int horizon_len);
  bool Solve(QpData &data);

 private:
  void Reset();
  int const horizon_len_;

  std::vector<char> var_elim_;
  std::vector<char> con_elim_;

  std::vector<int> var_ind_;
  std::vector<int> con_ind_;

  std::vector<qpOASES::real_t> H_red_;
  std::vector<qpOASES::real_t> A_red_;
  std::vector<qpOASES::real_t> ub_red_;
  std::vector<qpOASES::real_t> lb_red_;
  std::vector<qpOASES::real_t> g_red_;

  std::vector<qpOASES::real_t> qsoln_red_;
};

}  // namespace sdquadx::mpc

#include "mpc/qp.h"

#include <algorithm>

#include "utils/eigen.h"

namespace sdquadx::mpc {

namespace {
bool near_zero(fpt_t a) { return (a < .01 && a > -.01); }

bool near_one(fpt_t a) { return near_zero(a - 1); }
}  // namespace

using QPMatrixX = Eigen::Matrix<fpt_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

QpData::QpData(int horizon_len)
    : qsoln_(12 * horizon_len),

      A_qp_((13 * horizon_len) * 13),
      B_qp_((13 * horizon_len) * (12 * horizon_len)),
      S_((13 * horizon_len) * (13 * horizon_len)),
      eye_12h_((12 * horizon_len) * (12 * horizon_len)),
      x_0_(13),
      X_d_((13 * horizon_len)),

      qH_(12 * horizon_len * 12 * horizon_len),
      qA_(20 * horizon_len * 12 * horizon_len),
      qub_(20 * horizon_len),
      qlb_(20 * horizon_len),
      qg_(12 * horizon_len) {
  Eigen::Map<MatrixX> eye_12h(eye_12h_.data(), 12 * horizon_len, 12 * horizon_len);
  eye_12h.setIdentity();
  Zero();
}

void QpData::Zero() {
  std::fill(qsoln_.begin(), qsoln_.end(), 0.);

  std::fill(A_qp_.begin(), A_qp_.end(), 0.);
  std::fill(B_qp_.begin(), B_qp_.end(), 0.);
  std::fill(S_.begin(), S_.end(), 0.);
  std::fill(x_0_.begin(), x_0_.end(), 0.);
  std::fill(X_d_.begin(), X_d_.end(), 0.);

  std::fill(qub_.begin(), qub_.end(), 0.);
  std::fill(qlb_.begin(), qlb_.end(), 0.);
  std::fill(qA_.begin(), qA_.end(), 0.);
  std::fill(qH_.begin(), qH_.end(), 0.);
  std::fill(qg_.begin(), qg_.end(), 0.);
}

QpSolver::QpSolver(int horizon_len)
    : horizon_len_(horizon_len),
      var_elim_(12 * horizon_len),
      con_elim_(20 * horizon_len),

      var_ind_(12 * horizon_len),
      con_ind_(20 * horizon_len),

      H_red_(12 * horizon_len * 12 * horizon_len),
      A_red_(20 * horizon_len * 12 * horizon_len),
      ub_red_(20 * horizon_len),
      lb_red_(20 * horizon_len),
      g_red_(12 * horizon_len),

      qsoln_red_(12 * horizon_len) {}

void QpSolver::Reset() {
  std::fill(var_elim_.begin(), var_elim_.end(), 0.);
  std::fill(con_elim_.begin(), con_elim_.end(), 0.);

  std::fill(H_red_.begin(), H_red_.end(), 0.);
  std::fill(A_red_.begin(), A_red_.end(), 0.);
  std::fill(ub_red_.begin(), ub_red_.end(), 0.);
  std::fill(lb_red_.begin(), lb_red_.end(), 0.);
  std::fill(g_red_.begin(), g_red_.end(), 0.);
  std::fill(qsoln_red_.begin(), qsoln_red_.end(), 0.);
  std::fill(var_ind_.begin(), var_ind_.end(), 0.);
  std::fill(con_ind_.begin(), con_ind_.end(), 0.);
}

bool QpSolver::Solve(QpData &data) {
  int nWSR = 100;
  auto const num_constraints = 20 * horizon_len_;
  auto const num_variables = 12 * horizon_len_;
  int new_cons = num_constraints;
  int new_vars = num_variables;

  Eigen::Map<MatrixX> qH(data.qH_.data(), num_variables, num_variables);
  Eigen::Map<MatrixX> qA(data.qA_.data(), num_constraints, num_variables);

  for (int i = 0; i < num_constraints; i++) {
    if (!(near_zero(data.qlb_[i]) && near_zero(data.qub_[i]))) continue;

    // ub=0 swing
    auto c_row = qA.row(i);

    for (int j = 0; j < num_variables; j++) {
      if (near_one(c_row[j])) {
        new_vars -= 3;
        new_cons -= 5;
        int cs = (j * 5) / 3 - 3;
        var_elim_[j - 2] = 1;
        var_elim_[j - 1] = 1;
        var_elim_[j] = 1;
        con_elim_[cs] = 1;
        con_elim_[cs + 1] = 1;
        con_elim_[cs + 2] = 1;
        con_elim_[cs + 3] = 1;
        con_elim_[cs + 4] = 1;
      }
    }
  }

  int vc = 0;

  for (int i = 0; i < num_variables; i++) {
    if (!var_elim_[i]) {
      if (vc >= new_vars) {
        throw("BAD ERROR 1\n");
      }
      var_ind_[vc] = i;
      vc++;
    }
  }
  vc = 0;
  for (int i = 0; i < num_constraints; i++) {
    if (!con_elim_[i]) {
      if (vc >= new_cons) {
        throw("BAD ERROR 1\n");
      }
      con_ind_[vc] = i;
      vc++;
    }
  }

  for (int i = 0; i < new_vars; i++) {
    int olda = var_ind_[i];
    g_red_[i] = data.qg_[olda];
    for (int j = 0; j < new_vars; j++) {
      int oldb = var_ind_[j];
      H_red_[i * new_vars + j] = qH(olda, oldb);
    }
  }

  for (int con = 0; con < new_cons; con++) {
    for (int st = 0; st < new_vars; st++) {
      auto cval = qA(con_ind_[con], var_ind_[st]);
      A_red_[con * new_vars + st] = cval;
    }
  }

  for (int i = 0; i < new_cons; i++) {
    int old = con_ind_[i];
    ub_red_[i] = data.qub_[old];
    lb_red_[i] = data.qlb_[old];
  }

  qpOASES::QProblem problem_red(new_vars, new_cons);
  qpOASES::Options op;
  op.setToMPC();
  op.printLevel = qpOASES::PL_NONE;
  problem_red.setOptions(op);
  // int_t nWSR = 50000;

  problem_red.init(H_red_.data(), g_red_.data(), A_red_.data(), nullptr, nullptr, lb_red_.data(), ub_red_.data(), nWSR);

  int rval2 = problem_red.getPrimalSolution(qsoln_red_.data());
  if (rval2 != qpOASES::SUCCESSFUL_RETURN) printf("failed to solve!\n");

  vc = 0;
  for (int i = 0; i < num_variables; i++) {
    if (var_elim_[i]) {
      data.qsoln_[i] = 0.0;
    } else {
      data.qsoln_[i] = qsoln_red_[vc];
      vc++;
    }
  }
  return true;
}

}  // namespace sdquadx::mpc

#include "mpc/qp.h"

#include "utils/eigen.h"

namespace spotng::mpc {

namespace {
bool near_zero(fpt_t a) { return (a < .01 && a > -.01); }

bool near_one(fpt_t a) { return near_zero(a - 1); }
}  // namespace

QpData::QpData() {
  Eigen::Map<MatrixX> _eye_12h(eye_12h.data(), 12 * kPredLength, 12 * kPredLength);
  _eye_12h.setIdentity();
}

void QpData::Zero() {
  qsoln.fill(0.);

  A_qp.fill(0.);
  B_qp.fill(0.);
  S.fill(0.);
  x_0.fill(0.);
  X_d.fill(0.);

  qub.fill(0.);
  qlb.fill(0.);
  qA.fill(0.);
  qH.fill(0.);
  qg.fill(0.);
}

void QpSolver::Reset() {
  var_elim_.fill(0.);
  con_elim_.fill(0.);

  H_red_.fill(0.);
  A_red_.fill(0.);
  ub_red_.fill(0.);
  lb_red_.fill(0.);
  g_red_.fill(0.);
  qsoln_red_.fill(0.);
  var_ind_.fill(0.);
  con_ind_.fill(0.);
}

bool QpSolver::Solve(QpData &data) {
  Reset();

  int nWSR = 100;
  int new_cons = kNumConstraints;
  int new_vars = kNumVariables;

  Eigen::Map<Eigen::Matrix<fpt_t, kNumVariables, kNumVariables>> qH(data.qH.data());
  Eigen::Map<Eigen::Matrix<fpt_t, kNumConstraints, kNumVariables>> qA(data.qA.data());

  for (int i = 0; i < kNumConstraints; i++) {
    if (!(near_zero(data.qlb[i]) && near_zero(data.qub[i]))) continue;

    // ub=0 swing
    auto c_row = qA.row(i);

    for (int j = 0; j < kNumVariables; j++) {
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

  for (int i = 0; i < kNumVariables; i++) {
    if (!var_elim_[i]) {
      if (vc >= new_vars) {
        throw("BAD ERROR 1\n");
      }
      var_ind_[vc] = i;
      vc++;
    }
  }
  vc = 0;
  for (int i = 0; i < kNumConstraints; i++) {
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
    g_red_[i] = data.qg[olda];
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
    ub_red_[i] = data.qub[old];
    lb_red_[i] = data.qlb[old];
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
  for (int i = 0; i < kNumVariables; i++) {
    if (var_elim_[i]) {
      data.qsoln[i] = 0.0;
    } else {
      data.qsoln[i] = qsoln_red_[vc];
      vc++;
    }
  }
  return true;
}

}  // namespace spotng::mpc

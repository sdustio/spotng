#include "mpc/cmpc.h"

#include <cmath>

#include "dynamics/rotation.h"
#include "estimate/contact.h"
#include "math/algebra.h"
#include "skd/od_gait.h"
#include "unsupported/Eigen/MatrixFunctions"

namespace sdquadx::mpc {

CMpc::CMpc(Options::ConstSharedPtr const &opts)
    : opts_(opts),
      dt_(opts->ctrl_sec * opts->ctrl.mpc_iters),
      qp_data_(opts->ctrl.mpc_horizon_len),
      qp_solver_(opts->ctrl.mpc_horizon_len) {}

bool CMpc::RunOnce(wbc::InData &wbcdata, estimate::State const &estdata, std::vector<int> const &st_states) {
  auto const horizon_len = opts_->ctrl.mpc_horizon_len;
  auto const num_constraints = 20 * horizon_len;
  auto const num_variables = 12 * horizon_len;

  qp_data_.Zero();

  Eigen::Map<MatrixX> A_qp(qp_data_.A_qp_.data(), 13 * horizon_len, 13);
  Eigen::Map<MatrixX> B_qp(qp_data_.B_qp_.data(), 13 * horizon_len, 12 * horizon_len);
  Eigen::Map<MatrixX> S(qp_data_.S_.data(), 13 * horizon_len, 13 * horizon_len);
  Eigen::Map<MatrixX> eye_12h(qp_data_.eye_12h_.data(), 12 * horizon_len, 12 * horizon_len);
  Eigen::Map<VectorX> x_0(qp_data_.x_0_.data(), 13, 1);
  Eigen::Map<VectorX> X_d(qp_data_.X_d_.data(), 13 * horizon_len, 1);
  Eigen::Map<MatrixX> qH(qp_data_.qH_.data(), num_variables, num_variables);
  Eigen::Map<MatrixX> qA(qp_data_.qA_.data(), num_constraints, num_variables);
  Eigen::Map<VectorX> qub(qp_data_.qub_.data(), num_constraints, 1);
  Eigen::Map<VectorX> qlb(qp_data_.qlb_.data(), num_constraints, 1);
  Eigen::Map<VectorX> qg(qp_data_.qg_.data(), num_variables, 1);

  X_d[0] = wbcdata.body_rpy_des[0];
  X_d[1] = wbcdata.body_rpy_des[1];
  X_d[2] = wbcdata.body_rpy_des[2] + dt_ * wbcdata.body_avel_des[2];
  X_d[3] = wbcdata.body_pos_des[0] + dt_ * wbcdata.body_lvel_des[0];
  X_d[4] = wbcdata.body_pos_des[1] + dt_ * wbcdata.body_lvel_des[1];
  X_d[5] = wbcdata.body_pos_des[2];
  X_d[6] = wbcdata.body_avel_des[0];  // 0
  X_d[7] = wbcdata.body_avel_des[1];  // 0
  X_d[8] = wbcdata.body_avel_des[2];  // 因为 wbcdata.body_avel_des 采用 avel_des ~= avel_des_robot
  X_d[9] = wbcdata.body_lvel_des[0];
  X_d[10] = wbcdata.body_lvel_des[1];
  X_d[11] = wbcdata.body_lvel_des[2];
  X_d[12] = -opts_->gravity;  // or 0.

  for (int i = 1; i < horizon_len; i++) {
    for (int j = 0; j < 13; j++) X_d[13 * i + j] = X_d[13 * (i - 1) + j];
    X_d[13 * i + 2] += dt_ * wbcdata.body_avel_des[2];
    X_d[13 * i + 3] += dt_ * wbcdata.body_lvel_des[0];
    X_d[13 * i + 4] += dt_ * wbcdata.body_lvel_des[1];
  }

  auto const pos = ToConstEigenTp(estdata.pos);  // p
  auto const rpy = ToConstEigenTp(estdata.rpy);
  fpt_t alpha = 4e-5;

  auto vx = estdata.lvel[0];
  if (vx > 0.3 || vx < -0.3) {
    x_comp_integral_ += opts_->ctrl.mpc_x_drag * (pos[2] - wbcdata.body_pos_des[2]) * dt_ / vx;
  }

  auto yc = std::cos(rpy[2]);
  auto ys = std::sin(rpy[2]);
  Matrix3 Rotz;
  Rotz << yc, -ys, 0, ys, yc, 0, 0, 0, 1;
  Matrix3 Iinv = (Rotz * ToConstEigenTp(opts_->model.inertia_total) * Rotz.transpose()).inverse();

  x_0 << rpy(2), rpy(1), rpy(0), pos, ToConstEigenTp(estdata.avel), ToConstEigenTp(estdata.lvel), -opts_->gravity;

  // continuous time state space matrices.
  MatrixX A_ct = Eigen::Matrix<fpt_t, 13, 13>::Zero();
  MatrixX B_ct_r = Eigen::Matrix<fpt_t, 13, 12>::Zero();

  A_ct(3, 9) = 1.;
  A_ct(4, 10) = 1.;
  A_ct(5, 11) = 1.;
  A_ct(11, 9) = x_comp_integral_;
  A_ct(11, 12) = 1.;
  A_ct.block<3, 3>(0, 6) = Rotz.transpose();

  for (int i = 0; i < 4; i++) {
    Vector3 ri = ToConstEigenTp(estdata.foot_pos[i]) - pos;
    Matrix3 rotm;
    math::VecToSkewMat(rotm, ri);
    B_ct_r.block<3, 3>(6, i * 3) = Iinv * rotm;
    B_ct_r.block<3, 3>(9, i * 3) = Matrix3::Identity() / opts_->model.mass_total;
  }

  // QP matrices
  MatrixX Adt;  // shape: 13 x 13
  MatrixX Bdt;  // shape: 13 x 12
  MatrixX ABc = Eigen::Matrix<fpt_t, 25, 25>::Zero();
  ABc.block<13, 13>(0, 0) = A_ct;
  ABc.block<13, 12>(0, 13) = B_ct_r;
  ABc = dt_ * ABc;
  MatrixX expmm = ABc.exp();  // shape: 25 x 25
  Adt = expmm.block<13, 13>(0, 0);
  Bdt = expmm.block<13, 12>(0, 13);

  MatrixX powerMats[20];  // 20 是预留长度 shape: 13 x 13
  powerMats[0].setIdentity(13, 13);
  for (int i = 1; i < horizon_len + 1; i++) {
    powerMats[i] = Adt * powerMats[i - 1];
  }

  for (int r = 0; r < horizon_len; r++) {
    A_qp.block<13, 13>(13 * r, 0) = powerMats[r + 1];  // Adt.pow(r+1);
    for (int c = 0; c < horizon_len; c++) {
      if (r >= c) {
        int a_num = r - c;
        B_qp.block<13, 12>(13 * r, 12 * c) = powerMats[a_num] /*Adt.pow(a_num)*/ * Bdt;
      }
    }
  }

  Eigen::Map<Eigen::Matrix<fpt_t, 13, 1> const> weights(opts_->ctrl.mpc_weights.data());
  S.diagonal() = weights.replicate(horizon_len, 1);

  int k = 0;
  for (int i = 0; i < horizon_len; i++) {
    for (int j = 0; j < consts::model::kNumLeg; j++) {
      qub(5 * k + 0) = consts::math::kBigNum;
      qub(5 * k + 1) = consts::math::kBigNum;
      qub(5 * k + 2) = consts::math::kBigNum;
      qub(5 * k + 3) = consts::math::kBigNum;
      qub(5 * k + 4) = st_states[i * 4 + j] * opts_->rfmax;
      k++;
    }
  }

  fpt_t rep_mu = 1. / (opts_->rfmu + 1.e-12);
  Eigen::Matrix<fpt_t, 5, 3> f_block;
  f_block << rep_mu, 0, 1., -rep_mu, 0, 1., 0, rep_mu, 1., 0, -rep_mu, 1., 0, 0, 1.;
  for (int i = 0; i < horizon_len * 4; i++) {
    qA.block<5, 3>(i * 5, i * 3) = f_block;
  }
  qH = 2 * (B_qp.transpose() * S * B_qp + alpha * eye_12h);
  qg = 2 * B_qp.transpose() * S * (A_qp * x_0 - X_d);

  qp_solver_.Solve(qp_data_);
  for (int leg = 0; leg < consts::model::kNumLeg; leg++) {
    for (int axis = 0; axis < 3; axis++) wbcdata.Fr_des[leg][axis] = qp_data_.qsoln_[leg * 3 + axis];
  }
  return true;
}

}  // namespace sdquadx::mpc

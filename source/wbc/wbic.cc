#include "wbc/wbic.h"

#include <eiquadprog/eiquadprog-fast.hpp>

#include "math/algebra.h"

namespace sdrobot::wbc {
using Matrix18 =
    Eigen::Matrix<fpt_t, consts::model::kDimConfig, consts::model::kDimConfig>;
using Sv_t = Eigen::Matrix<fpt_t, 6, consts::model::kDimConfig>;
using Vector12 = Eigen::Matrix<fpt_t, consts::model::kNumActJoint, 1>;

Wbic::Wbic(double weight) {
  W_floating_.fill(weight);
  W_rf_.fill(1.);

  Eigen::Map<Sv_t> Sv(Sv_.data());
  Sv.block<6, 6>(0, 0).setIdentity();
}

bool Wbic::UpdateSetting(model::MassMatTp const &A,
                         model::MassMatTp const &Ainv,
                         model::GeneralFTp const &cori,
                         model::GeneralFTp const &grav) {
  A_ = A;
  Ainv_ = Ainv;
  cori_ = cori;
  grav_ = grav;
  b_updatesetting_ = true;
  return true;
}

bool Wbic::MakeTorque(
    SdVector12f &ret, std::vector<Task::ConstSharedPtr> const &task_list,
    std::vector<Contact::ConstSharedPtr> const &contact_list) {
  if (!b_updatesetting_) {
    printf("[Wanning] WBIC setting is not done\n");
  }

  Eigen::Map<Sv_t> Sv(Sv_.data());
  Eigen::Map<Matrix18> A(A_.data());
  Eigen::Map<Matrix18> Ainv(Ainv_.data());
  Eigen::Map<Vector18> cori(cori_.data());
  Eigen::Map<Vector18> grav(grav_.data());

  VectorX z;
  // Cost
  MatrixX G;
  VectorX g0;

  // Equality
  MatrixX CE;
  VectorX ce0;

  // Inequality
  MatrixX CI;
  VectorX ci0;

  MatrixX Uf;
  VectorX Uf_ieq_vec;

  MatrixX Jc;  //  , num_qdot_
  VectorX JcDotQdot;
  VectorX Fr_des;

  /*
   * resize G, g0, CE, ce0, CI, ci0
   * resize Uf, Uf_ieq_vec, Jc, JcDotQdot, Fr_des
   */
  _SetQPSize(G, g0, CE, ce0, CI, ci0, Uf, Uf_ieq_vec, Jc, JcDotQdot, Fr_des,
             contact_list);

  _SetCost(G);

  VectorX qddot_pre;
  MatrixX Npre;

  if (dim_rf_ > 0) {
    MatrixX JcBar;

    // Contact Setting
    _ContactBuilding(Uf, Uf_ieq_vec, Jc, JcDotQdot, Fr_des, contact_list);

    // Set inequality constraints
    _SetInEqualityConstraint(CI, ci0, Uf, Uf_ieq_vec, Fr_des);
    _WeightedInverse(JcBar, Jc, Ainv);
    qddot_pre = JcBar * (-JcDotQdot);
    Npre = Matrix18::Identity() - JcBar * Jc;
  } else {
    qddot_pre = VectorX::Zero(consts::model::kDimConfig);
    Npre = Matrix18::Identity();
  }

  // Task
  for (auto const &task : task_list) {
    Eigen::Map<Eigen::Matrix<fpt_t, 3, consts::model::kDimConfig> const> Jt(
        task->GetTaskJacobian().data());
    auto JtDotQdot = ToConstEigenTp(task->GetTaskJacobianDotQdot());
    auto xddot = ToConstEigenTp(task->GetCommand());

    MatrixX JtBar, JtPre;

    JtPre = Jt * Npre;
    _WeightedInverse(JtBar, JtPre, Ainv);

    qddot_pre += JtBar * (xddot - JtDotQdot - Jt * qddot_pre);
    Npre = Npre * (Matrix18::Identity() - JtBar * JtPre);
  }

  // Set equality constraints
  _SetEqualityConstraint(CE, ce0, Jc, Fr_des, qddot_pre);

  // Optimization
  auto n = g0.size();
  auto m = ce0.size();
  auto p = ci0.size();

  // use normal
  // VectorX activeSet(p);
  // int activeSetSize;
  // eiquadprog::solvers::solve_quadprog(2 * G,  2 * g0, CE.transpose(), ce0,
  // CI.transpose(), ci0, z, activeSet, activeSetSize);

  // use fast
  eiquadprog::solvers::EiquadprogFast qp_;
  qp_.reset(n, m, p);
  qp_.solve_quadprog(G, g0, CE, ce0, CI, ci0, z);

  // pretty_print(qddot_pre, std::cout, "qddot_cmd");
  for (int i(0); i < consts::model::kDimFloating; ++i) qddot_pre[i] += z[i];

  for (int i = 0; i < consts::model::kDimConfig; i++) {
    if (fabs(qddot_pre[i]) > 99999.) {
      qddot_pre = VectorX::Zero(consts::model::kDimConfig);
    }
  }

  _GetSolution(ret, qddot_pre, z, Fr_des, Jc);
  return true;
}

bool Wbic::_WeightedInverse(MatrixX &ret, MatrixX const &J, MatrixX const &Winv,
                            fpt_t threshold) {
  MatrixX lambda(J * Winv * J.transpose());
  MatrixX lambda_inv(lambda.cols(), lambda.rows());
  math::PseudoInverse(lambda_inv, lambda, threshold);
  ret = Winv * J.transpose() * lambda_inv;
  return true;
}

bool Wbic::_ContactBuilding(
    MatrixX &Uf, VectorX &Uf_ieq_vec,
    MatrixX &Jc,  //  , num_qdot_
    VectorX &JcDotQdot, VectorX &Fr_des,
    std::vector<Contact::ConstSharedPtr> const &contact_list) {
  int dim_accumul_rf = 0, dim_accumul_uf = 0;

  for (size_t i = 0; i < contact_list.size(); i++) {
    Eigen::Map<Eigen::Matrix<fpt_t, 3, consts::model::kDimConfig> const> _Jc(
        contact_list[i]->GetContactJacobian().data());

    auto dim_new_rf = contact_list[i]->GetDim();
    auto dim_new_uf = contact_list[i]->GetDimRFConstraint();

    Jc.block(dim_accumul_rf, 0, dim_new_rf, consts::model::kDimConfig) = _Jc;
    JcDotQdot.segment(dim_accumul_rf, dim_new_rf) =
        ToConstEigenTp(contact_list[i]->GetJcDotQdot());

    Eigen::Map<Eigen::Matrix<fpt_t, 6, 3> const> _Uf(
        contact_list[i]->GetRFConstraintMtx().data());

    Uf.block(dim_accumul_uf, dim_accumul_rf, dim_new_uf, dim_new_rf) = _Uf;
    Uf_ieq_vec.segment(dim_accumul_uf, dim_new_uf) =
        ToConstEigenTp(contact_list[i]->GetRFConstraintVec());

    Fr_des.segment(dim_accumul_rf, dim_new_rf) =
        ToConstEigenTp(contact_list[i]->GetRFdes());
    dim_accumul_rf += dim_new_rf;
    dim_accumul_uf += dim_new_uf;
  }
  return true;
}

bool Wbic::_SetEqualityConstraint(MatrixX &CE, VectorX &ce0, MatrixX const &Jc,
                                  VectorX const &Fr_des,
                                  Vector18 const &qddot) {
  Eigen::Map<Sv_t> Sv(Sv_.data());
  Eigen::Map<Matrix18> A(A_.data());
  Eigen::Map<Vector18> cori(cori_.data());
  Eigen::Map<Vector18> grav(grav_.data());
  if (dim_rf_ > 0) {
    CE.block(0, 0, dim_eq_cstr_, consts::model::kDimFloating) =
        A.block(0, 0, dim_eq_cstr_, consts::model::kDimFloating);
    CE.block(0, consts::model::kDimFloating, dim_eq_cstr_, dim_rf_) =
        -Sv * Jc.transpose();
    ce0 = -Sv * (A * qddot + cori + grav - Jc.transpose() * Fr_des);
  } else {
    CE.block(0, 0, dim_eq_cstr_, consts::model::kDimFloating) =
        A.block(0, 0, dim_eq_cstr_, consts::model::kDimFloating);
    ce0 = -Sv * (A * qddot + cori + grav);
  }

  return true;
}

bool Wbic::_SetInEqualityConstraint(MatrixX &CI, VectorX &ci0,
                                    MatrixX const &Uf,
                                    VectorX const &Uf_ieq_vec,
                                    VectorX const &Fr_des) {
  CI.block(0, consts::model::kDimFloating, dim_Uf_, dim_rf_) = Uf;
  ci0 = Uf_ieq_vec - Uf * Fr_des;
  return true;
}

bool Wbic::_SetCost(MatrixX &G) {
  // Set Cost
  int idx_offset = 0;
  for (int i = 0; i < consts::model::kDimFloating; ++i) {
    G((i + idx_offset), (i + idx_offset)) = 0.5 * W_floating_[i];
  }
  idx_offset += consts::model::kDimFloating;
  for (int i = 0; i < dim_rf_; ++i) {
    G((i + idx_offset), (i + idx_offset)) = 0.5 * W_rf_[i];
  }
  return true;
}

bool Wbic::_GetSolution(SdVector12f &ret, Vector18 const &qddot,
                        VectorX const &z, VectorX const &Fr_des,
                        MatrixX const &Jc) {
  Eigen::Map<Matrix18> A(A_.data());
  Eigen::Map<Vector18> cori(cori_.data());
  Eigen::Map<Vector18> grav(grav_.data());

  Vector18 tot_tau;
  if (dim_rf_ > 0) {
    VectorX _Fr(dim_rf_);
    // get Reaction forces
    for (int i(0); i < dim_rf_; ++i)
      _Fr[i] = z[i + consts::model::kDimFloating] + Fr_des[i];
    tot_tau = A * qddot + cori + grav - Jc.transpose() * _Fr;
  } else {
    tot_tau = A * qddot + cori + grav;
  }

  Eigen::Map<Vector12> _ret(ret.data());
  _ret = tot_tau.tail(consts::model::kNumActJoint);

  for (int i = 0; i < 12; i++) {
    if (fabs(ret[i]) > 24) {
      printf(
          "!!!!!!!!danger!!!!! WBC torque num %d\tout the limitation: %f.2f ",
          i, ret[i]);
      ret[i] = ret[i] / fabs(ret[i]) * consts::interface::kMaxTorque;
      printf("  edit to %f.2f\n", ret[i]);
    }
  }
  return true;
}

bool Wbic::_SetQPSize(
    MatrixX &G, VectorX &g0, MatrixX &CE, VectorX &ce0, MatrixX &CI,
    VectorX &ci0, MatrixX &Uf, VectorX &Uf_ieq_vec, MatrixX &Jc,
    VectorX &JcDotQdot, VectorX &Fr_des,
    std::vector<Contact::ConstSharedPtr> const &contact_list) {
  // Dimension
  dim_rf_ = 0;
  dim_Uf_ = 0;  // Dimension of inequality constraint
  for (size_t i = 0; i < contact_list.size(); ++i) {
    dim_rf_ += contact_list[i]->GetDim();
    dim_Uf_ += contact_list[i]->GetDimRFConstraint();
  }

  dim_opt_ = consts::model::kDimFloating + dim_rf_;
  dim_eq_cstr_ = consts::model::kDimFloating;

  // Matrix Setting
  G = MatrixX::Zero(dim_opt_, dim_opt_);
  g0 = VectorX::Zero(dim_opt_);

  // Eigen Matrix Setting
  CE = MatrixX::Zero(dim_eq_cstr_, dim_opt_);
  ce0 = VectorX(dim_eq_cstr_);
  if (dim_rf_ > 0) {
    CI = MatrixX::Zero(dim_Uf_, dim_opt_);
    ci0 = VectorX::Zero(dim_Uf_);

    Jc = MatrixX(dim_rf_, consts::model::kDimConfig);
    JcDotQdot = VectorX(dim_rf_);
    Fr_des = VectorX(dim_rf_);

    Uf = MatrixX(dim_Uf_, dim_rf_);
    Uf.setZero();
    Uf_ieq_vec = VectorX(dim_Uf_);
  } else {
    CI = MatrixX::Zero(1, dim_opt_);
    ci0 = VectorX::Zero(1);
  }
  return true;
}

}  // namespace sdrobot::wbc

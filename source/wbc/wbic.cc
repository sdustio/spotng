#include "wbc/wbic.h"

#include "eiquadprog/eiquadprog-fast.hpp"
#include "math/algebra.h"
#include "spdlog/spdlog.h"
#include "utils/eigen.h"
#include "wbc/contact.h"
#include "wbc/task/body_ori.h"
#include "wbc/task/body_pos.h"
#include "wbc/task/foot_contact.h"
#include "wbc/task/foot_pos.h"

#ifdef DEBUG_MODE
#include "utils/debug.h"
#endif

namespace forax::wbc {

using Vector12 = Eigen::Matrix<fpt_t, consts::model::kNumJoint, 1>;
using Vector18 = Eigen::Matrix<fpt_t, consts::model::kDimConfig, 1>;
using Sv_t = Eigen::Matrix<fpt_t, 6, consts::model::kDimConfig>;

namespace params {
constexpr fpt_t const kSigmaThreshold = 0.0001;
}

Wbic::Wbic(Options::ConstSharedPtr const &opts, model::Quadruped::SharedPtr const &quad, double weight)
    : opts_(opts),
      mquad_(quad),
      body_pos_task_(std::make_shared<TaskBodyPos>(opts->ctrl.kp_bodypos, opts->ctrl.kd_bodypos)),
      body_ori_task_(std::make_shared<TaskBodyOri>(opts->ctrl.kp_bodyori, opts->ctrl.kd_bodyori)),
      foot_task_({
          std::make_shared<TaskFootPos>(opts->ctrl.kp_foot, opts->ctrl.kd_foot, mquad_, consts::legidx::fr),
          std::make_shared<TaskFootPos>(opts->ctrl.kp_foot, opts->ctrl.kd_foot, mquad_, consts::legidx::fl),
          std::make_shared<TaskFootPos>(opts->ctrl.kp_foot, opts->ctrl.kd_foot, mquad_, consts::legidx::hr),
          std::make_shared<TaskFootPos>(opts->ctrl.kp_foot, opts->ctrl.kd_foot, mquad_, consts::legidx::hl),
      }),
      foot_contact_({
          std::make_shared<TaskFootContact>(mquad_, consts::legidx::fr),
          std::make_shared<TaskFootContact>(mquad_, consts::legidx::fl),
          std::make_shared<TaskFootContact>(mquad_, consts::legidx::hr),
          std::make_shared<TaskFootContact>(mquad_, consts::legidx::hl),
      }),
      weight_q_(weight) {}

bool Wbic::RunOnce(interface::LegCmds &cmds, InData const &wbcdata, estimate::State const &estdata) {
#ifdef DEBUG_MODE
  spdlog::debug("!!![WBC InData]");
  DebugVector("Body Pos Des", wbcdata.body_pos_des);
  DebugVector("Body Lvel Des", wbcdata.body_lvel_des);
  DebugVector("Body Acc Des", wbcdata.body_acc_des);
  DebugVector("Body RPY Des", wbcdata.body_rpy_des);
  DebugVector("Body Avel Des", wbcdata.body_avel_des);
  DebugVector("Foot Contact State", wbcdata.contact_state);
  for (int i = 0; i < consts::model::kNumLeg; i++) {
    DebugVector("Pos Des of foot " + std::to_string(i), wbcdata.foot_pos_des[i]);
    DebugVector("Lvel Des of foot " + std::to_string(i), wbcdata.foot_lvel_des[i]);
    DebugVector("Acc Des of foot " + std::to_string(i), wbcdata.foot_acc_des[i]);
    DebugVector("Fr Des of foot " + std::to_string(i), wbcdata.Fr_des[i]);
  }
#endif

  mquad_->UpdateDynamics(estdata);

  // Task & Contact Update
  _ContactTaskUpdate(wbcdata, estdata);

  // WBC Computation
  _ComputeWBC();
  return _UpdateLegCMD(cmds);
}

bool Wbic::_ComputeWBC() {
  // QP ret
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

  // ====
  Vector18 delta_q = Vector18::Zero();
  Vector18 qd = Vector18::Zero();
  Vector18 qdd = Vector18::Zero();

  VectorX Fr;
  MatrixX Jc;
  VectorX Jcdqd;

  MatrixX Ca;
  VectorX ca_l;

  MatrixX Nc;

  int const rfi = 3;
  int const Ufi = 6;
  int const dim_rf = contact_list_.size() * rfi;
  int const dim_Uf = contact_list_.size() * Ufi;
  int const dim_opt = consts::model::kDimFloating + dim_rf;

  if (dim_rf > 0) {
    Jc.resize(dim_rf, consts::model::kDimConfig);
    Jcdqd.resize(dim_rf);

    Fr.resize(dim_rf);

    Ca = MatrixX::Zero(dim_Uf, dim_rf);
    ca_l = VectorX::Zero(dim_Uf);

    auto num_rfrows = 0;
    auto num_Ufrows = 0;
    auto rfmu = opts_->rfmu;
    auto rfmax = opts_->rfmax;

    for (auto const &ci : contact_list_) {
      Eigen::Map<Eigen::Matrix<fpt_t, 3, consts::model::kDimConfig> const> Jc_i(ci->GetTaskJacobian().data());
      Jc.block(num_rfrows, 0, rfi, consts::model::kDimConfig) = Jc_i;
      Jcdqd.segment(num_rfrows, rfi) = ToConstEigenTp(ci->GetTaskJacobianDotQdot());

      Fr.segment(num_rfrows, rfi) = ToConstEigenTp(ci->GetXddCmd());

      BuildContactConstraintMat(Ca.block(num_Ufrows, num_rfrows, Ufi, rfi), rfmu);
      BuildContactConstraintUpperBoundVec(ca_l.segment(num_Ufrows, Ufi), rfmax);

      num_rfrows += rfi;
      num_Ufrows += Ufi;
    }

    MatrixX Jc_pinv(Jc.cols(), Jc.rows());
    math::PseudoInverse(Jc_pinv, Jc, params::kSigmaThreshold);
    qdd = Jc_pinv * (-Jcdqd);

    Nc = MatrixX::Identity(Jc.cols(), Jc.cols()) - Jc_pinv * Jc;
  } else {
    qdd = Vector18::Zero();
    Nc = MatrixX::Identity(consts::model::kDimConfig, consts::model::kDimConfig);
  }

  MatrixX Jt, N_nx;
  for (auto const &task : task_list_) {
    Eigen::Map<Eigen::Matrix<fpt_t, 3, consts::model::kDimConfig> const> Jt_i(task->GetTaskJacobian().data());
    Jt = Jt_i * Nc;
    MatrixX Jt_pinv(Jt.cols(), Jt.rows());
    math::PseudoInverse(Jt_pinv, Jt, params::kSigmaThreshold);

    delta_q = delta_q + Jt_pinv * (ToConstEigenTp(task->GetXError()) - Jt_i * delta_q);
    qd = qd + Jt_pinv * (ToConstEigenTp(task->GetXdDes()) - Jt_i * qd);
    qdd = qdd +
          Jt_pinv * (ToConstEigenTp(task->GetXddCmd()) - ToConstEigenTp(task->GetTaskJacobianDotQdot()) - Jt_i * qdd);

    // _BuildProjectionMatrix(N_nx, Jt);
    Nc = Nc * (MatrixX::Identity(consts::model::kDimConfig, consts::model::kDimConfig) - Jt_pinv * Jt);
  }

  auto const &dyndata = mquad_->GetDynamicsData();

  // Build G, g0
  G = MatrixX::Zero(dim_opt, dim_opt);
  g0 = VectorX::Zero(dim_opt);
  G.block(0, 0, consts::model::kDimFloating, consts::model::kDimFloating) =
      MatrixX::Identity(consts::model::kDimFloating, consts::model::kDimFloating) * weight_q_ * 0.5;
  G.block(consts::model::kDimFloating, consts::model::kDimFloating, dim_rf, dim_rf) =
      MatrixX::Identity(dim_rf, dim_rf) * weight_f_ * 0.5;

  // Build CE, ce0
  CE = MatrixX::Zero(consts::model::kDimFloating, dim_opt);
  ce0 = VectorX(consts::model::kDimFloating);

  MatrixX Sf = MatrixX::Zero(6, consts::model::kDimConfig);
  Sf.block(0, 0, consts::model::kDimFloating, consts::model::kDimFloating).setIdentity();

  Eigen::Map<Eigen::Matrix<fpt_t, consts::model::kDimConfig, consts::model::kDimConfig> const> Mm(dyndata.M.data());
  Eigen::Map<Vector18 const> Cc(dyndata.Cc.data());
  Eigen::Map<Vector18 const> Cg(dyndata.Cg.data());

  CE.block(0, 0, consts::model::kDimFloating, consts::model::kDimFloating) =
      Mm.block(0, 0, consts::model::kDimFloating, consts::model::kDimFloating);
  if (dim_rf > 0) {
    CE.block(0, consts::model::kDimFloating, consts::model::kDimFloating, dim_rf) = -Sf * Jc.transpose();
    ce0 = Sf * (Mm * qdd + Cc + Cg - Jc.transpose() * Fr);
  } else {
    ce0 = Sf * (Mm * qdd + Cc + Cg);
  }

  // Build CI, ci0
  if (dim_rf > 0) {
    CI = MatrixX::Zero(dim_Uf, dim_opt);
    ci0 = VectorX::Zero(dim_Uf);
    CI.block(0, consts::model::kDimFloating, dim_Uf, dim_rf) = Ca;
    ci0.segment(0, dim_Uf) = Ca * Fr - ca_l;
  } else {
    CI = MatrixX::Zero(1, dim_opt);
    ci0 = VectorX::Zero(1);
  }

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

  for (int i = 0; i < consts::model::kDimFloating; i++) qdd[i] += z[i];
  for (int i = 0; i < consts::model::kDimConfig; i++) {
    if (fabs(qdd[i]) > 99999.) qdd = VectorX::Zero(consts::model::kDimConfig);
  }
  Vector18 tot_tau;
  if (dim_rf > 0) {
    // get Reaction forces
    for (int i = 0; i < dim_rf; ++i) Fr[i] = Fr[i] + z[i + consts::model::kDimFloating];
    tot_tau = Mm * qdd + Cc + Cg - Jc.transpose() * Fr;
  } else {
    tot_tau = Mm * qdd + Cc + Cg;
  }

  // For Leg Cmd
  for (int i = 0; i < consts::model::kNumJoint; ++i) {
    q_cmd_[i] = dyndata.q[i] + delta_q[i + 6];
    qd_cmd_[i] = qd[i + 6];
    tau_ff_[i] = tot_tau[i + 6];
  }

  return true;
}

bool Wbic::_UpdateLegCMD(interface::LegCmds &cmds) {
  for (int leg(0); leg < consts::model::kNumLeg; ++leg) {
    for (int jidx(0); jidx < consts::model::kNumLegJoint; ++jidx) {
      cmds[leg].tau[jidx] = tau_ff_[consts::model::kNumLegJoint * leg + jidx];
      cmds[leg].q_des[jidx] = q_cmd_[consts::model::kNumLegJoint * leg + jidx];
      cmds[leg].qd_des[jidx] = qd_cmd_[consts::model::kNumLegJoint * leg + jidx];
      cmds[leg].kp = opts_->ctrl.kp_joint;
      cmds[leg].kd = opts_->ctrl.kd_joint;
    }
  }
  return true;
}

bool Wbic::_ContactTaskUpdate(InData const &wbcdata, estimate::State const &estdata) {
  // Wash out the previous setup
  _CleanUp();

  body_ori_task_->UpdateTask(estdata, wbcdata.body_rpy_des, wbcdata.body_avel_des, SdVector3f{});
  body_pos_task_->UpdateTask(estdata, wbcdata.body_pos_des, wbcdata.body_lvel_des, wbcdata.body_acc_des);

  task_list_.push_back(body_ori_task_);
  task_list_.push_back(body_pos_task_);

  for (int leg(0); leg < consts::model::kNumLeg; ++leg) {
    if (wbcdata.contact_state[leg] > consts::math::kZeroEpsilon) {  // Contact
      foot_contact_[leg]->UpdateTask(estdata, SdVector3f{}, SdVector3f{}, wbcdata.Fr_des[leg]);
      contact_list_.push_back(foot_contact_[leg]);
    } else {  // No Contact (swing)
      foot_task_[leg]->UpdateTask(estdata, wbcdata.foot_pos_des[leg], wbcdata.foot_lvel_des[leg],
                                  wbcdata.foot_acc_des[leg]);
      // zero_vec3);
      task_list_.push_back(foot_task_[leg]);
    }
  }
  return true;
}

bool Wbic::_CleanUp() {
  contact_list_.clear();
  task_list_.clear();
  return true;
}

}  // namespace forax::wbc

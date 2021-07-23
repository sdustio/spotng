#include "sdrobot/controllers/wbc.h"
#include "sdrobot/dynamics/math.h"

namespace sdrobot::ctrl::wbc
{

  Wbic::Wbic(int num_qdot, double weight) : num_act_joint_(num_qdot - 6), num_qdot_(num_qdot),
                                               W_floating_(VectorX::Constant(6, weight)),
                                               W_rf_(VectorX::Constant(12, 1.))

  {
    Sa_ = MatrixX::Zero(num_act_joint_, num_qdot_);
    Sv_ = MatrixX::Zero(6, num_qdot_);

    Sa_.block(0, 6, num_act_joint_, num_act_joint_).setIdentity();
    Sv_.block<6, 6>(0, 0).setIdentity();

    eye_ = MatrixX::Identity(num_qdot_, num_qdot_);
  }

  void Wbic::UpdateSetting(const MatrixX &A, const MatrixX &Ainv,
                           const VectorX &cori, const VectorX &grav)
  {
    A_ = A;
    Ainv_ = Ainv;
    cori_ = cori;
    grav_ = grav;
    b_updatesetting_ = true;
  }

  void Wbic::MakeTorque(VectorX &cmd, const std::vector<TaskPtr> &task_list, const std::vector<ContactPtr> &contact_list)
  {
    if (!b_updatesetting_)
    {
      printf("[Wanning] WBIC setting is not done\n");
    }

    // resize G, g0, CE, ce0, CI, ci0
    _SetOptimizationSize(contact_list);
    _SetCost();

    VectorX qddot_pre;
    MatrixX JcBar;
    MatrixX Npre;

    if (dim_rf_ > 0)
    {
      // Contact Setting
      _ContactBuilding(contact_list);

      // Set inequality constraints
      _SetInEqualityConstraint();
      _WeightedInverse(Jc_, Ainv_, JcBar);
      qddot_pre = JcBar * (-JcDotQdot_);
      Npre = eye_ - JcBar * Jc_;
    }
    else
    {
      qddot_pre = VectorX::Zero(num_qdot_);
      Npre = eye_;
    }

    // Task
    TaskPtr task;
    MatrixX Jt, JtBar, JtPre;
    VectorX JtDotQdot, xddot;

    for (auto &task : task_list)
    {
      Jt = task->GetTaskJacobian();
      JtDotQdot = task->GetTaskJacobianDotQdot();
      xddot = task->GetCommand();

      JtPre = Jt * Npre;
      _WeightedInverse(JtPre, Ainv_, JtBar);

      qddot_pre += JtBar * (xddot - JtDotQdot - Jt * qddot_pre);
      Npre = Npre * (eye_ - JtBar * JtPre);
    }

    // Set equality constraints
    _SetEqualityConstraint(qddot_pre);

    // Optimization
    // Timer timer;
    _SolveQP();

    // pretty_print(qddot_pre, std::cout, "qddot_cmd");
    for (int i(0); i < dim_floating_; ++i)
      qddot_pre[i] += z_[i];

    for (int i = 0; i < robot::ModelAttrs::dim_config; i++)
    {
      if (fabs(qddot_pre[i]) > 99999.)
      {
        qddot_pre = VectorX::Zero(num_qdot_);
      }
    }

    _GetSolution(qddot_pre, cmd);
  }

  void Wbic::_WeightedInverse(const MatrixX &J, const MatrixX &Winv, MatrixX &Jinv, double threshold)
  {
    MatrixX lambda(J * Winv * J.transpose());
    MatrixX lambda_inv;
    dynamics::PseudoInverse(lambda, threshold, lambda_inv);
    Jinv = Winv * J.transpose() * lambda_inv;
  }

  void Wbic::_ContactBuilding(const std::vector<ContactPtr> &contact_list)
  {
    MatrixX Uf;
    VectorX Uf_ieq_vec;
    // Initial
    MatrixX Jc;
    VectorX JcDotQdot;
    int dim_accumul_rf, dim_accumul_uf;

    Jc = contact_list[0]->GetContactJacobian();
    JcDotQdot = contact_list[0]->GetJcDotQdot();
    Uf = contact_list[0]->GetRFConstraintMtx();
    Uf_ieq_vec = contact_list[0]->GetRFConstraintVec();

    dim_accumul_rf = contact_list[0]->GetDim();
    dim_accumul_uf = contact_list[0]->GetDimRFConstraint();

    Jc_.block(0, 0, dim_accumul_rf, num_qdot_) = Jc;
    JcDotQdot_.head(dim_accumul_rf) = JcDotQdot;
    Uf_.block(0, 0, dim_accumul_uf, dim_accumul_rf) = Uf;
    Uf_ieq_vec_.head(dim_accumul_uf) = Uf_ieq_vec;
    Fr_des_.head(dim_accumul_rf) = contact_list[0]->GetRFDesired();

    int dim_new_rf, dim_new_uf;
    int csize = contact_list.size();
    for (int i(1); i < csize; ++i)
    {
      Jc = contact_list[i]->GetContactJacobian();
      JcDotQdot = contact_list[i]->GetJcDotQdot();

      dim_new_rf = contact_list[i]->GetDim();
      dim_new_uf = contact_list[i]->GetDimRFConstraint();

      // Jc append
      Jc_.block(dim_accumul_rf, 0, dim_new_rf, num_qdot_) = Jc;

      // JcDotQdot append
      JcDotQdot_.segment(dim_accumul_rf, dim_new_rf) = JcDotQdot;

      // Uf
      Uf = contact_list[i]->GetRFConstraintMtx();
      Uf_.block(dim_accumul_uf, dim_accumul_rf, dim_new_uf, dim_new_rf) = Uf;

      // Uf inequality vector
      Uf_ieq_vec = contact_list[i]->GetRFConstraintVec();
      Uf_ieq_vec_.segment(dim_accumul_uf, dim_new_uf) = Uf_ieq_vec;

      // Fr desired
      Fr_des_.segment(dim_accumul_rf, dim_new_rf) =
          contact_list[i]->GetRFDesired();
      dim_accumul_rf += dim_new_rf;
      dim_accumul_uf += dim_new_uf;
    }
  }

  void Wbic::_SetEqualityConstraint(const VectorX &qddot)
  {

    if (dim_rf_ > 0)
    {
      CE_.block(0, 0, dim_eq_cstr_, dim_floating_) =
          A_.block(0, 0, dim_eq_cstr_, dim_floating_);
      CE_.block(0, dim_floating_, dim_eq_cstr_, dim_rf_) =
          -Sv_ * Jc_.transpose();
      ce0_ = -Sv_ * (A_ * qddot + cori_ + grav_ -
                     Jc_.transpose() * Fr_des_);
    }
    else
    {
      CE_.block(0, 0, dim_eq_cstr_, dim_floating_) =
          A_.block(0, 0, dim_eq_cstr_, dim_floating_);
      ce0_ = -Sv_ * (A_ * qddot + cori_ + grav_);
    }
  }

  void Wbic::_SetInEqualityConstraint()
  {
    CI_.block(0, dim_floating_, dim_Uf_, dim_rf_) = Uf_;
    ci0_ = Uf_ieq_vec_ - Uf_ * Fr_des_;
  }

  void Wbic::_SetOptimizationSize(const std::vector<ContactPtr> &contact_list)
  {

    // Dimension
    dim_rf_ = 0;
    dim_Uf_ = 0; // Dimension of inequality constraint
    int csize = contact_list.size();
    for (int i(0); i < csize; ++i)
    {
      dim_rf_ += contact_list[i]->GetDim();
      dim_Uf_ += contact_list[i]->GetDimRFConstraint();
    }

    dim_opt_ = dim_floating_ + dim_rf_;
    dim_eq_cstr_ = dim_floating_;

    // Matrix Setting
    G_ = MatrixX::Zero(dim_opt_, dim_opt_);
    g0_ = VectorX::Zero(dim_opt_);

    // Eigen Matrix Setting
    CE_ = MatrixX::Zero(dim_eq_cstr_, dim_opt_);
    ce0_ = VectorX(dim_eq_cstr_);
    if (dim_rf_ > 0)
    {
      CI_ = MatrixX::Zero(dim_Uf_, dim_opt_);
      ci0_ = VectorX::Zero(dim_Uf_);

      Jc_ = MatrixX(dim_rf_, num_qdot_);
      JcDotQdot_ = VectorX(dim_rf_);
      Fr_des_ = VectorX(dim_rf_);

      Uf_ = MatrixX(dim_Uf_, dim_rf_);
      Uf_.setZero();
      Uf_ieq_vec_ = VectorX(dim_Uf_);
    }
    else
    {
      CI_ = MatrixX::Zero(1, dim_opt_);
      ci0_ = VectorX::Zero(1);
    }
  }

  void Wbic::_SetCost()
  {
    // Set Cost
    int idx_offset(0);
    for (int i(0); i < dim_floating_; ++i)
    {
      G_((i + idx_offset), (i + idx_offset)) = 0.5 * W_floating_[i];
    }
    idx_offset += dim_floating_;
    for (int i(0); i < dim_rf_; ++i)
    {
      G_((i + idx_offset), (i + idx_offset)) = 0.5 * W_rf_[i];
    }
  }

  void Wbic::_GetSolution(const VectorX &qddot, VectorX &cmd)
  {

    VectorX tot_tau;
    if (dim_rf_ > 0)
    {
      VectorX _Fr(dim_rf_);
      // get Reaction forces
      for (int i(0); i < dim_rf_; ++i)
        _Fr[i] = z_[i + dim_floating_] + Fr_des_[i];
      tot_tau =
          A_ * qddot + cori_ + grav_ - Jc_.transpose() * _Fr;
    }
    else
    {
      tot_tau = A_ * qddot + cori_ + grav_;
    }

    cmd = tot_tau.tail(num_act_joint_) * 1.0;

    for (int i = 0; i < 12; i++)
    {
      if (fabs(cmd[i]) > 24)
      {

        printf("!!!!!!!!danger!!!!! WBC torque num %d\tout the limitation: %f.2f ", i, cmd[i]);
        cmd[i] = cmd[i] / fabs(cmd[i]) * 24.0;
        printf("  edit to %f.2f\n", cmd[i]);
      }
    }
  }

  int Wbic::_SolveQP()
  {
    int n(z_.size());
    int m(ce0_.size());
    int p(ci0_.size());

    qp_.reset(n, m, p);

    // use fast
    MatrixX &_G = G_;
    VectorX &_g0 = g0_;
    MatrixX _CE = CE_.transpose();
    MatrixX _CI = CI_.transpose();
    VectorX &_ce0 = ce0_;
    VectorX &_ci0 = ci0_;
    eiquadprog::solvers::EiquadprogFast_status status = qp_.solve_quadprog(
        _G, _g0, _CE, _ce0, _CI, _ci0, z_);
    return status;
    // use normal
    // MatrixX _G = 2 * G_;
    // VectorX &_g0 = g0_;
    // MatrixX &_CE = CE_;
    // MatrixX &_CI = CI_;
    // VectorX &_ce0 = ce0_;
    // VectorX &_ci0 = ci0_;
    // VectorX activeSet(p);
    // int activeSetSize;
    // eiquadprog::solvers::solve_quadprog(_G, _g0, _CE, _ce0, _CI, _ci0, z_, activeSet, activeSetSize);
  }

}

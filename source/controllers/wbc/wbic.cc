#include "QuadProgpp/QuadProg++.hh"

#include "sdrobot/controllers/wbc.h"
#include "sdrobot/dynamics/math.h"

namespace sdrobot::ctrl::wbc
{

  double solve_quadprog(MatrixX &_G, VectorX &_g0,
                        const MatrixX &_CE, const VectorX &_ce0,
                        const MatrixX &_CI, const VectorX &_ci0,
                        VectorX &_x)
  {
    quadprogpp::Matrix<double> G, CE, CI;
    quadprogpp::Vector<double> g0, ce0, ci0, x;
    int n(_x.size());
    int m(_ce0.size());
    int p(_ci0.size());

    G.resize(n, n);
    for (int i = 0; i < n; ++i)
    {
      for (int j = 0; j < n; ++j)
      {
        G[i][j] = _G(i, j);
      }
    }

    g0.resize(n);
    for (int i = 0; i < n; ++i)
    {
      g0[i] = _g0[i];
    }

    CE.resize(n, m);
    for (int i = 0; i < n; ++i)
    {
      for (int j = 0; j < m; ++j)
      {
        CE[i][j] = _CE(j, i);
      }
    }

    ce0.resize(m);
    for (int i = 0; i < m; ++i)
    {
      ce0[i] = -_ce0[i];
    }

    CI.resize(n, p);
    for (int i = 0; i < n; ++i)
    {
      for (int j = 0; j < p; ++j)
      {
        CI[i][j] = _CI(j, i);
      }
    }

    ci0.resize(p);
    for (int i = 0; i < p; ++i)
    {
      ci0[i] = -_ci0[i];
    }

    x.resize(n);

    double ret(quadprogpp::solve_quadprog(G, g0, CE, ce0, CI, ci0, x));
    for (int i = 0; i < n; ++i)
    {
      _x[i] = x[i];
    }

    return ret;
  }

  Wbic::Wbic(size_t num_qdot, double weight) : num_act_joint_(num_qdot - 6), num_qdot_(num_qdot),
                                               _W_floating(VectorX::Constant(6, weight)),
                                               _W_rf(VectorX::Constant(12, 1.))

  {
    Sa_ = MatrixX::Zero(num_act_joint_, num_qdot_);
    Sv_ = MatrixX::Zero(6, num_qdot_);

    Sa_.block(0, 6, num_act_joint_, num_act_joint_).setIdentity();
    Sv_.block<6, 6>(0, 0).setIdentity();

    _eye = MatrixX::Identity(num_qdot_, num_qdot_);
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

    if (_dim_rf > 0)
    {
      // Contact Setting
      _ContactBuilding(contact_list);

      // Set inequality constraints
      _SetInEqualityConstraint();
      _WeightedInverse(_Jc, Ainv_, JcBar);
      qddot_pre = JcBar * (-_JcDotQdot);
      Npre = _eye - JcBar * _Jc;
    }
    else
    {
      qddot_pre = VectorX::Zero(num_qdot_);
      Npre = _eye;
    }

    // Task
    TaskPtr task;
    MatrixX Jt, JtBar, JtPre;
    VectorX JtDotQdot, xddot;

    for (auto &task : task_list)
    {
      Jt = task->getTaskJacobian();
      JtDotQdot = task->getTaskJacobianDotQdot();
      xddot = task->getCommand();

      JtPre = Jt * Npre;
      _WeightedInverse(JtPre, Ainv_, JtBar);

      qddot_pre += JtBar * (xddot - JtDotQdot - Jt * qddot_pre);
      Npre = Npre * (_eye - JtBar * JtPre);
    }

    // Set equality constraints
    _SetEqualityConstraint(qddot_pre);

    // Optimization
    // Timer timer;
    solve_quadprog(G, g0, _dyn_CE, _dyn_ce0, _dyn_CI, _dyn_ci0, z);

    // pretty_print(qddot_pre, std::cout, "qddot_cmd");
    for (size_t i(0); i < _dim_floating; ++i)
      qddot_pre[i] += z[i];

    for (size_t i = 0; i < robot::ModelAttrs::dim_config; i++)
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
    size_t dim_accumul_rf, dim_accumul_uf;

    Jc = contact_list[0]->getContactJacobian();
    JcDotQdot = contact_list[0]->getJcDotQdot();
    Uf = contact_list[0]->getRFConstraintMtx();
    Uf_ieq_vec = contact_list[0]->getRFConstraintVec();

    dim_accumul_rf = contact_list[0]->getDim();
    dim_accumul_uf = contact_list[0]->getDimRFConstraint();

    _Jc.block(0, 0, dim_accumul_rf, num_qdot_) = Jc;
    _JcDotQdot.head(dim_accumul_rf) = JcDotQdot;
    _Uf.block(0, 0, dim_accumul_uf, dim_accumul_rf) = Uf;
    _Uf_ieq_vec.head(dim_accumul_uf) = Uf_ieq_vec;
    _Fr_des.head(dim_accumul_rf) = contact_list[0]->getRFDesired();

    size_t dim_new_rf, dim_new_uf;

    for (size_t i(1); i < contact_list.size(); ++i)
    {
      Jc = contact_list[i]->getContactJacobian();
      JcDotQdot = contact_list[i]->getJcDotQdot();

      dim_new_rf = contact_list[i]->getDim();
      dim_new_uf = contact_list[i]->getDimRFConstraint();

      // Jc append
      _Jc.block(dim_accumul_rf, 0, dim_new_rf, num_qdot_) = Jc;

      // JcDotQdot append
      _JcDotQdot.segment(dim_accumul_rf, dim_new_rf) = JcDotQdot;

      // Uf
      Uf = contact_list[i]->getRFConstraintMtx();
      _Uf.block(dim_accumul_uf, dim_accumul_rf, dim_new_uf, dim_new_rf) = Uf;

      // Uf inequality vector
      Uf_ieq_vec = contact_list[i]->getRFConstraintVec();
      _Uf_ieq_vec.segment(dim_accumul_uf, dim_new_uf) = Uf_ieq_vec;

      // Fr desired
      _Fr_des.segment(dim_accumul_rf, dim_new_rf) =
          contact_list[i]->getRFDesired();
      dim_accumul_rf += dim_new_rf;
      dim_accumul_uf += dim_new_uf;
    }
  }

  void Wbic::_SetEqualityConstraint(const VectorX &qddot)
  {

    if (_dim_rf > 0)
    {
      _dyn_CE.block(0, 0, _dim_eq_cstr, _dim_floating) =
          A_.block(0, 0, _dim_eq_cstr, _dim_floating);
      _dyn_CE.block(0, _dim_floating, _dim_eq_cstr, _dim_rf) =
          -Sv_ * _Jc.transpose();
      _dyn_ce0 = -Sv_ * (A_ * qddot + cori_ + grav_ -
                         _Jc.transpose() * _Fr_des);
    }
    else
    {
      _dyn_CE.block(0, 0, _dim_eq_cstr, _dim_floating) =
          A_.block(0, 0, _dim_eq_cstr, _dim_floating);
      _dyn_ce0 = -Sv_ * (A_ * qddot + cori_ + grav_);
    }
  }

  void Wbic::_SetInEqualityConstraint()
  {
    _dyn_CI.block(0, _dim_floating, _dim_Uf, _dim_rf) = _Uf;
    _dyn_ci0 = _Uf_ieq_vec - _Uf * _Fr_des;
  }

  void Wbic::_SetOptimizationSize(const std::vector<ContactPtr> &contact_list)
  {

    // Dimension
    _dim_rf = 0;
    _dim_Uf = 0; // Dimension of inequality constraint
    for (size_t i(0); i < contact_list.size(); ++i)
    {
      _dim_rf += contact_list[i]->getDim();
      _dim_Uf += contact_list[i]->getDimRFConstraint();
    }

    _dim_opt = _dim_floating + _dim_rf;
    _dim_eq_cstr = _dim_floating;

    // Matrix Setting
    G = MatrixX::Zero(_dim_opt, _dim_opt);
    g0 = VectorX::Zero(_dim_opt);

    // Eigen Matrix Setting
    _dyn_CE = MatrixX::Zero(_dim_eq_cstr, _dim_opt);
    _dyn_ce0 = VectorX(_dim_eq_cstr);
    if (_dim_rf > 0)
    {
      _dyn_CI = MatrixX::Zero(_dim_Uf, _dim_opt);
      _dyn_ci0 = VectorX::Zero(_dim_Uf);

      _Jc = MatrixX(_dim_rf, num_qdot_);
      _JcDotQdot = VectorX(_dim_rf);
      _Fr_des = VectorX(_dim_rf);

      _Uf = MatrixX(_dim_Uf, _dim_rf);
      _Uf.setZero();
      _Uf_ieq_vec = VectorX(_dim_Uf);
    }
    else
    {
      _dyn_CI = MatrixX::Zero(1, _dim_opt);
      _dyn_ci0 = VectorX::Zero(1);
    }
  }

  void Wbic::_SetCost()
  {
    // Set Cost
    size_t idx_offset(0);
    for (size_t i(0); i < _dim_floating; ++i)
    {
      G((i + idx_offset), (i + idx_offset)) = _W_floating[i];
    }
    idx_offset += _dim_floating;
    for (size_t i(0); i < _dim_rf; ++i)
    {
      G((i + idx_offset), (i + idx_offset)) = _W_rf[i];
    }
  }

  void Wbic::_GetSolution(const VectorX &qddot, VectorX &cmd)
  {

    VectorX tot_tau;
    if (_dim_rf > 0)
    {
      VectorX _Fr(_dim_rf);
      // get Reaction forces
      for (size_t i(0); i < _dim_rf; ++i)
        _Fr[i] = z[i + _dim_floating] + _Fr_des[i];
      tot_tau =
          A_ * qddot + cori_ + grav_ - _Jc.transpose() * _Fr;
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

}

#include "sdrobot/controllers/wbc.h"
#include "sdrobot/dynamics/math.h"

namespace sdrobot::ctrl::wbc
{
  Wbic::Wbic([[maybe_unused]] size_t num_qdot, double weight) : _dim_floating(6),
                                                                _W_floating(VectorXd::Constant(6, weight)),
                                                                _W_rf(VectorXd::Constant(12, 1.))

  {
    Sa_ = MatrixXd::Zero(num_act_joint_, num_qdot_);
    Sv_ = MatrixXd::Zero(6, num_qdot_);

    Sa_.block(0, 6, num_act_joint_, num_act_joint_).setIdentity();
    Sv_.block(0, 0, 6, 6).setIdentity();

    _eye = MatrixXd::Identity(num_qdot_, num_qdot_);
  }

  void Wbic::UpdateSetting(const MatrixXd &A, const MatrixXd &Ainv,
                           const VectorXd &cori, const VectorXd &grav)
  {
    A_ = A;
    Ainv_ = Ainv;
    cori_ = cori;
    grav_ = grav;
    b_updatesetting_ = true;
  }

  void Wbic::MakeTorque(VectorXd &cmd, const std::vector<TaskPtr> &task_list, const std::vector<ContactPtr> &contact_list)
  {
    if (!b_updatesetting_)
    {
      printf("[Wanning] WBIC setting is not done\n");
    }

    // resize G, g0, CE, ce0, CI, ci0
    _SetOptimizationSize(contact_list);
    _SetCost();

    VectorXd qddot_pre;
    MatrixXd JcBar;
    MatrixXd Npre;

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
      qddot_pre = VectorXd::Zero(num_qdot_);
      Npre = _eye;
    }

    // Task
    TaskPtr task;
    MatrixXd Jt, JtBar, JtPre;
    VectorXd JtDotQdot, xddot;

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
    quadprogpp::solve_quadprog(G, g0, CE, ce0, CI, ci0, z);

    // pretty_print(qddot_pre, std::cout, "qddot_cmd");
    for (size_t i(0); i < _dim_floating; ++i)
      qddot_pre[i] += z[i];

    for (size_t i = 0; i < robot::ModelAttrs::dim_config; i++)
    {
      if (fabs(qddot_pre[i]) > 99999.)
      {
        qddot_pre = VectorXd::Zero(num_qdot_);
      }
    }

    _GetSolution(qddot_pre, cmd);

    _opt_result = VectorXd(_dim_opt);
    for (size_t i(0); i < _dim_opt; ++i)
    {
      _opt_result[i] = z[i];
    }
  }

  void Wbic::_WeightedInverse(const MatrixXd &J, const MatrixXd &Winv, MatrixXd &Jinv, double threshold)
  {
    MatrixXd lambda(J * Winv * J.transpose());
    MatrixXd lambda_inv;
    dynamics::PseudoInverse(lambda, threshold, lambda_inv);
    Jinv = Winv * J.transpose() * lambda_inv;
  }

  void Wbic::_ContactBuilding(const std::vector<ContactPtr> &contact_list)
  {
    MatrixXd Uf;
    VectorXd Uf_ieq_vec;
    // Initial
    MatrixXd Jc;
    VectorXd JcDotQdot;
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

  void Wbic::_SetEqualityConstraint(const VectorXd &qddot)
  {

    if (_dim_rf > 0)
    {
      _dyn_CE.block(0, 0, _dim_eq_cstr, _dim_floating) =
          A_.block(0, 0, _dim_floating, _dim_floating);
      _dyn_CE.block(0, _dim_floating, _dim_eq_cstr, _dim_rf) =
          -Sv_ * _Jc.transpose();
      _dyn_ce0 = -Sv_ * (A_ * qddot + cori_ + grav_ -
                         _Jc.transpose() * _Fr_des);
    }
    else
    {
      _dyn_CE.block(0, 0, _dim_eq_cstr, _dim_floating) =
          A_.block(0, 0, _dim_floating, _dim_floating);
      _dyn_ce0 = -Sv_ * (A_ * qddot + cori_ + grav_);
    }

    for (size_t i(0); i < _dim_eq_cstr; ++i)
    {
      for (size_t j(0); j < _dim_opt; ++j)
      {
        CE[j][i] = _dyn_CE(i, j);
      }
      ce0[i] = -_dyn_ce0[i];
    }
  }

  void Wbic::_SetInEqualityConstraint()
  {
    _dyn_CI.block(0, _dim_floating, _dim_Uf, _dim_rf) = _Uf;
    _dyn_ci0 = _Uf_ieq_vec - _Uf * _Fr_des;

    for (size_t i(0); i < _dim_Uf; ++i)
    {
      for (size_t j(0); j < _dim_opt; ++j)
      {
        CI[j][i] = _dyn_CI(i, j);
      }
      ci0[i] = -_dyn_ci0[i];
    }
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
    G.resize(0., _dim_opt, _dim_opt);
    g0.resize(0., _dim_opt);
    CE.resize(0., _dim_opt, _dim_eq_cstr);
    ce0.resize(0., _dim_eq_cstr);

    // Eigen Matrix Setting
    _dyn_CE = MatrixXd::Zero(_dim_eq_cstr, _dim_opt);
    _dyn_ce0 = VectorXd(_dim_eq_cstr);
    if (_dim_rf > 0)
    {
      CI.resize(0., _dim_opt, _dim_Uf);
      ci0.resize(0., _dim_Uf);
      _dyn_CI = MatrixXd::Zero(_dim_Uf, _dim_opt);
      _dyn_ci0 = VectorXd(_dim_Uf);

      _Jc = MatrixXd(_dim_rf, num_qdot_);
      _JcDotQdot = VectorXd(_dim_rf);
      _Fr_des = VectorXd(_dim_rf);

      _Uf = MatrixXd(_dim_Uf, _dim_rf);
      _Uf.setZero();
      _Uf_ieq_vec = VectorXd(_dim_Uf);
    }
    else
    {
      CI.resize(0., _dim_opt, 1);
      ci0.resize(0., 1);
    }
  }

  void Wbic::_SetCost()
  {
    // Set Cost
    size_t idx_offset(0);
    for (size_t i(0); i < _dim_floating; ++i)
    {
      G[i + idx_offset][i + idx_offset] = _W_floating[i];
    }
    idx_offset += _dim_floating;
    for (size_t i(0); i < _dim_rf; ++i)
    {
      G[i + idx_offset][i + idx_offset] = _W_rf[i];
    }
  }

  void Wbic::_GetSolution(const VectorXd &qddot, VectorXd &cmd)
  {

    VectorXd tot_tau;
    if (_dim_rf > 0)
    {
      _Fr = VectorXd(_dim_rf);
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
    _qddot = qddot;

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

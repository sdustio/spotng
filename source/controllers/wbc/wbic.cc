#include "sdrobot/controllers/wbc.h"
#include "sdrobot/dynamics/math.h"

namespace sdrobot::ctrl::wbc
{
  Wbic::Wbic([[maybe_unused]] size_t num_qdot, double weight) : _dim_floating(6),
                                               _W_floating(VectorXd::Constant(6, weight)),
                                               _W_rf(VectorXd::Constant(12, 1.))

  {
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
    _SetOptimizationSize();
    _SetCost();

    VectorXd qddot_pre;
    MatrixXd JcBar;
    MatrixXd Npre;

    if (_dim_rf > 0)
    {
      // Contact Setting
      _ContactBuilding();

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

  void Wbic::_ContactBuilding() {}
  void Wbic::_SetEqualityConstraint([[maybe_unused]] const VectorXd &qddot) {}
  void Wbic::_SetInEqualityConstraint() {}

  void Wbic::_SetOptimizationSize() {}
  void Wbic::_SetCost() {}
  void Wbic::_GetSolution([[maybe_unused]] const VectorXd &qddot, [[maybe_unused]] VectorXd &cmd) {}

}

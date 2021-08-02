#include <unsupported/Eigen/MatrixFunctions>

#include "mpc/qp_solver.h"
#include "dynamics/rotation.h"
#include "math/algebra.h"

namespace sdrobot::mpc
{
  namespace
  {
    bool near_zero(double a)
    {
      return (a < .01 && a > -.01);
    }

    bool near_one(double a)
    {
      return near_zero(a - 1);
    }
  }

  void QPSolver::SolveQP(fpt_t const x_drag, SdVector3f const &pos, SdVector3f const &vel,
                         SdVector4f const &ori, SdVector3f const &vel_rpy, std::array<fpt_t, 12> const &rel_foot_p,
                         fpt_t const yaw, std::array<fpt_t, 12> const &weights,
                         std::array<fpt_t, 12 * 36> const &state_trajectory, fpt_t alpha, fpt_t g,
                         std::vector<int> const &gait)
  {

    Eigen::Map<A_qp_t> A_qp(A_qp_.data());
    Eigen::Map<B_qp_t> B_qp(B_qp_.data());
    Eigen::Map<S_t> S(S_.data());
    Eigen::Map<eye_12h_t> eye_12h(eye_12h_.data());
    Eigen::Map<X_d_t> X_d(X_d_.data());
    Eigen::Map<qH_t> qH(qH_.data());
    Eigen::Map<qA_t> qA(qA_.data());
    Eigen::Map<qub_t> qub(qub_.data());
    Eigen::Map<qlb_t> qlb(qlb_.data());
    Eigen::Map<qg_t> qg(qg_.data());
    Eigen::Map<qsoln_t> qsoln(qsoln_.data());

    Matrix3x4 r_feet;
    for (int rs = 0; rs < 3; rs++)
      for (int c = 0; c < 4; c++)
        r_feet(rs, c) = rel_foot_p[rs * 4 + c];
    Matrix3 I_body, R_yaw;
    auto yc = cos(yaw);
    auto ys = sin(yaw);
    R_yaw << yc, -ys, 0,
        ys, yc, 0,
        0, 0, 1;
    I_body.diagonal() << .07, 0.26, 0.242;

    Vector3 rpy;
    dynamics::QuatToRPY(rpy, ToConstEigenTp(ori));

    Eigen::Matrix<fpt_t, 13, 1> x_0;
    x_0 << rpy(2), rpy(1), rpy(0), ToConstEigenTp(pos), ToConstEigenTp(vel_rpy), ToConstEigenTp(vel), -g;

    Matrix3 I_world = R_yaw * I_body * R_yaw.transpose();

    //continuous time state space matrices.
    MatrixX A_ct = Eigen::Matrix<fpt_t, 13, 13>::Zero();
    MatrixX B_ct_r = Eigen::Matrix<fpt_t, 13, 12>::Zero();

    A_ct(3, 9) = 1.;
    A_ct(11, 9) = x_drag;
    A_ct(4, 10) = 1.;
    A_ct(5, 11) = 1.;
    A_ct(11, 12) = 1.;
    A_ct.block<3, 3>(0, 6) = R_yaw.transpose();

    for (int b = 0; b < 4; b++)
    {
      Matrix3 rotm;
      math::VecToSkewMat(rotm, r_feet.col(b));
      B_ct_r.block<3, 3>(6, b * 3) = I_world.inverse() * rotm;
      B_ct_r.block<3, 3>(9, b * 3) = Matrix3::Identity() / m_;
    }

    //QP matrices
    MatrixX Adt; //shape: 13 x 13
    MatrixX Bdt; //shape: 13 x 12

    MatrixX ABc = Eigen::Matrix<fpt_t, 25, 25>::Zero();
    ABc.block<13, 13>(0, 0) = A_ct;
    ABc.block<13, 12>(0, 13) = B_ct_r;
    ABc = dt_ * ABc;
    MatrixX expmm = ABc.exp(); // shape: 25 x 25
    Adt = expmm.block<13, 13>(0, 0);
    Bdt = expmm.block<13, 12>(0, 13);

    MatrixX powerMats[20]; // shape: 13 x 13
    powerMats[0].setIdentity(13, 13);
    for (int i = 1; i < params::horizon_len + 1; i++)
    {
      powerMats[i] = Adt * powerMats[i - 1];
    }

    for (int r = 0; r < params::horizon_len; r++)
    {
      A_qp.block<13, 13>(13 * r, 0) = powerMats[r + 1]; //Adt.pow(r+1);
      for (int c = 0; c < params::horizon_len; c++)
      {
        if (r >= c)
        {
          int a_num = r - c;
          B_qp.block(13 * r, 12 * c, 13, 12) = powerMats[a_num] /*Adt.pow(a_num)*/ * Bdt;
        }
      }
    }

    //weights
    Eigen::Matrix<double, 13, 1> full_weight;
    for (int i = 0; i < 12; i++)
      full_weight(i) = weights[i];
    full_weight(12) = 0.;
    S.diagonal() = full_weight.replicate(params::horizon_len, 1);

    //trajectory
    for (int i = 0; i < params::horizon_len; i++)
    {
      for (int j = 0; j < 12; j++)
        X_d(13 * i + j, 0) = state_trajectory[12 * i + j];
    }

    int k = 0;
    for (int i = 0; i < params::horizon_len; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        qub(5 * k + 0) = params::big_num;
        qub(5 * k + 1) = params::big_num;
        qub(5 * k + 2) = params::big_num;
        qub(5 * k + 3) = params::big_num;
        qub(5 * k + 4) = gait[i * 4 + j] * f_max_;
        k++;
      }
    }

    double rep_mu = 1. / (mu_ + 1.e-12);
    Eigen::Matrix<double, 5, 3> f_block;
    f_block << rep_mu, 0, 1.,
        -rep_mu, 0, 1.,
        0, rep_mu, 1.,
        0, -rep_mu, 1.,
        0, 0, 1.;

    for (int i = 0; i < params::horizon_len * 4; i++)
    {
      qA.block<5, 3>(i * 5, i * 3) = f_block;
    }

    qH = 2 * (B_qp.transpose() * S * B_qp + alpha * eye_12h);
    qg = 2 * B_qp.transpose() * S * (A_qp * x_0 - X_d);

    int nWSR = 100;
    int new_cons = num_constraints;
    int new_vars = num_variables;

    for (int i = 0; i < num_constraints; i++)
    {
      if (!(near_zero(qlb_[i]) && near_zero(qub_[i])))
        continue;
      auto c_row = qA.row(i);

      for (int j = 0; j < num_variables; j++)
      {
        if (near_one(c_row[j]))
        {
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

    for (int i = 0; i < num_variables; i++)
    {
      if (!var_elim_[i])
      {
        if (vc >= new_vars)
        {
          throw("BAD ERROR 1\n");
        }
        var_ind_[vc] = i;
        vc++;
      }
    }
    vc = 0;
    for (int i = 0; i < num_constraints; i++)
    {
      if (!con_elim_[i])
      {
        if (vc >= new_cons)
        {
          throw("BAD ERROR 1\n");
        }
        con_ind_[vc] = i;
        vc++;
      }
    }

    for (int i = 0; i < new_vars; i++)
    {
      int olda = var_ind_[i];
      g_red_[i] = qg_[olda];
      for (int j = 0; j < new_vars; j++)
      {
        int oldb = var_ind_[j];
        H_red_[i * new_vars + j] = qH(olda, oldb);
      }
    }

    for (int con = 0; con < new_cons; con++)
    {
      for (int st = 0; st < new_vars; st++)
      {
        auto cval = qA(con_ind_[con], var_ind_[st]);
        A_red_[con * new_vars + st] = cval;
      }
    }

    for (int i = 0; i < new_cons; i++)
    {
      int old = con_ind_[i];
      ub_red_[i] = qub_[old];
      lb_red_[i] = qlb_[old];
    }

    qpOASES::QProblem problem_red(new_vars, new_cons);
    qpOASES::Options op;
    op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    problem_red.setOptions(op);
    //int_t nWSR = 50000;

    problem_red.init(H_red_.data(), g_red_.data(), A_red_.data(), nullptr, nullptr, lb_red_.data(), ub_red_.data(), nWSR);

    int rval2 = problem_red.getPrimalSolution(q_red_.data());
    if (rval2 != qpOASES::SUCCESSFUL_RETURN)
      printf("failed to solve!\n");

    vc = 0;
    for (int i = 0; i < num_variables; i++)
    {
      if (var_elim_[i])
      {
        qsoln_[i] = 0.0;
      }
      else
      {
        qsoln_[i] = q_red_[vc];
        vc++;
      }
    }
  }

  void QPSolver::Setup(fpt_t dt, fpt_t mu, fpt_t f_max)
  {
    f_max_ = f_max;
    mu_ = mu;
    dt_ = dt;

    ResetQPMats();
  }

  void QPSolver::ResetQPMats()
  {
    A_qp_.fill(0.);
    B_qp_.fill(0.);
    S_.fill(0.);
    X_d_.fill(0.);

    qub_.fill(0.);
    qlb_.fill(0.);
    qA_.fill(0.);
    qH_.fill(0.);
    qg_.fill(0.);
    qsoln_.fill(0.);
    var_elim_.fill(0.);
    con_elim_.fill(0.);

    H_red_.fill(0.);
    A_red_.fill(0.);
    ub_red_.fill(0.);
    lb_red_.fill(0.);
    g_red_.fill(0.);
    q_red_.fill(0.);
    var_ind_.fill(0.);
    con_ind_.fill(0.);
  }

  QPSolver::QPSolver()
  {
    Eigen::Map<eye_12h_t> eye_12h(eye_12h_.data());
    eye_12h.setIdentity();
  }

}

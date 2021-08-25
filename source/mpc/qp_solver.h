#pragma once

#include <qpOASES.hpp>

#include "mpc/mpc.h"
#include "eigen.h"

namespace sdrobot::mpc
{
  constexpr inline int const num_constraints = 20 * opts::horizon_len;
  constexpr inline int const num_variables = 12 * opts::horizon_len;

  class QPSolver
  {
  public:
    QPSolver();
    bool Setup(fpt_t dt, fpt_t mu, fpt_t f_max);

    bool ResetQPMats();

    bool SolveQP(fpt_t const x_drag, SdVector3f const &pos, SdVector3f const &lvel,
                 SdVector4f const &ori, SdVector3f const &avel, std::array<fpt_t, 12> const &rel_foot_p,
                 fpt_t const yaw, std::array<fpt_t, 12> const &weights,
                 std::array<fpt_t, 12 * 36> const &state_trajectory, fpt_t alpha, fpt_t g,
                 std::vector<int> const &gait);
    std::array<fpt_t, 12 * opts::horizon_len> const &GetSolution() { return qsoln_; }

  private:
    fpt_t dt_;
    fpt_t mu_;
    fpt_t f_max_;
    fpt_t m_ = 10.5;

    std::array<fpt_t, (13 * opts::horizon_len) * 13> A_qp_;
    std::array<fpt_t, (13 * opts::horizon_len) * (12 * opts::horizon_len)> B_qp_;
    std::array<fpt_t, (13 * opts::horizon_len) * (13 * opts::horizon_len)> S_;
    std::array<fpt_t, (12 * opts::horizon_len) * (12 * opts::horizon_len)> eye_12h_;
    std::array<fpt_t, (13 * opts::horizon_len)> X_d_;

    std::array<fpt_t, num_variables * num_variables> qH_;
    std::array<fpt_t, num_constraints * num_variables> qA_;
    std::array<fpt_t, num_constraints> qub_;
    std::array<fpt_t, num_constraints> qlb_;
    std::array<fpt_t, num_variables> qg_;

    std::array<fpt_t, num_variables> qsoln_;

    std::array<char, num_variables> var_elim_;
    std::array<char, num_constraints> con_elim_;

    std::array<qpOASES::real_t, num_variables * num_variables> H_red_;
    std::array<qpOASES::real_t, num_constraints * num_variables> A_red_;
    std::array<qpOASES::real_t, num_constraints> ub_red_;
    std::array<qpOASES::real_t, num_constraints> lb_red_;
    std::array<qpOASES::real_t, num_variables> g_red_;

    std::array<qpOASES::real_t, num_variables> q_red_;

    std::array<int, num_variables> var_ind_;
    std::array<int, num_constraints> con_ind_;
  };
}

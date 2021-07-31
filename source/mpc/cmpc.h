#pragma once

#include <unordered_map>

#include "mpc/mpc.h"
#include "mpc/gait.h"
#include "mpc/foot_swing.h"

namespace sdrobot::mpc
{

  struct Params
  {
    constexpr static int bonus_swing = 0;
    constexpr static int cmpc_x_drag = 3;
    constexpr static int max_gait_segments = 36;
    constexpr static double big_num = 5e10;
  };

  class QPSolver
  {
  public:
    void Setup(double dt, int horizonLen, double mu, double f_max);
    void ResizeQPMats();
    void SolveQP(double x_drag, const Vector3 &p, const Vector3 &v, const dynamics::Quat &q, const Vector3 &w,
                 const Eigen::Matrix<double, 12, 1> &r, double yaw, Eigen::Matrix<double, 12, 1> &weights,
                 const Eigen::Matrix<double, 12 * 36, 1> &state_trajectory, double alpha, const Eigen::VectorXi &gait);
    const VectorX &GetSolution() { return qsoln_; }

  private:
    double dt_;
    double mu_;
    double f_max_;
    int horizon_ = 0;
    double m_ = 10.5;

    Eigen::Matrix<double, Eigen::Dynamic, 13> A_qp_;
    MatrixX B_qp_;
    MatrixX S_, eye_12h_;
    VectorX X_d_;

    MatrixX qH_, qA_;
    VectorX qg_, qlb_, qub_;
    std::array<char, 250> var_elim_ = {};
    std::array<char, 250> con_elim_ = {};

    VectorX qsoln_;
  };

  class CMpc : public Mpc
  {
  public:
    CMpc(double dt, int iterations_between_mpc);
    bool Init() override;
    bool Run(WbcData &data, LegPtr &cleg, const robot::QuadrupedPtr &quad, const StateCmdPtr &cmd, const est::StateEstPtr &est) override;

  private:
    void UpdateMPCIfNeeded(std::array<Vector3, 4> &out, const Eigen::VectorXi &mpcTable, const StateCmdPtr &cmd, const est::StateEstPtr &est, const Vector3 &v_des_world);
    void SolveMPC(std::array<Vector3, 4> &out, const Eigen::VectorXi &mpcTable, const est::StateEstPtr &est);

    double dt_;
    double dt_mpc_;
    int horizon_len_ = 10;
    int iter_between_mpc_;
    int iteration_counter_ = 0;

    std::array<FootSwingTrajectory, 4> foot_swing_trajs_;
    std::array<bool, 4> first_swing_;
    Vector4 swing_times_;
    Vector4 swing_time_remaining_;

    Vector3 pos_des_world_;
    Vector3 rpy_int_;
    std::array<Vector3, 4> p_foot_;
    double x_comp_integral = 0;

    Eigen::Matrix<double, 6, 1> stand_traj_;
    Eigen::Matrix<double, 12 * 36, 1> traj_all_;

    Gait current_gait_;
    bool first_run_ = true;

    Matrix3 Kp_, Kd_, Kp_stance_, Kd_stance_;

    QPSolver qpsolver_;
    // RobotState qs_;

    std::unordered_map<Gait, GaitSkdPtr> gait_map_;
  };
}

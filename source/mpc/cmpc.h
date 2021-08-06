#pragma once

#include <unordered_map>

#include "sdrobot/drive.h"
#include "mpc/mpc.h"
#include "mpc/gait_skd.h"
#include "mpc/foot_swing.h"
#include "mpc/qp_solver.h"

namespace sdrobot::mpc
{

  class CMpc : public Mpc
  {
  public:
    CMpc(fpt_t dt, fpt_t g, int iter_between_mpc);
    bool Init() override;
    bool RunOnce(wbc::InData &wbcdata,
                 leg::LegCtrl::SharedPtr const &legctrl,
                 model::Quadruped::ConstSharedPtr const &quad,
                 drive::DriveCtrl::ConstSharedPtr const &drivectrl,
                 estimate::EstimateCtrl::ConstSharedPtr const &estctrl) override;

  private:
    bool UpdateMPCIfNeeded(
        std::array<SdVector3f, 4> &out,
        std::vector<int> const &mpcTable,
        drive::DriveCtrl::ConstSharedPtr const &drivectrl,
        estimate::EstimateCtrl::ConstSharedPtr const &estctrl,
        SdVector3f const &v_des_world);
    bool SolveMPC(
        std::array<SdVector3f, 4> &out,
        std::vector<int> const &mpcTable,
        estimate::EstimateCtrl::ConstSharedPtr const &estctrl);

    fpt_t dt_;
    fpt_t dt_mpc_;
    fpt_t gravity_;
    int iter_between_mpc_;
    int iter_counter_ = 0;

    std::array<FootSwingTrajectory, 4> foot_swing_trajs_;
    std::array<bool, 4> first_swing_;
    SdVector4f swing_times_;
    SdVector4f swing_time_remaining_;

    SdVector3f pos_des_world_;
    SdVector3f pos_rpy_int_;
    std::array<SdVector3f, 4> p_foot_;
    fpt_t x_comp_integral = 0;

    SdVector6f stand_traj_;
    std::array<fpt_t, 12 * 36> traj_all_;

    drive::Gait current_gait_;
    bool first_run_ = true;
    SdMatrix3f kp_stance_ = {};
    SdMatrix3f kd_stance_ = {7, 0, 0, 0, 7, 0, 0, 0, 7}; //row major == column major

    std::unique_ptr<QPSolver> qpsolver_;

    std::unordered_map<drive::Gait, GaitSkd::Ptr> gait_map_;
  };
}

#include "mpc/cmpc.h"
#include "dynamics/rotation.h"

namespace sdrobot::mpc
{
  CMpc::CMpc(fpt_t dt, fpt_t g, int iter_between_mpc_) : dt_(dt),
                                                         dt_mpc_(dt * iter_between_mpc_),
                                                         gravity_(g),
                                                         iter_between_mpc_(iter_between_mpc_),
                                                         pos_rpy_int_({})
  {
    first_swing_.fill(true);
    qpsolver_ = std::make_unique<QPSolver>();
    gait_map_[drive::Gait::Trot] = std::make_unique<OffsetDurationGait>(opts::horizon_len, SdVector4i{0, 5, 5, 0}, SdVector4i{5, 5, 5, 5}, "Trot");
    gait_map_[drive::Gait::SlowTrot] = std::make_unique<OffsetDurationGait>(int(opts::horizon_len * 1.2), SdVector4i{0, 6, 6, 0}, SdVector4i{6, 6, 6, 6}, "SlowTrot");
    gait_map_[drive::Gait::FlyingTrot] = std::make_unique<OffsetDurationGait>(opts::horizon_len, SdVector4i{0, 5, 5, 0}, SdVector4i{4, 4, 4, 4}, "FlyingTrot");
    gait_map_[drive::Gait::Walk] = std::make_unique<OffsetDurationGait>(int(opts::horizon_len * 1.6), SdVector4i{0, 8, 4, 12}, SdVector4i{12, 12, 12, 12}, "Walk");
    gait_map_[drive::Gait::Bound] = std::make_unique<OffsetDurationGait>(opts::horizon_len, SdVector4i{5, 5, 0, 0}, SdVector4i{5, 5, 5, 5}, "Bound");
  }

  bool CMpc::Run(wbc::InData &wbcdata,
                 leg::LegCtrl::SharedPtr &legctrl,
                 model::Quadruped::SharedPtr const &quad,
                 drive::DriveCtrl::SharedPtr const &drivectrl,
                 estimate::EstimateCtrl::SharedPtr const &estctrl)
  {
    auto _body_height = 0.3 + drivectrl->GetPosDes()[2];

    auto const &seResult = estctrl->GetEstState();

    // some first time initialization
    if (first_run_)
    {
      stand_traj_[0] = seResult.pos[0];
      stand_traj_[1] = seResult.pos[1];
      stand_traj_[2] = _body_height; // ?? 0.21;
      stand_traj_[3] = 0;
      stand_traj_[4] = 0;
      stand_traj_[5] = seResult.pos_rpy[2];
      pos_des_world_[0] = stand_traj_[0];
      pos_des_world_[1] = stand_traj_[1];
      pos_des_world_[2] = stand_traj_[2]; //?? seResult.rpy[2];

      for (int i = 0; i < 4; i++)
      {

        foot_swing_trajs_[i].UpdateHeight(0.05);
        foot_swing_trajs_[i].UpdateInitialPosition(p_foot_[i]);
        foot_swing_trajs_[i].UpdateFinalPosition(p_foot_[i]);
      }
      first_run_ = false;
    }

    auto cmd_gait = drivectrl->GetGait();
    GaitSkd::Ptr &gait_skd = gait_map_[cmd_gait];

    gait_skd->SetIterations(iter_between_mpc_, iter_counter_);

    if (_body_height < 0.02)
    {
      _body_height = 0.3;
    }

    if (cmd_gait == drive::Gait::FlyingTrot)
      _body_height = 0.31;

    // integrate position setpoint
    auto rot_body = ToConstEigenTp(seResult.rot_body);
    auto vel_world = ToConstEigenTp(seResult.vel_world);
    auto pos = ToConstEigenTp(seResult.pos);
    auto pos_rpy = ToConstEigenTp(seResult.pos_rpy);

    auto vel_des_robot = ToConstEigenTp(drivectrl->GetVelDes());
    auto vel_rpy_des_robot = ToConstEigenTp(drivectrl->GetVelRpyDes());
    auto pos_rpy_des_robot = ToConstEigenTp(drivectrl->GetPosRpyDes());
    SdVector3f sd_vel_des_world;
    ToEigenTp(sd_vel_des_world) = rot_body.transpose() * vel_des_robot;

    //Integral-esque pitche and roll compensation
    if (fabs(vel_world[0]) > .02) //avoid dividing by zero
    {
      pos_rpy_int_[1] += 5 * dt_ * (pos_rpy_des_robot[1] - pos_rpy[1]) / vel_world[0];
    }
    if (fabs(vel_world[1]) > 0.01)
    {
      pos_rpy_int_[0] += dt_ * (pos_rpy_des_robot[0] - pos_rpy[0]) / vel_world[1];
    }

    pos_rpy_int_[0] = fminf(fmaxf(pos_rpy_int_[0], -.25), .25);
    pos_rpy_int_[1] = fminf(fmaxf(pos_rpy_int_[1], -.25), .25);

    for (int i = 0; i < 4; i++)
    {
      SdVector3f loc;
      quad->CalcHipLocation(loc, i);
      ToEigenTp(p_foot_[i]) = pos + rot_body.transpose() * (ToConstEigenTp(loc) +
                                                            ToConstEigenTp(legctrl->GetDatas()[i].p));
    }

    ToEigenTp(pos_des_world_) += dt_ * ToConstEigenTp(sd_vel_des_world);
    pos_des_world_[2] = _body_height;

    // foot placement
    for (int l = 0; l < 4; l++)
      swing_times_[l] = gait_skd->GetCurrentSwingTime(dt_mpc_, l);

    fpt_t side_sign[4] = {-1, 1, -1, 1};
    //  fptype interleave_y[4] = {-0.08, 0.08, 0.02, -0.02};
    fpt_t interleave_y[4] = {0.0, 0.0, 0.02, -0.02};
    //fptype interleave_gain = -0.13;
    fpt_t interleave_gain = 0; //-0.2;
    //fptype v_abs = std::fabs(seResult.vBody[0]);
    fpt_t v_abs = std::fabs(vel_des_robot[0]);

    for (int i = 0; i < 4; i++)
    {
      if (first_swing_[i])
      {
        swing_time_remaining_[i] = swing_times_[i];
      }
      else
      {
        swing_time_remaining_[i] -= dt_;
      }
      //if(first_swing_[i]) {
      //foot_swing_trajs_[i].UpdateHeight(.05);

      foot_swing_trajs_[i].UpdateHeight(drivectrl->GetStepHeight()); //.125);

      //    Vector3 offset(0.05, side_sign[i] * .062, 0);
      Vector3 offset(0, side_sign[i] * .072, 0);
      //     Vector3 offset(0, side_sign[i] * .075, 0);

      if (i < 2)
        offset[0] = 0; //0.03;
      else
        offset[0] = -0.02; //0.02;//

      //      offset[0]=seResult.vBody[0]*0.01;

      if (cmd_gait == drive::Gait::Walk) //walk gait
      {
        if (i == 0)
          offset(1) = -0.085 * (1 - fabs(vel_des_robot[0]) / 2.0);
        else if (i == 1)
          offset(1) = 0.085 * (1 - fabs(vel_des_robot[0]) / 2.0);
      }

      SdVector3f loc;
      quad->CalcHipLocation(loc, i);
      Vector3 pRobotFrame = ToConstEigenTp(loc) + offset;

      pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;
      fpt_t stance_time = gait_skd->GetCurrentStanceTime(dt_mpc_, i);
      Matrix3 _rot;
      dynamics::CoordinateRot(_rot, dynamics::CoordinateAxis::Z, -vel_rpy_des_robot[2] * stance_time / 2);
      Vector3 pYawCorrected = _rot * pRobotFrame;

      Vector3 Pf = pos + rot_body.transpose() * (pYawCorrected + vel_des_robot * swing_time_remaining_[i]);

      //+ vel_world * swing_time_remaining_[i];

      fpt_t p_rel_max = 0.35;
      //    fptype p_rel_max = 0.3f;

      // Using the estimated velocity is correct
      //Vector3 v_des_robot_world = seResult.rot_body.transpose() * vel_des_robot;
      fpt_t pfx_rel = vel_world[0] * (.5 + opts::bonus_swing) * stance_time +
                      .1 * (vel_world[0] - sd_vel_des_world[0]) +
                      (0.5 * pos[2] / 9.81) * (vel_world[1] * vel_rpy_des_robot[2]);

      if (fabs(pfx_rel) > p_rel_max)
        printf("!!!!!!!!!!!!!!!!out of the max step\n");

      fpt_t pfy_rel = vel_world[1] * .5 * stance_time * dt_mpc_ +
                      .09 * (vel_world[1] - sd_vel_des_world[1]) +
                      (0.5 * pos[2] / 9.81) * (-vel_world[0] * vel_rpy_des_robot[2]);
      pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
      pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
      Pf[0] += pfx_rel;
      Pf[1] += pfy_rel;
      Pf[2] = -0.01; //0;//-0.003; //
      //Pf[2] = 0.0;
      foot_swing_trajs_[i].UpdateFinalPosition({Pf[0], Pf[1], Pf[2]});
    }

    // calc gait
    iter_counter_++;

    // gait
    SdVector4f swingStates;
    gait_skd->CalcSwingState(swingStates);

    UpdateMPCIfNeeded(
        wbcdata.Fr_des, gait_skd->GetMpcTable(), drivectrl, estctrl, sd_vel_des_world);

    //  StateEstimator* se = hw_i->state_estimator;
    for (int foot = 0; foot < 4; foot++)
    {
      fpt_t swingState = swingStates[foot];
      if (swingState > 0) // foot is in swing
      {
        if (first_swing_[foot])
        {
          first_swing_[foot] = false;
          foot_swing_trajs_[foot].UpdateInitialPosition(p_foot_[foot]);
        }

        foot_swing_trajs_[foot].ComputeSwingTrajectoryBezier(swingState, swing_times_[foot]);

        // Update for WBC
        wbcdata.p_foot_des[foot] = foot_swing_trajs_[foot].GetPosition();
        wbcdata.v_foot_des[foot] = foot_swing_trajs_[foot].GetVelocity();
        wbcdata.a_foot_des[foot] = foot_swing_trajs_[foot].GetAcceleration();
      }
      else // foot is in stance
      {
        first_swing_[foot] = true;

        // Stance foot damping
        auto &leg_cmds = legctrl->GetCmdsForUpdate();
        SdVector3f loc;
        quad->CalcHipLocation(loc, foot);
        ToEigenTp(leg_cmds[foot].p_des) =
            rot_body * (ToConstEigenTp(foot_swing_trajs_[foot].GetPosition()) - pos) - ToConstEigenTp(loc);
        ToEigenTp(leg_cmds[foot].v_des) =
            rot_body * (ToConstEigenTp(foot_swing_trajs_[foot].GetVelocity()) - vel_world);
        //cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";
      }
    }

    // Update For WBC
    wbcdata.p_body_des = pos_des_world_;
    wbcdata.v_body_des = sd_vel_des_world;
    wbcdata.a_body_des.fill(0.);
    wbcdata.p_body_rpy_des = drivectrl->GetPosRpyDes();
    wbcdata.vbody_ori_des = drivectrl->GetVelRpyDes();
    gait_skd->CalcContactState(wbcdata.contact_state);
    // END of WBC Update

    return true;
  }

  bool CMpc::Init()
  {
    return true;
  }

  void CMpc::UpdateMPCIfNeeded(
      std::array<SdVector3f, 4> &out,
      std::vector<int> const &mpcTable,
      drive::DriveCtrl::SharedPtr const &drivectrl,
      estimate::EstimateCtrl::SharedPtr const &estctrl,
      const SdVector3f &vel_des_world)
  {
    //iter_between_mpc_ = 30;
    if ((iter_counter_ % iter_between_mpc_) != 0)
    {
      return;
    }

    if ((iter_counter_ / iter_between_mpc_) >= opts::horizon_len)
    {
      iter_counter_ = 0;
    }

    auto const &seResult = estctrl->GetEstState();
    auto const &pos = seResult.pos;
    auto const &v_world = seResult.vel_world;

    SdVector3f rpy_comp = {};
    rpy_comp[1] = v_world[0] * pos_rpy_int_[1];
    rpy_comp[0] = v_world[1] * pos_rpy_int_[0];
    rpy_comp[2] = drivectrl->GetPosRpyDes()[2]; // angle_y

    fpt_t const max_pos_error = .1;
    fpt_t xStart = pos_des_world_[0];
    fpt_t yStart = pos_des_world_[1];

    if (xStart - pos[0] > max_pos_error)
      xStart = pos[0] + max_pos_error;
    if (pos[0] - xStart > max_pos_error)
      xStart = pos[0] - max_pos_error;

    if (yStart - pos[1] > max_pos_error)
      yStart = pos[1] + max_pos_error;
    if (pos[1] - yStart > max_pos_error)
      yStart = pos[1] - max_pos_error;

    pos_des_world_[0] = xStart;
    pos_des_world_[1] = yStart;

    auto const &velrpy = drivectrl->GetVelRpyDes();

    fpt_t trajInitial[12] = {rpy_comp[0],       // 0
                             rpy_comp[1],       // 1
                             rpy_comp[2],       // 2
                             pos_des_world_[0], // 3
                             pos_des_world_[1], // 4
                             pos_des_world_[2], // 5
                             velrpy[0],         // 6
                             velrpy[1],         // 7
                             velrpy[2],         // 8
                             vel_des_world[0],  // 9
                             vel_des_world[1],  // 10
                             vel_des_world[2]}; // 11

    for (int i = 0; i < opts::horizon_len; i++)
    {
      for (int j = 0; j < 12; j++)
        traj_all_[12 * i + j] = trajInitial[j];

      if (i == 0) // start at current position  TODO consider not doing this
      {
        //traj_all_[3] = hw_i->state_estimator->se_pBody[0];
        //traj_all_[4] = hw_i->state_estimator->se_pBody[1];
        traj_all_[2] = seResult.pos_rpy[2];
      }
      else
      {
        traj_all_[12 * i + 3] = traj_all_[12 * (i - 1) + 3] + dt_mpc_ * vel_des_world[0];
        traj_all_[12 * i + 4] = traj_all_[12 * (i - 1) + 4] + dt_mpc_ * vel_des_world[1];
        traj_all_[12 * i + 2] = traj_all_[12 * (i - 1) + 2] + dt_mpc_ * velrpy[2];
      }
    }
    SolveMPC(out, mpcTable, estctrl);
    //    printf("TOTAL SOLVE TIME: %.3f\n", solveTimer.getMs());
  }

  void CMpc::SolveMPC(
      std::array<SdVector3f, 4> &out,
      std::vector<int> const &mpcTable,
      estimate::EstimateCtrl::SharedPtr const &estctrl)
  {
    const auto &seResult = estctrl->GetEstState();

    std::array<fpt_t, 12> weights = {1.25, 1.25, 10, 2, 2, 50, 0, 0, 0.3, 1.5, 1.5, 0.2};
    //fptype Q[12] = {0.25, 0.25, 10, 2, 2, 40, 0, 0, 0.3, 0.2, 0.2, 0.2};
    fpt_t yaw = seResult.pos_rpy[2];
    fpt_t alpha = 4e-5; // make setting eventually
    //fptype alpha = 4e-7; // make setting eventually: DH
    const auto &pos = seResult.pos;               //p
    const auto &v_world = seResult.vel_world;     //v
    const auto &w_world = seResult.vel_rpy_world; //w
    const auto &ori = seResult.ori;               //q

    std::array<fpt_t, 12> r;
    for (int i = 0; i < 12; i++)
      r[i] = p_foot_[i % 4][i / 4] - pos[i / 4];

    if (alpha > 1e-4)
    {
      printf("Alpha was set too high (%f) adjust to 1e-5\n", alpha);
      alpha = 1e-5;
    }

    fpt_t pz_err = pos[2] - pos_des_world_[2];

    Vector3 vxy(v_world[0], v_world[1], 0);

    qpsolver_->Setup(dt_mpc_, 0.4, 150);

    if (vxy[0] > 0.3 || vxy[0] < -0.3)
    {
      x_comp_integral += opts::cmpc_x_drag * pz_err * dt_mpc_ / vxy[0];
    }

    qpsolver_->SolveQP(x_comp_integral, pos, v_world, ori, w_world, r, yaw, weights, traj_all_, alpha, gravity_, mpcTable);

    auto const &solu = qpsolver_->GetSolution();
    for (int leg = 0; leg < 4; leg++)
    {
      SdVector3f f;
      for (int axis = 0; axis < 3; axis++)
        f[axis] = solu[leg * 3 + axis];

      out[leg] = f;
    }
  }

}

#include "wbc/wbc_ctrl.h"
#include "wbc/task/body_ori.h"
#include "wbc/task/body_pos.h"
#include "wbc/task/link_pos.h"

namespace sdrobot::wbc
{
  using Matrix18 = Eigen::Matrix<fpt_t, params::model::dim_config, params::model::dim_config>;

  WbcCtrl::WbcCtrl(
      model::FloatBaseModel::SharedPtr const &model,
      Options const &opts,
      double weight) : model_(model),
                       Kp_joint_(opts.kp_joint),
                       Kd_joint_(opts.kd_joint),
                       body_pos_task_(std::make_shared<TaskBodyPos>(model, opts.kp_body, opts.kd_body)),
                       body_ori_task_(std::make_shared<TaskBodyOri>(model, opts.kp_ori, opts.kd_ori)),
                       foot_task_({
                           std::make_shared<TaskLinkPos>(model, opts.kp_foot, opts.kd_foot, linkid::fr),
                           std::make_shared<TaskLinkPos>(model, opts.kp_foot, opts.kd_foot, linkid::fl),
                           std::make_shared<TaskLinkPos>(model, opts.kp_foot, opts.kd_foot, linkid::hr),
                           std::make_shared<TaskLinkPos>(model, opts.kp_foot, opts.kd_foot, linkid::hl),
                       }),
                       foot_contact_({
                           std::make_shared<Contact>(model, linkid::fr),
                           std::make_shared<Contact>(model, linkid::fl),
                           std::make_shared<Contact>(model, linkid::hr),
                           std::make_shared<Contact>(model, linkid::hl),
                       }),
                       kin_wbc_(std::make_unique<KinWbc>()),
                       wbic_(std::make_unique<Wbic>(weight))
  {
    full_config_.fill(0.);
  }

  void WbcCtrl::Run(InData &wbcdata,
                    leg::LegCtrl::SharedPtr &legctrl,
                    drive::DriveCtrl::SharedPtr const &drivectrl,
                    estimate::EstimateCtrl::SharedPtr const &estctrl)
  {

    // Update Model
    _UpdateModel(estctrl->GetEstState(), legctrl->GetDatas());

    // Task & Contact Update
    _ContactTaskUpdate(wbcdata);

    // WBC Computation
    _ComputeWBC();
    _UpdateLegCMD(legctrl, drivectrl);
  }

  bool WbcCtrl::_UpdateModel(estimate::State const &estdata, leg::Datas const &legdata)
  {
    model::FloatBaseModelState _state;
    _state.ori = estdata.ori;
    _state.pos = estdata.pos;

    for (int i = 0; i < params::model::num_leg_joint; i++)
    {
      _state.vel[i] = estdata.vel_rpy_body[i];
      _state.vel[i + 3] = estdata.vel_body[i];
      for (int leg = 0; leg < params::model::num_leg; leg++)
      {
        _state.q[3 * leg + i] = legdata[leg].q[i];
        _state.qd[3 * leg + i] = legdata[leg].qd[i];

        full_config_[3 * leg + i] = _state.q[3 * leg + i];
      }
    }
    model_->UpdateState(_state);
    model_->ComputeContactJacobians();
    model_->ComputeGeneralizedGravityForce();
    model_->ComputeGeneralizedCoriolisForce();
    model_->ComputeGeneralizedMassMatrix();
    return true;
  }

  bool WbcCtrl::_ComputeWBC()
  {
    // TEST
    kin_wbc_->FindConfiguration(full_config_, task_list_, contact_list_,
                                des_jpos_, des_jvel_);

    auto grav = model_->GetGeneralizedGravityForce();
    auto coriolis = model_->GetGeneralizedCoriolisForce();
    auto A = model_->GetMassMatrix();
    model::MassMatTp Ainv;

    Eigen::Map<Matrix18> _A(A.data());
    Eigen::Map<Matrix18> _Ainv(Ainv.data());
    _Ainv = _A.inverse();

    // WBIC
    wbic_->UpdateSetting(A, Ainv, coriolis, grav);
    wbic_->MakeTorque(tau_ff_, task_list_, contact_list_);
    return true;
  }

  bool WbcCtrl::_UpdateLegCMD(leg::LegCtrl::SharedPtr &legctrl, drive::DriveCtrl::SharedPtr const &drivectrl)
  {
    auto &cmds = legctrl->GetCmdsForUpdate();

    for (int leg(0); leg < params::model::num_leg; ++leg)
    {
      cmds[leg].Zero();
      for (int jidx(0); jidx < params::model::num_leg_joint; ++jidx)
      {
        cmds[leg].tau_feed_forward[jidx] = tau_ff_[params::model::num_leg_joint * leg + jidx];
        cmds[leg].q_des[jidx] = des_jpos_[params::model::num_leg_joint * leg + jidx];
        cmds[leg].qd_des[jidx] = des_jvel_[params::model::num_leg_joint * leg + jidx];

        ToEigenTp(cmds[leg].kp_joint)(jidx, jidx) = Kp_joint_[jidx];
        ToEigenTp(cmds[leg].kd_joint)(jidx, jidx) = Kd_joint_[jidx];

        auto gait = drivectrl->GetGait();

        if (gait == drive::Gait::Bound)
        {
          cmds[leg].tau_feed_forward[jidx] *= 0.8;
        }
      }
    }

    // Knee joint non flip barrier
    for (int leg(0); leg < params::model::num_leg; ++leg)
    {
      if (cmds[leg].q_des[2] < 0.3)
      {
        cmds[leg].q_des[2] = 0.3;
      }
      if (legctrl->GetDatas()[leg].q[2] < 0.3)
      {
        auto knee_pos = legctrl->GetDatas()[leg].q[2];
        cmds[leg].tau_feed_forward[2] = 1. / (knee_pos * knee_pos + 0.02);
      }
    }
    return true;
  }

  bool WbcCtrl::_ContactTaskUpdate(InData const &wbcdata)
  {
    // Wash out the previous setup
    _CleanUp();

    body_ori_task_->UpdateTask(
        wbcdata.pos_rpy_body_des,
        wbcdata.vel_rpy_body_des,
        SdVector3f{});
    body_pos_task_->UpdateTask(
        wbcdata.pos_body_des,
        wbcdata.vel_body_des,
        wbcdata.acc_body_des);

    task_list_.push_back(body_ori_task_);
    task_list_.push_back(body_pos_task_);

    for (int leg(0); leg < params::model::num_leg; ++leg)
    {
      if (wbcdata.contact_state[leg] > 0.)
      { // Contact
        foot_contact_[leg]->UpdateRFdes(wbcdata.Fr_des[leg]);
        foot_contact_[leg]->UpdateContact();
        contact_list_.push_back(foot_contact_[leg]);
      }
      else
      { // No Contact (swing)
        foot_task_[leg]->UpdateTask(
            wbcdata.pos_foot_des[leg],
            wbcdata.vel_foot_des[leg],
            wbcdata.acc_foot_des[leg]);
        //zero_vec3);
        task_list_.push_back(foot_task_[leg]);
      }
    }
    return true;
  }

  bool WbcCtrl::_CleanUp()
  {
    contact_list_.clear();
    task_list_.clear();
    return true;
  }

} // namespace sdrobot::wbc

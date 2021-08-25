#include "wbc/wbc_ctrl.h"

#include "wbc/task/body_ori.h"
#include "wbc/task/body_pos.h"
#include "wbc/task/link_pos.h"

namespace sdrobot::wbc {
using Matrix18 =
    Eigen::Matrix<fpt_t, consts::model::kDimConfig, consts::model::kDimConfig>;

WbcCtrl::WbcCtrl(model::FloatBaseModel::SharedPtr const &model,
                 Options const &opts, double weight)
    : model_(model),
      Kp_joint_(opts.kp_joint),
      Kd_joint_(opts.kd_joint),
      body_pos_task_(
          std::make_shared<TaskBodyPos>(model, opts.kp_body, opts.kd_body)),
      body_ori_task_(
          std::make_shared<TaskBodyOri>(model, opts.kp_ori, opts.kd_ori)),
      foot_task_({
          std::make_shared<TaskLinkPos>(model, opts.kp_foot, opts.kd_foot,
                                        linkid::fr),
          std::make_shared<TaskLinkPos>(model, opts.kp_foot, opts.kd_foot,
                                        linkid::fl),
          std::make_shared<TaskLinkPos>(model, opts.kp_foot, opts.kd_foot,
                                        linkid::hr),
          std::make_shared<TaskLinkPos>(model, opts.kp_foot, opts.kd_foot,
                                        linkid::hl),
      }),
      foot_contact_({
          std::make_shared<Contact>(model, linkid::fr),
          std::make_shared<Contact>(model, linkid::fl),
          std::make_shared<Contact>(model, linkid::hr),
          std::make_shared<Contact>(model, linkid::hl),
      }),
      kin_wbc_(std::make_unique<KinWbc>()),
      wbic_(std::make_unique<Wbic>(weight)) {
  full_config_.fill(0.);
}

bool WbcCtrl::RunOnce(InData &wbcdata, leg::LegCtrl::SharedPtr const &legctrl,
                      drive::DriveCtrl::ConstSharedPtr const &drivectrl,
                      estimate::EstimateCtrl::ConstSharedPtr const &estctrl) {
  // Update Model
  _UpdateModel(estctrl->GetEstState(), legctrl->GetDatas());

  // Task & Contact Update
  _ContactTaskUpdate(wbcdata);

  // WBC Computation
  _ComputeWBC();
  return _UpdateLegCMD(legctrl, drivectrl);
}

bool WbcCtrl::_UpdateModel(estimate::State const &estdata,
                           leg::Datas const &legdata) {
  model::FloatBaseModelState _state;
  _state.ori = estdata.ori;
  _state.pos = estdata.pos;

  for (int i = 0; i < consts::model::kNumLegJoint; i++) {
    _state.gvel_robot[i] = estdata.avel_robot[i];
    _state.gvel_robot[i + 3] = estdata.lvel_robot[i];
    for (int leg = 0; leg < consts::model::kNumLeg; leg++) {
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

bool WbcCtrl::_ComputeWBC() {
  // TEST
  kin_wbc_->FindConfiguration(des_jpos_, des_jvel_, full_config_, task_list_,
                              contact_list_);

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

bool WbcCtrl::_UpdateLegCMD(leg::LegCtrl::SharedPtr const &legctrl,
                            drive::DriveCtrl::ConstSharedPtr const &drivectrl) {
  auto &cmds = legctrl->GetCmdsForUpdate();

  for (int leg(0); leg < consts::model::kNumLeg; ++leg) {
    for (int jidx(0); jidx < consts::model::kNumLegJoint; ++jidx) {
      cmds[leg].tau_feed_forward[jidx] =
          tau_ff_[consts::model::kNumLegJoint * leg + jidx];
      cmds[leg].q_des[jidx] =
          des_jpos_[consts::model::kNumLegJoint * leg + jidx];
      cmds[leg].qd_des[jidx] =
          des_jvel_[consts::model::kNumLegJoint * leg + jidx];

      ToEigenTp(cmds[leg].kp_joint)(jidx, jidx) = Kp_joint_[jidx];
      ToEigenTp(cmds[leg].kd_joint)(jidx, jidx) = Kd_joint_[jidx];

      auto gait = drivectrl->GetGait();

      if (gait == drive::Gait::Bound) {
        cmds[leg].tau_feed_forward[jidx] *= 0.8;
      }
    }
  }

  // Knee joint non flip barrier
  for (int leg(0); leg < consts::model::kNumLeg; ++leg) {
    if (cmds[leg].q_des[2] < 0.3) {
      cmds[leg].q_des[2] = 0.3;
    }
    if (legctrl->GetDatas()[leg].q[2] < 0.3) {
      auto knee_pos = legctrl->GetDatas()[leg].q[2];
      cmds[leg].tau_feed_forward[2] = 1. / (knee_pos * knee_pos + 0.02);
    }
  }
  return true;
}

bool WbcCtrl::_ContactTaskUpdate(InData const &wbcdata) {
  // Wash out the previous setup
  _CleanUp();

  body_ori_task_->UpdateTask(wbcdata.body_rpy_des, wbcdata.body_avel_des,
                             SdVector3f{});
  body_pos_task_->UpdateTask(wbcdata.body_pos_des, wbcdata.body_lvel_des,
                             wbcdata.body_acc_des);

  task_list_.push_back(body_ori_task_);
  task_list_.push_back(body_pos_task_);

  for (int leg(0); leg < consts::model::kNumLeg; ++leg) {
    if (wbcdata.contact_state[leg] > 0.) {  // Contact
      foot_contact_[leg]->UpdateRFdes(wbcdata.Fr_des[leg]);
      foot_contact_[leg]->UpdateContact();
      contact_list_.push_back(foot_contact_[leg]);
    } else {  // No Contact (swing)
      foot_task_[leg]->UpdateTask(wbcdata.foot_pos_des[leg],
                                  wbcdata.foot_lvel_des[leg],
                                  wbcdata.foot_acc_des[leg]);
      // zero_vec3);
      task_list_.push_back(foot_task_[leg]);
    }
  }
  return true;
}

bool WbcCtrl::_CleanUp() {
  contact_list_.clear();
  task_list_.clear();
  return true;
}

}  // namespace sdrobot::wbc

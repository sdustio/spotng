#pragma once
#include "sdrobot/options.h"
#include "sdrobot/drive.h"
#include "sdrobot/estimate.h"
#include "sdrobot/leg.h"
#include "sdrobot/model.h"
#include "wbc/wbic.h"
#include "wbc/kin_wbc.h"

namespace sdrobot::wbc
{
  namespace linkid
  {
    constexpr inline int const fr = 9;  // Front Right Foot
    constexpr inline int const fl = 11; // Front Left Foot
    constexpr inline int const hr = 13; // Hind Right Foot
    constexpr inline int const hl = 15; // Hind Left Foot

    constexpr inline int const fr_abd = 2; // Front Right Abduction
    constexpr inline int const fl_abd = 0; // Front Left Abduction
    constexpr inline int const hr_abd = 3; // Hind Right Abduction
    constexpr inline int const hl_abd = 1; // Hind Left Abduction
  }

  class WbcCtrl
  {
  public:
    using Ptr = std::unique_ptr<WbcCtrl>;
    using SharedPtr = std::shared_ptr<WbcCtrl>;

    WbcCtrl(model::FloatBaseModel::SharedPtr const &model,
            Options const &opts, double weight = 0.1);
    void Run(InData &wbcdata,
             leg::LegCtrl::SharedPtr &legctrl,
             drive::DriveCtrl::SharedPtr const &drivectrl,
             estimate::EstimateCtrl::SharedPtr const &estctrl);

  private:
    bool _UpdateModel(estimate::State const &estdata, leg::Datas const &legdata);
    bool _ComputeWBC();
    bool _UpdateLegCMD(leg::LegCtrl::SharedPtr &legctrl, drive::DriveCtrl::SharedPtr const &drivectrl);
    bool _ContactTaskUpdate(InData const &wbcdata);
    bool _CleanUp();

    model::FloatBaseModel::SharedPtr model_;

    std::vector<Task::SharedPtr> task_list_;
    std::vector<Contact::SharedPtr> contact_list_;

    std::array<fpt_t, params::model::num_leg_joint> Kp_joint_, Kd_joint_;

    Task::SharedPtr body_pos_task_;
    Task::SharedPtr body_ori_task_;

    std::array<Task::SharedPtr, params::model::num_leg> foot_task_;
    std::array<Contact::SharedPtr, params::model::num_leg> foot_contact_;

    std::array<SdVector3f, params::model::num_leg> pre_foot_vel_;
    std::array<SdVector3f, params::model::num_leg> Fr_result_;

    SdVector12f full_config_;
    SdVector12f tau_ff_;
    SdVector12f des_jpos_;
    SdVector12f des_jvel_;

    SdVector4f quat_des_;

    KinWbc::Ptr kin_wbc_;
    Wbic::Ptr wbic_;
  };

} // namespace sdrobot::wbc

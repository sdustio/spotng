#pragma once

#include <memory>
#include <vector>

#include "sdquadx/drive.h"
#include "sdquadx/estimate.h"
#include "sdquadx/leg.h"
#include "sdquadx/model.h"
#include "sdquadx/options.h"
#include "wbc/kin_wbc.h"
#include "wbc/wbic.h"

namespace sdquadx::wbc {
namespace linkid {
constexpr inline int const fr = 9;   // Front Right Foot
constexpr inline int const fl = 11;  // Front Left Foot
constexpr inline int const hr = 13;  // Hind Right Foot
constexpr inline int const hl = 15;  // Hind Left Foot

constexpr inline int const fr_abd = 2;  // Front Right Abduction
constexpr inline int const fl_abd = 0;  // Front Left Abduction
constexpr inline int const hr_abd = 3;  // Hind Right Abduction
constexpr inline int const hl_abd = 1;  // Hind Left Abduction
}  // namespace linkid

class WbcCtrl {
 public:
  using Ptr = std::unique_ptr<WbcCtrl>;
  using SharedPtr = std::shared_ptr<WbcCtrl>;
  using ConstSharedPtr = std::shared_ptr<WbcCtrl const>;

  WbcCtrl(model::FloatBaseModel::SharedPtr const &model, Options const &opts, double weight = 0.1);
  bool RunOnce(InData &wbcdata, leg::LegCtrl::SharedPtr const &legctrl,
               drive::DriveCtrl::ConstSharedPtr const &drivectrl,
               estimate::EstimateCtrl::ConstSharedPtr const &estctrl);

 private:
  bool _UpdateModel(estimate::State const &estdata, leg::Datas const &legdata);
  bool _ComputeWBC();
  bool _UpdateLegCMD(leg::LegCtrl::SharedPtr const &legctrl, drive::DriveCtrl::ConstSharedPtr const &drivectrl);
  bool _ContactTaskUpdate(InData const &wbcdata);
  bool _CleanUp();

  model::FloatBaseModel::SharedPtr model_;

  std::vector<Task::ConstSharedPtr> task_list_;
  std::vector<Contact::ConstSharedPtr> contact_list_;

  std::array<fpt_t, consts::model::kNumLegJoint> Kp_joint_, Kd_joint_;

  Task::SharedPtr body_pos_task_;
  Task::SharedPtr body_ori_task_;

  std::array<Task::SharedPtr, consts::model::kNumLeg> foot_task_;
  std::array<Contact::SharedPtr, consts::model::kNumLeg> foot_contact_;

  std::array<SdVector3f, consts::model::kNumLeg> pre_foot_vel_;
  std::array<SdVector3f, consts::model::kNumLeg> Fr_result_;

  SdVector12f full_config_;
  SdVector12f tau_ff_;
  SdVector12f des_jpos_;
  SdVector12f des_jvel_;

  SdVector4f quat_des_;

  KinWbc::Ptr kin_wbc_;
  Wbic::Ptr wbic_;
};

}  // namespace sdquadx::wbc

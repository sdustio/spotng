#pragma once

#include <memory>

#include "sdengine/consts.h"
#include "sdengine/drive.h"
#include "skd/foot_swing.h"
#include "skd/gait.h"
#include "wbc/wbc.h"

namespace sdengine::skd {

// 质心和落足点规划

class StateDes {
 public:
  using Ptr = std::unique_ptr<StateDes>;
  using SharedPtr = std::shared_ptr<StateDes>;
  using ConstSharedPtr = std::shared_ptr<StateDes const>;

  explicit StateDes(Options::ConstSharedPtr const &opts);
  bool RunOnce(wbc::InData &des, estimate::State const &estdata, drive::DriveCtrl::ConstSharedPtr const &drivectrl,
               Gait::ConstSharedPtr const &gait_skd);

 private:
  Options::ConstSharedPtr const opts_;

  std::array<FootSwingTrajectory, consts::model::kNumLeg> foot_swing_trajs_;
  std::array<bool, consts::model::kNumLeg> first_swing_;
  SdVector4f swing_times_;
  SdVector4f swing_time_remaining_;

  SdVector3f pos_des_ = {};

  drive::Gait current_gait_;
  bool first_run_ = true;
};
}  // namespace sdengine::skd

#pragma once

#include "sdengine/drive.h"
#include "sdengine/options.h"

namespace sdengine::drive {
class DriveCtrlImpl : public DriveCtrl {
 public:
  explicit DriveCtrlImpl(Options::ConstSharedPtr const &opts);

  bool UpdateMode(Mode const &mode) override;
  bool UpdateTwist(Twist const &twist) override;
  bool UpdatePose(Pose const &pose) override;
  bool UpdateState(State const &state) override;
  bool UpdateGait(Gait const &gait) override;
  bool UpdateStepHeight(fpt_t const height) override;

  bool CmdtoDesData() override;

  fpt_t GetStepHeight() const override;
  State GetState() const override;
  Gait GetGait() const override;

  SdVector3f const &GetPosDes() const override;
  SdVector3f const &GetRpyDes() const override;
  SdVector3f const &GetLvelDes() const override;
  SdVector3f const &GetAvelDes() const override;

 private:
  bool ZeroVel();

  Options::ConstSharedPtr const opts_;
  Twist twist_;
  Pose pose_;

  fpt_t step_height_ = 0.1;

  Mode mode_ = Mode::Auto;
  State state_ = State::Init;
  Gait gait_ = Gait::Trot;
  SdVector3f pos_ = {};
  SdVector3f rpy_ = {};
  SdVector3f lvel_ = {};
  SdVector3f avel_ = {};
};

}  // namespace sdengine::drive

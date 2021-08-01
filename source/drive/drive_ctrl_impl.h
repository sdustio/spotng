#pragma once

#include "sdrobot/drive.h"

namespace sdrobot::drive
{
  class DriveCtrlImpl : public DriveCtrl
  {
  public:
    explicit DriveCtrlImpl(fptype dt);
    DriveCtrlImpl(DriveMode mode, fptype dt);

    bool UpdateDriveCmd(DriveCmd const &cmd) override;
    bool CmdtoDesData() override;

    fptype GetDuration() const override;
    fptype GetStepHeight() const override;
    State GetState() const override;
    Gait GetGait() const override;

    SdVector3f const &GetPosDes() const override;
    SdVector3f const &GetPosRpyDes() const override;
    SdVector3f const &GetVelDes() const override;
    SdVector3f const &GetVelRpyDes() const override;

  private:
    fptype Deadband(fptype v, fptype minVal, fptype maxVal);

    DriveMode mode_;
    fptype dt_;
    DriveCmd cmd_;

    fptype step_height_ = 0.1;
    State state_ = State::Init;
    Gait gait_ = Gait::Trot;
    SdVector3f pos_ = {};
    SdVector3f pos_rpy_ = {};
    SdVector3f vel_ = {};
    SdVector3f vel_rpy_ = {};
  };

}

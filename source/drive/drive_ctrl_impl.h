#pragma once

#include "sdrobot/drive.h"

namespace sdrobot::drive
{
  class DriveCtrlImpl : public DriveCtrl
  {
  public:
    explicit DriveCtrlImpl(fpt_t dt);
    DriveCtrlImpl(DriveMode mode, fpt_t dt);

    bool UpdateDriveCmd(DriveCmd const &cmd) override;
    bool CmdtoDesData() override;

    fpt_t GetDuration() const override;
    fpt_t GetStepHeight() const override;
    State GetState() const override;
    Gait GetGait() const override;

    SdVector3f const &GetPosDes() const override;
    SdVector3f const &GetPosRpyDes() const override;
    SdVector3f const &GetVelDes() const override;
    SdVector3f const &GetVelRpyDes() const override;

  private:
    fpt_t Deadband(fpt_t v, fpt_t minVal, fpt_t maxVal);

    DriveMode mode_;
    fpt_t dt_;
    DriveCmd cmd_;

    fpt_t step_height_ = 0.1;
    State state_ = State::Init;
    Gait gait_ = Gait::Trot;
    SdVector3f pos_ = {};
    SdVector3f pos_rpy_ = {};
    SdVector3f vel_ = {};
    SdVector3f vel_rpy_ = {};
  };

}

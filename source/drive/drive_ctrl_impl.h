#pragma once

#include "sdrobot/drive.h"

namespace sdrobot::drive
{
  class DriveCtrlImpl : public DriveCtrl
  {
  public:
    DriveCtrlImpl(DriveMode mode, double dt);

    bool UpdateDriveCmd(DriveCmd const &cmd) override;
    bool CmdtoDesData() override;

    double GetDuration() const override;
    double GetStepHeight() const override;
    State GetState() const override;
    Gait GetGait() const override;

    SdArray3f const &GetPosDes() const override;
    SdArray3f const &GetPosRpyDes() const override;
    SdArray3f const &GetVelDes() const override;
    SdArray3f const &GetVelRpyDes() const override;

  private:
    double Deadband(double v, double minVal, double maxVal);

    DriveMode mode_;
    double dt_;
    DriveCmd cmd_;

    double step_height_ = 0.1;
    State state_ = State::Init;
    Gait gait_ = Gait::Trot;
    SdArray3f pos_ = {};
    SdArray3f pos_rpy_ = {};
    SdArray3f vel_ = {};
    SdArray3f vel_rpy_ = {};
  };

}

#pragma once

#include "sdrobot/drive.h"

namespace sdrobot::drive
{
  class DriveCtrlImpl : public DriveCtrl
  {
  public:
    explicit DriveCtrlImpl(double dt);
    DriveCtrlImpl(DriveMode mode, double dt);

    bool UpdateDriveCmd(DriveCmd const &cmd) override;
    bool CmdtoDesData() override;

    double GetDuration() const override;
    double GetStepHeight() const override;
    State GetState() const override;
    Gait GetGait() const override;

    SdVector3f const &GetPosDes() const override;
    SdVector3f const &GetPosRpyDes() const override;
    SdVector3f const &GetVelDes() const override;
    SdVector3f const &GetVelRpyDes() const override;

  private:
    double Deadband(double v, double minVal, double maxVal);

    DriveMode mode_;
    double dt_;
    DriveCmd cmd_;

    double step_height_ = 0.1;
    State state_ = State::Init;
    Gait gait_ = Gait::Trot;
    SdVector3f pos_ = {};
    SdVector3f pos_rpy_ = {};
    SdVector3f vel_ = {};
    SdVector3f vel_rpy_ = {};
  };

}

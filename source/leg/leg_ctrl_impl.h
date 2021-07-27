#pragma once

#include "sdrobot/leg.h"

namespace sdrobot::leg
{
  class LegCtrlImpl : public LegCtrl
  {
  public:
    LegCtrlImpl(interface::ActuatorInterface::SharedPtr const &act_itf);

    const Datas &GetDatas() const override;

    Cmds &GetCmdsForUpdate() override;
    bool UpdateCmds(Cmds const &cmds) override;
    bool UpdateCmd(int leg, Cmd const &cmd) override;

    void ZeroCmd() override;

    void ComputeLegJacobianAndPosition(int leg) override;

    void UpdateDatasFromActuatorInterface() override;

    void SendCmdsToActuatorInterface() override;

  private:
    Datas datas_;
    Cmds cmds_;
    interface::ActuatorInterface::SharedPtr act_itf_;
  };

}

#pragma once

#include "sdrobot/leg.h"

namespace sdrobot::leg {
class LegCtrlImpl : public LegCtrl {
 public:
  explicit LegCtrlImpl(interface::ActuatorInterface::SharedPtr const &act_itf);

  Datas const &GetDatas() const override;

  Cmds &GetCmdsForUpdate() override;
  bool UpdateCmds(Cmds const &cmds) override;
  bool UpdateCmd(int leg, Cmd const &cmd) override;

  void ZeroCmd() override;

  bool ComputeLegJacobianAndPosition(int leg) override;

  bool UpdateDatasFromActuatorInterface() override;

  bool SendCmdsToActuatorInterface() override;

 private:
  Datas datas_;
  Cmds cmds_;
  interface::ActuatorInterface::SharedPtr act_itf_;
};

}  // namespace sdrobot::leg

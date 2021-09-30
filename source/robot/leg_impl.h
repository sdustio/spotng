#pragma once

#include "sdquadx/leg.h"
#include "sdquadx/options.h"

namespace sdquadx::leg {
class LegCtrlImpl : public LegCtrl {
 public:
  LegCtrlImpl(interface::ActuatorInterface::SharedPtr const &act_itf, Options::ConstSharedPtr const &opts);

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
  Options::ConstSharedPtr opts_;
  unsigned iter_ = 0;
};

}  // namespace sdquadx::leg

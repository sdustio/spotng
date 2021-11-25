#include "fsm/legctrl.h"

namespace sdquadx::fsm {
LegCtrl::LegCtrl(interface::Leg::SharedPtr const &itf) : itf_(itf) {}

bool LegCtrl::RunOnce() { return itf_->ReceiveLegCmds(cmds); }

void LegCtrl::ZeroCmds() {
  for (auto &cmd : cmds) {
    cmd.tau.fill(0.);
    cmd.q_des.fill(0.);
    cmd.qd_des.fill(0.);
    cmd.kp_joint.fill(0.);
    cmd.kd_joint.fill(0.);
  }
}
}  // namespace sdquadx::fsm

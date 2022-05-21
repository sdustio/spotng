#include "fsm/legctrl.h"

#include "spdlog/spdlog.h"

#ifdef DEBUG_MODE
#include "utils/debug.h"
#endif

namespace forax::fsm {
LegCtrl::LegCtrl(interface::Leg::SharedPtr const &itf) : itf_(itf) {}

bool LegCtrl::RunOnce() {
#ifdef DEBUG_MODE
  spdlog::debug("!!![Leg Cmds]");
  for (int i = 0; i < consts::model::kNumLeg; i++) {
    DebugVector("q of leg " + std::to_string(i), cmds[i].q_des);
    DebugVector("qd of leg " + std::to_string(i), cmds[i].qd_des);
    DebugVector("tau of leg " + std::to_string(i), cmds[i].tau);
  }
#endif
  return itf_->WriteFrom(cmds);
}

void LegCtrl::ZeroCmds() {
  for (auto &cmd : cmds) {
    cmd.tau.fill(0.);
    cmd.q_des.fill(0.);
    cmd.qd_des.fill(0.);
    cmd.kp.fill(0.);
    cmd.kd.fill(0.);
  }
}
}  // namespace forax::fsm

#include "estimate/joints.h"

namespace sdquadx::estimate {
Joints::Joints(interface::Leg::ConstSharedPtr const &itf) : itf_(itf) {}

bool Joints::RunOnce(State &ret) {
  for (size_t leg = 0; leg < consts::model::kNumLeg; leg++) {
    itf_->ReadTo(legdatas_[leg], leg);
    ret.q[leg] = legdatas_[leg].q;
    ret.qd[leg] = legdatas_[leg].qd;
  }
  return true;
}
}  // namespace sdquadx::estimate

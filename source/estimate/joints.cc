#include "estimate/joints.h"

namespace sdquadx::estimate {
Joints::Joints(interface::Leg::ConstSharedPtr const &itf) : itf_(itf) {}

bool Joints::RunOnce(State &ret) {
  itf_->ReadTo(legdatas_);
  for (size_t leg = 0; leg < consts::model::kNumLeg; leg++) {
    ret.q[leg] = legdatas_[leg].q;
    ret.qd[leg] = legdatas_[leg].qd;
  }
  return true;
}
}  // namespace sdquadx::estimate

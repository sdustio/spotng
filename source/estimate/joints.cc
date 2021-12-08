#include "estimate/joints.h"

#include "math/utils.h"

namespace sdquadx::estimate {
Joints::Joints(interface::Leg::ConstSharedPtr const &itf) : itf_(itf) {}

bool Joints::RunOnce(State &ret) {
  if (!InterfaceValid()) {
    ret.success = false;
    return false;
  }

  for (size_t leg = 0; leg < consts::model::kNumLeg; leg++) {
    ret.q[leg] = legdatas_[leg].q;
    ret.qd[leg] = legdatas_[leg].qd;
  }
  return true;
}
bool Joints::InterfaceValid() {
  itf_->ReadTo(legdatas_);
  for (size_t leg = 0; leg < consts::model::kNumLeg; leg++) {
    if (math::HasNaN(legdatas_[leg].q.cbegin(), legdatas_[leg].q.cend())) return false;
    if (math::HasNaN(legdatas_[leg].qd.cbegin(), legdatas_[leg].qd.cend())) return false;
  }
  return true;
}
}  // namespace sdquadx::estimate

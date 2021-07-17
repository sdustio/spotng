#include "controllers/mpc/gait.h"

namespace sdrobot::ctrl::mpc
{
  OffsetDurationGait::OffsetDurationGait(unsigned nSegment, Eigen::Vector4i offset, Eigen::Vector4i durations, const std::string &name)
  {
  }
  Vector4 OffsetDurationGait::getContactState() { return Vector4(); }

  Vector4 OffsetDurationGait::getSwingState() { return Vector4(); }

  const std::vector<int> &OffsetDurationGait::getMpcTable() { return _mpc_table; }

  void OffsetDurationGait::setIterations([[maybe_unused]] unsigned iterationsBetweenMPC, [[maybe_unused]] unsigned currentIteration) { return; }

  double OffsetDurationGait::getCurrentStanceTime(double dtMPC, size_t leg) { return dtMPC + leg; }

  double OffsetDurationGait::getCurrentSwingTime(double dtMPC, size_t leg) { return dtMPC + leg; }

  unsigned OffsetDurationGait::getCurrentGaitPhase() { return 0; }
}

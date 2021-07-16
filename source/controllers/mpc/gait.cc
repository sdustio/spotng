#include "controllers/mpc/gait.h"

namespace sdrobot::ctrl::mpc
{
  OffsetDurationGait::OffsetDurationGait(int nSegment, Eigen::Vector4i offset, Eigen::Vector4i durations, const std::string &name)
  {
  }
  Vector4 OffsetDurationGait::getContactState() { return Vector4(); }

  Vector4 OffsetDurationGait::getSwingState() { return Vector4(); }

  const std::vector<int> &OffsetDurationGait::getMpcTable() { return _mpc_table; }

  void OffsetDurationGait::setIterations([[maybe_unused]] int iterationsBetweenMPC, [[maybe_unused]] int currentIteration) { return; }

  double OffsetDurationGait::getCurrentStanceTime(double dtMPC, int leg) { return dtMPC + leg; }

  double OffsetDurationGait::getCurrentSwingTime(double dtMPC, int leg) { return dtMPC + leg; }

  int OffsetDurationGait::getCurrentGaitPhase() { return 0; }
}

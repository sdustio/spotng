#include "estimate/impl.h"

#include <string>

#include "estimate/orientation.h"
#include "estimate/pos_vel.h"
#include "spdlog/spdlog.h"

#ifdef DEBUG_MODE
#include "utils/debug.h"
#endif

namespace sdquadx::estimate {
EstimateCtrlImpl::EstimateCtrlImpl() {}

bool EstimateCtrlImpl::AddEstimator(std::string const &name, Estimator::SharedPtr const &est) {
  if (ests_map_.find(name) != ests_map_.end()) return false;
  ests_.push_back(est);
  ests_map_[name] = num_ests_;
  num_ests_++;
  return true;
}

Estimator::SharedPtr const &EstimateCtrlImpl::GetEstimator(std::string const &name) const {
  auto iter = ests_map_.find(name);
  if (iter == ests_map_.end()) return null_est_;
  return ests_[iter->second];
}

bool EstimateCtrlImpl::RunOnce() {
  bool ret = true;
  for (auto const &est : ests_) {
    ret = ret && est->RunOnce(est_state_);
  }

#ifdef DEBUG_MODE
  spdlog::debug("estimated state");
  for (int i = 0; i < consts::model::kNumLeg; i++) {
    DebugVector("q of leg " + std::to_string(i), est_state_.q[i]);
    DebugVector("qd of leg " + std::to_string(i), est_state_.qd[i]);
  }
  DebugVector("Contact", est_state_.contact);
  DebugVector("Rpy", est_state_.rpy);
  DebugVector("Gyro", est_state_.avel_robot);
  DebugVector("Acc", est_state_.acc_robot);
  DebugVector("Pos", est_state_.pos);
  DebugVector("Vel", est_state_.lvel);
#endif
  return ret;
}

State const &EstimateCtrlImpl::GetEstState() const { return est_state_; }

bool EstimateCtrlImpl::RemoveEstimator(std::string const &name) {
  auto iter = ests_map_.find(name);
  if (iter == ests_map_.end()) return false;
  ests_.erase(ests_.begin() + iter->second);
  return ests_map_.erase(name);
}

bool EstimateCtrlImpl::RemoveAllEstimators() {
  ests_.clear();
  ests_map_.clear();
  return true;
}
}  // namespace sdquadx::estimate

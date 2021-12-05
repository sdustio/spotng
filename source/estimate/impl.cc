#include "estimate/impl.h"

#include <string>

#include "estimate/orientation.h"
#include "estimate/pos_vel.h"

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

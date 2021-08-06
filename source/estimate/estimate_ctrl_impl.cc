#include "estimate/estimate_ctrl_impl.h"
#include "estimate/pos_vel.h"
#include "estimate/orientation.h"

namespace sdrobot::estimate
{

  bool EstimateCtrlImpl::AddEstimator(std::string const &name, Estimator::SharedPtr const &est)
  {
    if (est_map_.find(name) != est_map_.end())
      return false;
    est_map_[name] = est;
    return true;
  }

  Estimator::SharedPtr const &EstimateCtrlImpl::GetEstimator(std::string const &name)
  {
    auto iter = est_map_.find(name);
    if (iter == est_map_.end())
      return est_map_[""];
    return iter->second;
  }

  bool EstimateCtrlImpl::RunOnce()
  {
    bool ret = true;
    for (auto const &mp : est_map_)
    {
      ret = ret && mp.second->RunOnce(est_state_);
    }
    return ret;
  }

  State const &EstimateCtrlImpl::GetEstState() const
  {
    return est_state_;
  }

  bool EstimateCtrlImpl::RemoveEstimator(std::string const &name)
  {
    return est_map_.erase(name);
  }

  bool EstimateCtrlImpl::RemoveAllEstimators()
  {
    est_map_.clear();
    return true;
  }
}

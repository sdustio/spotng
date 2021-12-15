#pragma once


#include <unordered_map>
#include <vector>

#include "sdquadx/estimate.h"

namespace sdquadx::estimate {

class EstimateCtrlImpl : public EstimateCtrl {
 public:
  EstimateCtrlImpl() = default;
  bool AddEstimator(char const *name, Estimator::SharedPtr const &est) override;
  Estimator::SharedPtr const &GetEstimator(char const *name) const override;

  bool RunOnce() override;

  State const &GetEstState() const override;

  bool RemoveEstimator(char const *name) override;
  bool RemoveAllEstimators() override;

 private:
  std::size_t num_ests_ = 0;
  std::vector<Estimator::SharedPtr> ests_;
  std::unordered_map<char const *, std::size_t> ests_map_;
  State est_state_;
  Estimator::SharedPtr const null_est_;
};
}  // namespace sdquadx::estimate

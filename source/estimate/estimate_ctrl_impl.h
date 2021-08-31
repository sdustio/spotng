#pragma once

#include <string>
#include <unordered_map>

#include "sdquadx/estimate.h"

namespace sdquadx::estimate {

class EstimateCtrlImpl : public EstimateCtrl {
 public:
  EstimateCtrlImpl();
  bool AddEstimator(std::string const &name, Estimator::SharedPtr const &est) override;
  Estimator::SharedPtr const &GetEstimator(std::string const &name) const override;

  bool RunOnce() override;

  State const &GetEstState() const override;

  bool RemoveEstimator(std::string const &name) override;
  bool RemoveAllEstimators() override;

 private:
  std::unordered_map<std::string, Estimator::SharedPtr> est_map_;
  State est_state_;
  Estimator::SharedPtr const null_est_;
};
}  // namespace sdquadx::estimate

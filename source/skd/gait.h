#pragma once

#include <memory>
#include <string>
#include <vector>

#include "dynamics/types.h"
#include "sdquadx/consts.h"

namespace sdquadx::skd {

class Gait {
 public:
  using Ptr = std::unique_ptr<Gait>;
  using SharedPtr = std::shared_ptr<Gait>;
  using ConstSharedPtr = std::shared_ptr<Gait const>;

  virtual ~Gait() = default;

  virtual bool SetCurrentIter(int iter) = 0;
  virtual bool CalcStancePhase(SdVector4f &ret) const = 0;
  virtual bool CalcSwingPhase(SdVector4f &ret) const = 0;
  virtual fpt_t GetCurrentStanceTime(int leg) const = 0;
  virtual fpt_t GetCurrentSwingTime(int leg) const = 0;
  virtual std::vector<int> const &GetNextPeriodStanceStates() const = 0;
};
}  // namespace sdquadx::skd

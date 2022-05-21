#pragma once

#include <array>
#include <cstdint>
#include <memory>

#include "forax/consts.h"

namespace forax::skd {

using PredStanceVector = std::array<int, consts::ctrl::kPredLength * consts::model::kNumLeg>;

class Gait {
 public:
  using Ptr = std::unique_ptr<Gait>;
  using SharedPtr = std::shared_ptr<Gait>;
  using ConstSharedPtr = std::shared_ptr<Gait const>;

  virtual ~Gait() = default;

  virtual bool SetCurrentIter(std::int64_t iter) = 0;
  virtual bool CalcStancePhase(SdVector4f &ret) const = 0;
  virtual bool CalcSwingPhase(SdVector4f &ret) const = 0;
  virtual fpt_t GetCurrentStanceTime(int leg) const = 0;
  virtual fpt_t GetCurrentSwingTime(int leg) const = 0;
  virtual PredStanceVector const &GetNextPeriodStanceStates() const = 0;
};
}  // namespace forax::skd

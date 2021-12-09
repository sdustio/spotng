#pragma once

#include <string>

#include "skd/gait.h"

namespace sdquadx::skd {
using SdVector4i = std::array<int, consts::model::kNumLeg>;

class OffsetDurationGait : public Gait {
 public:
  OffsetDurationGait(int const period_iters, SdVector4i const &offsets, SdVector4i const &durations, fpt_t dt,
                     std::string const &name);

  bool SetCurrentIter(int iter) override;

  bool CalcStancePhase(SdVector4f &ret) const override;
  bool CalcSwingPhase(SdVector4f &ret) const override;
  fpt_t GetCurrentStanceTime(int leg) const override;
  fpt_t GetCurrentSwingTime(int leg) const override;
  PredStanceVector const &GetNextPeriodStanceStates() const override;

 private:
  int const iters_p_;
  SdVector4i const iters_o_;
  SdVector4i const iters_d_;
  fpt_t const dt_;
  std::string const name_;
  fpt_t stance_;
  fpt_t swing_;

  int iter_;
  SdVector4i progress_;
  PredStanceVector stance_states_ = {};
};
}  // namespace sdquadx::skd

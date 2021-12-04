#include "skd/od_gait.h"

namespace sdquadx::skd {

OffsetDurationGait::OffsetDurationGait(int const period_iters, SdVector4i const &offsets, SdVector4i const &durations,
                                       fpt_t dt, std::string const &name)
    : iters_p_(period_iters),
      iters_o_(offsets),
      iters_d_(durations),
      dt_(dt),
      name_(name),
      stance_states_(period_iters * 4) {
  stance_ = durations[0] * dt;
  swing_ = (period_iters - durations[0]) * dt;
}
bool OffsetDurationGait::SetCurrentIter(int iter) {
  iter_ = iter % iters_p_;

  for (auto j = 0; j < 4; j++) {
    progress_[j] = iter - iters_o_[j];
    if (progress_[j] < 0) progress_[j] += iters_p_;
  }
  // update mpc_table
  std::array<int, 4> progress;
  for (auto i = 0; i < iters_p_; i++) {
    int iter = (i + iter_ + 1) % iters_p_;
    for (auto j = 0; j < 4; j++) {
      progress[j] = iter - iters_o_[j];
      if (progress[j] < 0) progress[j] += iters_p_;
      if (progress[j] <= iters_d_[j])
        stance_states_[i * 4 + j] = 1;
      else
        stance_states_[i * 4 + j] = 0;
    }
  }
  return true;
}

bool OffsetDurationGait::CalcStancePhase(SdVector4f &ret) const {
  for (size_t i = 0; i < ret.size(); i++) {
    if (progress_[i] > iters_d_[i]) {
      ret[i] = 0.;
    } else {
      ret[i] = progress_[i] / iters_d_[i];
    }
  }
  return true;
}

bool OffsetDurationGait::CalcSwingPhase(SdVector4f &ret) const {
  for (size_t i = 0; i < ret.size(); i++) {
    if (progress_[i] > iters_d_[i]) {
      ret[i] = (progress_[i] - iters_d_[i]) / (iters_p_ - iters_d_[i]);
    } else {
      ret[i] = 0.;
    }
  }
  return true;
}

fpt_t OffsetDurationGait::GetCurrentStanceTime([[maybe_unused]] int leg) const { return stance_; }

fpt_t OffsetDurationGait::GetCurrentSwingTime([[maybe_unused]] int leg) const { return swing_; }

std::vector<int> const &OffsetDurationGait::GetNextPeriodStanceStates() const { return stance_states_; }
}  // namespace sdquadx::skd

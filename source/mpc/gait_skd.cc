#include "mpc/gait_skd.h"

namespace sdquadx::mpc {
OffsetDurationGait::OffsetDurationGait(int const nSegment, SdVector4i const &offsets, SdVector4i const &durations,
                                       std::string const &name)
    : offsets_(offsets), durations_(durations), n_iterations_(nSegment), name_(name), mpc_table_(nSegment * 4) {
  for (size_t i = 0; i < offsets_.size(); i++) {
    offsetsd_[i] = fpt_t(offsets_[i]) / fpt_t(nSegment);
  }

  for (size_t i = 0; i < offsets_.size(); i++) {
    durationsd_[i] = fpt_t(durations_[i]) / fpt_t(nSegment);
  }
  stance_ = durations[0];
  swing_ = nSegment - durations[0];
}

bool OffsetDurationGait::CalcContactState(SdVector4f &ret) const {
  for (size_t i = 0; i < ret.size(); i++) {
    ret[i] = phase_ - offsetsd_[i];
    if (ret[i] < 0) ret[i] += 1.;
    if (ret[i] > durationsd_[i]) {
      ret[i] = 0.;
    } else {
      ret[i] = ret[i] / durationsd_[i];
    }
  }
  return true;
}

bool OffsetDurationGait::CalcSwingState(SdVector4f &ret) const {
  for (size_t i = 0; i < ret.size(); i++) {
    auto o = offsetsd_[i] + durationsd_[i];
    if (o > 1) o -= 1.;
    auto d = 1. - durationsd_[i];
    ret[i] = phase_ - o;
    if (ret[i] < 0) ret[i] += 1.;
    if (ret[i] > d) {
      ret[i] = 0.;
    } else {
      ret[i] = ret[i] / d;
    }
  }

  // printf("swing state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1],
  // progress[2], progress[3]);
  return true;
}

std::vector<int> const &OffsetDurationGait::GetMpcTable() const { return mpc_table_; }

bool OffsetDurationGait::SetIterations(int iterationsPerMPC, int currentIteration) {
  iteration_ = (currentIteration / iterationsPerMPC) % n_iterations_;
  phase_ = (fpt_t)(currentIteration % (iterationsPerMPC * n_iterations_)) / (fpt_t)(iterationsPerMPC * n_iterations_);

  // update mpc_table
  for (int i = 0; i < n_iterations_; i++) {
    int iter = (i + iteration_ + 1) % n_iterations_;
    std::array<int, 4> progress;
    for (size_t j = 0; j < progress.size(); j++) {
      progress[j] = iter - offsets_[j];
      if (progress[j] < 0) progress[j] += n_iterations_;
      if (progress[j] < durations_[j])
        mpc_table_[i * 4 + j] = 1;
      else
        mpc_table_[i * 4 + j] = 0;
    }
  }
  return true;
}

fpt_t OffsetDurationGait::GetCurrentStanceTime(fpt_t dtMPC, [[maybe_unused]] int leg) const { return dtMPC * stance_; }

fpt_t OffsetDurationGait::GetCurrentSwingTime(fpt_t dtMPC, [[maybe_unused]] int leg) const { return dtMPC * swing_; }

int OffsetDurationGait::GetCurrentGaitPhase() const { return iteration_; }
}  // namespace sdquadx::mpc

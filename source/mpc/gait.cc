#include "mpc/gait.h"

namespace sdrobot::mpc
{

  using Eigen::Array4d;
  using Eigen::Array4i;

  OffsetDurationGait::OffsetDurationGait(
      int nSegment, Eigen::Vector4i offsets, Eigen::Vector4i durations,
      const std::string &name) : offsets_(offsets.array()),
                                 durations_(durations.array()),
                                 n_iterations_(nSegment),
                                 name_(name),
                                 mpc_table_(nSegment * 4)
  {
    offsetsd_ = offsets_.cast<double>() / double(nSegment);
    durationsd_ = durations_.cast<double>() / double(nSegment);

    stance_ = durations[0];
    swing_ = nSegment - durations[0];
  }

  Vector4 OffsetDurationGait::GetContactState()
  {
    Array4d progress = phase_ - offsetsd_;

    for (int i = 0; i < 4; i++)
    {
      if (progress[i] < 0)
        progress[i] += 1.;
      if (progress[i] > durationsd_[i])
      {
        progress[i] = 0.;
      }
      else
      {
        progress[i] = progress[i] / durationsd_[i];
      }
    }

    //printf("contact state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
    return progress.matrix();
  }

  Vector4 OffsetDurationGait::GetSwingState()
  {
    Array4d swing_offset = offsetsd_ + durationsd_;
    for (int i = 0; i < 4; i++)
      if (swing_offset[i] > 1)
        swing_offset[i] -= 1.;
    Array4d swing_duration = 1. - durationsd_;

    Array4d progress = phase_ - swing_offset;

    for (int i = 0; i < 4; i++)
    {
      if (progress[i] < 0)
        progress[i] += 1.f;
      if (progress[i] > swing_duration[i])
      {
        progress[i] = 0.;
      }
      else
      {
        progress[i] = progress[i] / swing_duration[i];
      }
    }

    //printf("swing state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
    return progress.matrix();
  }

  const Eigen::VectorXi &OffsetDurationGait::GetMpcTable()
  {

    //printf("MPC table:\n");
    for (int i = 0; i < n_iterations_; i++)
    {
      int iter = (i + iteration_ + 1) % n_iterations_;
      Array4i progress = iter - offsets_;
      for (int j = 0; j < 4; j++)
      {
        if (progress[j] < 0)
          progress[j] += n_iterations_;
        if (progress[j] < durations_[j])
          mpc_table_[i * 4 + j] = 1;
        else
          mpc_table_[i * 4 + j] = 0;
      }
    }

    return mpc_table_;
  }

  void OffsetDurationGait::SetIterations(int iterationsPerMPC, int currentIteration)
  {
    iteration_ = (currentIteration / iterationsPerMPC) % n_iterations_;
    phase_ = (double)(currentIteration % (iterationsPerMPC * n_iterations_)) / (double)(iterationsPerMPC * n_iterations_);
  }

  double OffsetDurationGait::GetCurrentStanceTime(double dtMPC, [[maybe_unused]] int leg) { return dtMPC * stance_; }

  double OffsetDurationGait::GetCurrentSwingTime(double dtMPC, [[maybe_unused]] int leg) { return dtMPC * swing_; }

  int OffsetDurationGait::GetCurrentGaitPhase() { return iteration_; }
}

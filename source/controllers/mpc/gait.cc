#include "controllers/mpc/gait.h"

namespace sdrobot::ctrl::mpc
{

  using Eigen::Array4d;
  using Eigen::Array4i;

  OffsetDurationGait::OffsetDurationGait(
      int nSegment, Eigen::Vector4i offsets, Eigen::Vector4i durations,
      const std::string &name) : _offsets(offsets.array()),
                                 _durations(durations.array()),
                                 _nIterations(nSegment),
                                 name_(name)
  {
    _mpc_table.resize(nSegment * 4);
    _offsetsd = _offsets.cast<double>() / double(nSegment);
    _durationsd = _durations.cast<double>() / double(nSegment);

    _stance = durations[0];
    _swing = nSegment - durations[0];
  }

  Vector4 OffsetDurationGait::getContactState()
  {
    Array4d progress = _phase - _offsetsd;

    for (int i = 0; i < 4; i++)
    {
      if (progress[i] < 0)
        progress[i] += 1.;
      if (progress[i] > _durationsd[i])
      {
        progress[i] = 0.;
      }
      else
      {
        progress[i] = progress[i] / _durationsd[i];
      }
    }

    //printf("contact state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
    return progress.matrix();
  }

  Vector4 OffsetDurationGait::getSwingState()
  {
    Array4d swing_offset = _offsetsd + _durationsd;
    for (int i = 0; i < 4; i++)
      if (swing_offset[i] > 1)
        swing_offset[i] -= 1.;
    Array4d swing_duration = 1. - _durationsd;

    Array4d progress = _phase - swing_offset;

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

  const std::vector<int> &OffsetDurationGait::getMpcTable()
  {

    //printf("MPC table:\n");
    for (int i = 0; i < _nIterations; i++)
    {
      int iter = (i + _iteration + 1) % _nIterations;
      Array4i progress = iter - _offsets;
      for (int j = 0; j < 4; j++)
      {
        if (progress[j] < 0)
          progress[j] += _nIterations;
        if (progress[j] < _durations[j])
          _mpc_table[i * 4 + j] = 1;
        else
          _mpc_table[i * 4 + j] = 0;
      }
    }

    return _mpc_table;
  }

  void OffsetDurationGait::setIterations(int iterationsPerMPC, int currentIteration)
  {
    _iteration = (currentIteration / iterationsPerMPC) % _nIterations;
    _phase = (double)(currentIteration % (iterationsPerMPC * _nIterations)) / (double)(iterationsPerMPC * _nIterations);
  }

  double OffsetDurationGait::getCurrentStanceTime(double dtMPC, [[maybe_unused]] int leg) { return dtMPC * _stance; }

  double OffsetDurationGait::getCurrentSwingTime(double dtMPC, [[maybe_unused]] int leg) { return dtMPC * _swing; }

  int OffsetDurationGait::getCurrentGaitPhase() { return _iteration; }
}

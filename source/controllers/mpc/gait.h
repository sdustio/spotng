#pragma once

#include <memory>
#include <string>

#include "sdrobot/types.h"

namespace sdrobot::ctrl::mpc
{
  class GaitSkd
  {
  public:
    virtual Vector4 GetContactState() = 0;
    virtual Vector4 GetSwingState() = 0;
    virtual const Eigen::VectorXi &GetMpcTable() = 0;
    virtual void SetIterations(int iterationsBetweenMPC, int currentIteration) = 0;
    virtual double GetCurrentStanceTime(double dtMPC, int leg) = 0;
    virtual double GetCurrentSwingTime(double dtMPC, int leg) = 0;
    virtual int GetCurrentGaitPhase() = 0;
  };

  using GaitSkdPtr = std::shared_ptr<GaitSkd>;

  class OffsetDurationGait : public GaitSkd
  {
  public:
    OffsetDurationGait(int nSegment, Eigen::Vector4i offsets, Eigen::Vector4i durations, const std::string &name);
    Vector4 GetContactState() override;
    Vector4 GetSwingState() override;
    const Eigen::VectorXi &GetMpcTable() override;
    void SetIterations(int iterationsPerMPC, int currentIteration) override;
    double GetCurrentStanceTime(double dtMPC, int leg) override;
    double GetCurrentSwingTime(double dtMPC, int leg) override;
    int GetCurrentGaitPhase() override;

  private:
    Eigen::Array4i offsets_; // offset in mpc segments
    Eigen::Array4i durations_; // duration of step in mpc segments
    Eigen::Array4d offsetsd_; // offsets in phase (0 to 1)
    Eigen::Array4d durationsd_; // durations in phase (0 to 1)

    int iteration_;
    int n_iterations_;
    int stance_;
    int swing_;
    double phase_;

    std::string name_;

    Eigen::VectorXi mpc_table_;
  };

} // namespace sdrobot::ctrl::mpc

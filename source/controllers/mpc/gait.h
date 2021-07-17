#pragma once

#include <memory>
#include <string>
#include <vector>

#include "sdrobot/types.h"

namespace sdrobot::ctrl::mpc
{
  class GaitSkd
  {
  public:
    virtual Vector4 getContactState() = 0;
    virtual Vector4 getSwingState() = 0;
    virtual const std::vector<int> &getMpcTable() = 0;
    virtual void setIterations(unsigned iterationsBetweenMPC, unsigned currentIteration) = 0;
    virtual double getCurrentStanceTime(double dtMPC, size_t leg) = 0;
    virtual double getCurrentSwingTime(double dtMPC, size_t leg) = 0;
    virtual unsigned getCurrentGaitPhase() = 0;
  };

  using GaitSkdPtr = std::shared_ptr<GaitSkd>;

  class OffsetDurationGait : public GaitSkd
  {
  public:
    OffsetDurationGait(unsigned nSegment, Eigen::Vector4i offset, Eigen::Vector4i durations, const std::string &name);
    Vector4 getContactState() override;
    Vector4 getSwingState() override;
    const std::vector<int> &getMpcTable() override;
    void setIterations(unsigned iterationsBetweenMPC, unsigned currentIteration) override;
    double getCurrentStanceTime(double dtMPC, size_t leg) override;
    double getCurrentSwingTime(double dtMPC, size_t leg) override;
    unsigned getCurrentGaitPhase() override;

  private:
    std::vector<int> _mpc_table;
  };

} // namespace sdrobot::ctrl::mpc

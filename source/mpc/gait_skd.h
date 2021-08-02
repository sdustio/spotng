#pragma once

#include <memory>
#include <string>

#include "dynamics/types.h"
#include "mpc/mpc.h"

namespace sdrobot::mpc
{
  class GaitSkd
  {
  public:
    using Ptr = std::unique_ptr<GaitSkd>;
    using SharedPtr = std::shared_ptr<GaitSkd>;

    virtual ~GaitSkd() = default;

    virtual bool CalcContactState(SdVector4f &ret) const = 0;
    virtual bool CalcSwingState(SdVector4f &ret) const = 0;
    virtual std::vector<int> const &GetMpcTable() const = 0;
    virtual void SetIterations(int iterationsBetweenMPC, int currentIteration) = 0;
    virtual fpt_t GetCurrentStanceTime(fpt_t dtMPC, int leg) const = 0;
    virtual fpt_t GetCurrentSwingTime(fpt_t dtMPC, int leg) const = 0;
    virtual int GetCurrentGaitPhase() const = 0;
  };

  class OffsetDurationGait : public GaitSkd
  {
  public:
    OffsetDurationGait(int const nSegment, SdVector4i const &offsets, SdVector4i const &durations, std::string const &name);
    bool CalcContactState(SdVector4f &ret) const override;
    bool CalcSwingState(SdVector4f &ret) const override;
    std::vector<int> const &GetMpcTable() const override;
    void SetIterations(int iterationsPerMPC, int currentIteration) override;
    fpt_t GetCurrentStanceTime(fpt_t dtMPC, int leg) const override;
    fpt_t GetCurrentSwingTime(fpt_t dtMPC, int leg) const override;
    int GetCurrentGaitPhase() const override;

  private:
    SdVector4i offsets_;    // offset in mpc segments
    SdVector4i durations_;  // duration of step in mpc segments
    SdMatrix4f offsetsd_;   // offsets in phase (0 to 1)
    SdMatrix4f durationsd_; // durations in phase (0 to 1)

    int iteration_;
    int n_iterations_;
    int stance_;
    int swing_;
    fpt_t phase_;

    std::string name_;

    std::vector<int> mpc_table_;
  };

} // namespace sdrobot::ctrl::mpc

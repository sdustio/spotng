#pragma once

#include "sdrobot/model.h"

namespace sdrobot::model
{
class QuadrupedImpl : public Quadruped
  {
  public:
    bool ComputeFloatBaseModel(fptype g) override;
    FloatBaseModel::SharedPtr const &GetFloatBaseModel() const override;

    bool CalcHipLocation(SdVector3f &ret, int const leg) const override;

  private:
    FloatBaseModel::SharedPtr model_;
  };
}

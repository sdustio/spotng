#pragma once

#include "sdrobot/model.h"

namespace sdrobot::model {
class QuadrupedImpl : public Quadruped {
 public:
  bool ComputeFloatBaseModel(fpt_t g) override;
  FloatBaseModel::SharedPtr const &GetFloatBaseModel() const override;

  bool CalcHipLocation(SdVector3f &ret, int const leg) const override;

 private:
  FloatBaseModel::SharedPtr model_;
};
}  // namespace sdrobot::model

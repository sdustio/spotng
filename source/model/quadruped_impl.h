#pragma once

#include "sdquadx/model.h"

namespace sdquadx::model {
class QuadrupedImpl : public Quadruped {
 public:
  QuadrupedImpl(Options::ConstSharedPtr const &opts) : opts_(opts) {}
  bool ComputeFloatBaseModel() override;
  FloatBaseModel::SharedPtr const &GetFloatBaseModel() const override;

  bool CalcAbadLocation(SdVector3f &ret, int const leg) const override;

 private:
  Options::ConstSharedPtr opts_;
  FloatBaseModel::SharedPtr model_;
};
}  // namespace sdquadx::model

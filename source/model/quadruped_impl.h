#pragma once

#include "model/float_base.h"
#include "spotng/model.h"

namespace spotng::model {
class QuadrupedImpl : public Quadruped {
 public:
  explicit QuadrupedImpl(Options::ConstSharedPtr const &opts) : opts_(opts) { BuildFBModel(); }

  bool UpdateDynamics(estimate::State const &estdata) override;
  DynamicsData const &GetDynamicsData() const override { return data_; }

 private:
  bool BuildFBModel();

  Options::ConstSharedPtr opts_;
  FBModel::SharedPtr fbmodel_;
  DynamicsData data_;
};
}  // namespace spotng::model

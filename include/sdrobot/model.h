#pragma once

#include <memory>

#include "types.h"

namespace sdrobot::model
{
  class FloatBaseModel
  {
  public:
    using Ptr = std::unique_ptr<FloatBaseModel>;
    using SharedPtr = std::shared_ptr<FloatBaseModel>;

    virtual ~FloatBaseModel() = default;
  };
  class Quadruped
  {
  public:
    using Ptr = std::unique_ptr<Quadruped>;
    using SharedPtr = std::shared_ptr<Quadruped>;

    virtual ~Quadruped() = default;

    virtual bool BuildFloatBaseModel(FloatBaseModel::SharedPtr &ret) = 0;
    virtual bool UpdateFloatBaseModel(FloatBaseModel::SharedPtr const &fbm) = 0;
    virtual FloatBaseModel::SharedPtr const &GetFloatBaseModel() const = 0;

    virtual bool CalcHipLocation(SdArray3f &ret, int const leg) const = 0;
  };
} // namespace sdrobot::model

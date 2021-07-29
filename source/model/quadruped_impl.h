#pragma once

#include "sdrobot/model.h"

namespace sdrobot::model
{
class QuadrupedImpl : public Quadruped
  {
  public:
    QuadrupedImpl();
    bool ComputeFloatBaseModel(double g) const override;
    FloatBaseModel::SharedPtr const &GetFloatBaseModel() const override;

    bool CalcHipLocation(SdVector3f &ret, int const leg) const override;

  private:
    SdVector3f abad_location_, abad_rotor_location_;
    SdVector3f hip_location_, hip_rotor_location_;
    SdVector3f knee_location_, knee_rotor_location_;
    SdMatrix6f abad_spatial_inertia_, hip_spatial_inertia_, knee_spatial_inertia_;
    SdMatrix6f abad_rotor_spatial_inertia_, hip_rotor_spatial_inertia_, knee_rotor_spatial_inertia_;
    SdMatrix6f body_spatial_inertia_;

    FloatBaseModel::SharedPtr model_;
  };
}

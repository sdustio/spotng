#pragma once

#include <memory>

#include "sdrobot/types.h"
#include "sdrobot/params.h"

namespace sdrobot::model
{
  struct SDROBOT_EXPORT FloatBaseModelState
  {
    SdVector4f ori;
    SdVector3f pos;
    SdVector6f vel_body; // body coordinates
    std::array<fptype, 12> q;
    std::array<fptype, 12> qd;
  };

  class SDROBOT_EXPORT FloatBaseModel
  {
  public:
    using Ptr = std::unique_ptr<FloatBaseModel>;
    using SharedPtr = std::shared_ptr<FloatBaseModel>;

    using GeneralizedForceType = std::array<fptype, params::model::dim_config>;
    using MassMatrixType = std::array<fptype, params::model::dim_config * params::model::dim_config>;
    using ContactJacobiansType = std::array<fptype, 3 * params::model::dim_config>;

    virtual ~FloatBaseModel() = default;

    virtual bool UpdateState(FloatBaseModelState const &state) = 0;
    virtual FloatBaseModelState const &GetState() const = 0;
    virtual bool UpdateGravity(SdVector3f const &g) = 0;

    virtual bool ComputeGeneralizedMassMatrix() = 0;
    virtual bool ComputeGeneralizedGravityForce() = 0;
    virtual bool ComputeGeneralizedCoriolisForce() = 0;
    virtual bool ComputeContactJacobians() = 0;

    virtual MassMatrixType const &GetMassMatrix() const = 0;
    virtual GeneralizedForceType const &GetGeneralizedGravityForce() const = 0;
    virtual GeneralizedForceType const &GetGeneralizedCoriolisForce() const = 0;
    virtual std::vector<ContactJacobiansType> const &GetContactJacobians() const = 0; //vector of matrix 3 x X
    virtual std::vector<SdVector3f> const &GetContactJacobiansdqd() const = 0;
    virtual std::vector<SdVector3f> const &GetGroundContactPos() const = 0;
    virtual std::vector<SdVector3f> const &GetGroundContactVel() const = 0;
  };

  class SDROBOT_EXPORT Quadruped
  {
  public:
    using Ptr = std::unique_ptr<Quadruped>;
    using SharedPtr = std::shared_ptr<Quadruped>;

    virtual ~Quadruped() = default;

    virtual bool ComputeFloatBaseModel(double g) = 0; // gravity
    virtual FloatBaseModel::SharedPtr const &GetFloatBaseModel() const = 0;

    virtual bool CalcHipLocation(SdVector3f &ret, int const leg) const = 0;
  };
} // namespace sdrobot::model

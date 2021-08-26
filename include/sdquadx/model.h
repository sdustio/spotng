#pragma once

#include <memory>
#include <vector>

#include "sdquadx/consts.h"
#include "sdquadx/types.h"

namespace sdquadx::model {

struct SDQUADX_EXPORT FloatBaseModelState {
  SdVector4f ori;
  SdVector3f pos;
  SdVector6f gvel_robot;  // combine of avel and lvel [avel, lvel]
  std::array<fpt_t, 12> q;
  std::array<fpt_t, 12> qd;
};

using GeneralFTp = std::array<fpt_t, consts::model::kDimConfig>;
using MassMatTp = std::array<fpt_t, consts::model::kDimConfig * consts::model::kDimConfig>;
using ContactJacobTp = std::array<fpt_t, 3 * consts::model::kDimConfig>;

class SDQUADX_EXPORT FloatBaseModel {
 public:
  using Ptr = std::unique_ptr<FloatBaseModel>;
  using SharedPtr = std::shared_ptr<FloatBaseModel>;
  using ConstSharedPtr = std::shared_ptr<FloatBaseModel const>;

  virtual ~FloatBaseModel() = default;

  virtual bool UpdateState(FloatBaseModelState const &state) = 0;
  virtual FloatBaseModelState const &GetState() const = 0;
  virtual bool UpdateGravity(SdVector3f const &g) = 0;

  virtual bool ComputeGeneralizedMassMatrix() = 0;
  virtual bool ComputeGeneralizedGravityForce() = 0;
  virtual bool ComputeGeneralizedCoriolisForce() = 0;
  virtual bool ComputeContactJacobians() = 0;

  virtual MassMatTp const &GetMassMatrix() const = 0;
  virtual GeneralFTp const &GetGeneralizedGravityForce() const = 0;
  virtual GeneralFTp const &GetGeneralizedCoriolisForce() const = 0;
  virtual std::vector<ContactJacobTp> const &GetContactJacobians() const = 0;  // vector of matrix 3 x X
  virtual std::vector<SdVector3f> const &GetContactJacobiansdqd() const = 0;
  virtual std::vector<SdVector3f> const &GetGroundContactPos() const = 0;
  virtual std::vector<SdVector3f> const &GetGroundContactVel() const = 0;
};

class SDQUADX_EXPORT Quadruped {
 public:
  using Ptr = std::unique_ptr<Quadruped>;
  using SharedPtr = std::shared_ptr<Quadruped>;
  using ConstSharedPtr = std::shared_ptr<Quadruped const>;

  virtual ~Quadruped() = default;

  virtual bool ComputeFloatBaseModel(fpt_t g) = 0;  // gravity
  virtual FloatBaseModel::SharedPtr const &GetFloatBaseModel() const = 0;

  virtual bool CalcHipLocation(SdVector3f &ret, int const leg) const = 0;
};
}  // namespace sdquadx::model

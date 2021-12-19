#pragma once

#include <array>
#include <memory>
#include <string>

#include "sdquadx/consts.h"
#include "sdquadx/types.h"

namespace sdquadx::estimate {

struct SDQUADX_EXPORT State {
  SdVector4f contact = {};
  SdVector3f pos = {};
  SdVector3f rpy = {};
  SdVector4f ori = {};
  SdMatrix3f rot_mat = {};
  SdVector3f lvel_robot = {};
  SdVector3f lvel = {};
  SdVector3f avel_robot = {};
  SdVector3f avel = {};
  SdVector3f acc_robot = {};
  SdVector3f acc = {};
  std::array<SdVector3f, consts::model::kNumLeg> q = {};
  std::array<SdVector3f, consts::model::kNumLeg> qd = {};
  std::array<SdVector3f, consts::model::kNumLeg> foot_pos = {};
  std::array<SdVector3f, consts::model::kNumLeg> foot_pos_robot = {};
  std::array<SdVector3f, consts::model::kNumLeg> foot_vel = {};
  std::array<SdVector3f, consts::model::kNumLeg> foot_vel_robot = {};
  bool success = false;
};

class SDQUADX_EXPORT Estimator {
 public:
  using Ptr = std::unique_ptr<Estimator>;
  using SharedPtr = std::shared_ptr<Estimator>;
  using ConstSharedPtr = std::shared_ptr<Estimator const>;

  virtual ~Estimator() = default;
  virtual bool RunOnce(State &ret) = 0;
};

class SDQUADX_EXPORT EstimateCtrl {
 public:
  using Ptr = std::unique_ptr<EstimateCtrl>;
  using SharedPtr = std::shared_ptr<EstimateCtrl>;
  using ConstSharedPtr = std::shared_ptr<EstimateCtrl const>;

  virtual ~EstimateCtrl() = default;

  virtual bool AddEstimator(std::string const &name, Estimator::SharedPtr const &est) = 0;
  virtual Estimator::SharedPtr const &GetEstimator(std::string const &name) const = 0;
  virtual bool RemoveEstimator(std::string const &name) = 0;
  virtual bool RemoveAllEstimators() = 0;

  virtual bool RunOnce() = 0;

  virtual State const &GetEstState() const = 0;
};
}  // namespace sdquadx::estimate

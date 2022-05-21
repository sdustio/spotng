#pragma once

#include <array>
#include <memory>

#include "forax/consts.h"
#include "forax/estimate.h"
#include "forax/options.h"
#include "forax/types.h"

namespace forax::model {

using GeneralFTp = std::array<fpt_t, consts::model::kDimConfig>;
using MassMatTp = std::array<fpt_t, consts::model::kDimConfig * consts::model::kDimConfig>;
using ContactJacobTp = std::array<fpt_t, 3 * consts::model::kDimConfig>;

struct FORAX_EXPORT DynamicsData {
  MassMatTp M;
  GeneralFTp Cc;
  GeneralFTp Cg;
  std::array<fpt_t, consts::model::kNumJoint> q;
  std::array<fpt_t, consts::model::kNumJoint> qd;
  std::array<ContactJacobTp, consts::model::kNumLeg> Jc;
  std::array<SdVector3f, consts::model::kNumLeg> Jcdqd;

  void Zero();
};

class FORAX_EXPORT Quadruped {
 public:
  using Ptr = std::unique_ptr<Quadruped>;
  using SharedPtr = std::shared_ptr<Quadruped>;
  using ConstSharedPtr = std::shared_ptr<Quadruped const>;

  virtual ~Quadruped() = default;

  virtual bool UpdateDynamics(estimate::State const &estdata) = 0;
  virtual DynamicsData const &GetDynamicsData() const = 0;
};
}  // namespace forax::model

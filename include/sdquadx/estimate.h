#pragma once

#include <array>
#include <memory>
#include <string>

#include "sdquadx/consts.h"
#include "sdquadx/types.h"

namespace sdquadx::estimate {

struct SDQUADX_EXPORT State {
  SdVector4f contact = {};                                       // 接触估计
  SdVector3f pos = {};                                           // 位置
  SdVector3f rpy = {};                                           // 欧拉角
  SdVector4f ori = {};                                           // 四元数: w, x, y, z
  SdMatrix3f rot_mat = {};                                       // 旋转矩阵 3x3，从世界坐标到机身坐标
  SdVector3f lvel_robot = {};                                    // 机身坐标速度
  SdVector3f lvel = {};                                          // 世界坐标速度
  SdVector3f avel_robot = {};                                    // 机身坐标角速度
  SdVector3f avel = {};                                          // 世界坐标角速度
  SdVector3f acc_robot = {};                                     // 机身坐标加速度
  SdVector3f acc = {};                                           // 世界坐标加速度
  std::array<SdVector3f, consts::model::kNumLeg> q = {};         // 关节角度
  std::array<SdVector3f, consts::model::kNumLeg> qd = {};        // 关节转速
  std::array<SdVector3f, consts::model::kNumLeg> foot_pos = {};  // 世界坐标系足底位置
  std::array<SdVector3f, consts::model::kNumLeg> foot_pos_robot = {};  // 世界坐标系足底速度
  std::array<SdVector3f, consts::model::kNumLeg> foot_vel = {};        // 机身坐标系足底位置
  std::array<SdVector3f, consts::model::kNumLeg> foot_vel_robot = {};  // 机身坐标系足底位置
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

  // follow the order estimators added(First In First Run)
  virtual bool RunOnce() = 0;

  virtual State const &GetEstState() const = 0;
};
}  //  namespace sdquadx::estimate

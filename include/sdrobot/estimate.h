#pragma once

#include <memory>
#include <cstring>

#include "sdrobot/types.h"

namespace sdrobot::estimate
{

  struct SDROBOT_EXPORT State
  {
    Array4f contact = {};       //接触估计
    Array3f pos = {};           //位置
    Array3f pos_rpy = {};       //欧拉角
    Array4f ori = {};           //四元数: w, x, y, z
    Matrix3f rot_body = {};     //旋转矩阵 3x3
    Array3f vel_body = {};      //机身坐标速度
    Array3f vel_world = {};     //世界坐标速度
    Array3f vel_rpy_body = {};  //机身坐标角速度
    Array3f vel_rpy_world = {}; //世界坐标角速度
    Array3f acc_body = {};      //机身坐标加速度
    Array3f acc_world = {};     //世界坐标加速度
  };

  class SDROBOT_EXPORT Estimator
  {
  public:
    using Ptr = std::unique_ptr<Estimator>;
    using SharedPtr = std::shared_ptr<Estimator>;

    virtual ~Estimator() = default;
    virtual bool RunOnce(State &ret) = 0;
  };

  class SDROBOT_EXPORT EstimateCtrl
  {
  public:
    using Ptr = std::unique_ptr<EstimateCtrl>;
    using SharedPtr = std::shared_ptr<EstimateCtrl>;

    virtual ~EstimateCtrl() = default;

    virtual bool AddEstimator(std::string const &name, Estimator::SharedPtr const &est) = 0;
    virtual Estimator::SharedPtr const &GetEstimator(std::string const &name) = 0;
    virtual bool RemoveEstimator(std::string const &name) = 0;
    virtual bool RemoveAllEstimators() = 0;

    virtual bool RunOnce() = 0;

    virtual State const &GetEstState() const = 0;
  };
}
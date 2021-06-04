#pragma once

#include "sd/robot/model.h"
#include "sd/robot/interface.h"

namespace sd::est
{
  struct StateEstimate
  {
    Vec4<double> contact_estimate;    //接触估计
    Vector3d position;                //位置
    Vector3d rpy;                     //欧拉角
    Quat orientation;                 //四元数
    RotMat rot_body;                  //旋转矩阵
    Vector3d v_body, v_world;         //速度
    Vector3d omega_body, omega_world; //角速度
    Vector3d a_body, a_world;         //加速度，世界坐标下加速度
  };

  struct EstimatorDataSource
  {
    const robot::leg::Data *leg_data;
    const robot::IMUData *imu_data;
    const Vec4<double> *contact_phase;
  }

  /*!
 * All Estimators should inherit from this class
 *所有的估计器都应该继承这个类
 */
  class GenericEstimator
  {
  public:
    virtual void run() = 0;
    virtual void setup() = 0;

    void setData(EstimatorDataSource data) { _stateEstimatorData = data; }

    virtual ~GenericEstimator() = default;
    EstimatorDataSource _stateEstimatorData;
  };

}

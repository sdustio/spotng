#pragma once

#include "sd/robot/model.h"
#include "sd/robot/interface.h"
#include "sd/dynamics/state.h"

namespace sd::est
{
  template <typename T>
  class StateEstimate
  {
  private:
    Vec12<T> state_;
  };

  template <typename T>
  struct EstimatorDataSource
  {
    const robot::leg::Data<T> *leg_data;
    const robot::IMUData *imu_data;
    const Vec4<T> *contact_phase;
  }

}

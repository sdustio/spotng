#pragma once

#include "sd/Robot/Model.hpp"

#include "Dynamics/Spatial.hpp"

namespace sd::robot::model{

  template <typename T>
  class Quadruped {
    public:
      explicit Quadruped(){};
      const Vec3<T>& GetAbadLocation() const {return mAbadLocation;}
      const Vec3<T>& GetAbadRotorLocation() const {return mAbadRotorLocation;}
      const Vec3<T>& GetHipLocation() const {return mHipLocation;}
      const Vec3<T>& GetHipRotorLocation() const {return mHipRotorLocation;}
      const Vec3<T>& GetKneeLocation() const {return mKneeLocation;}
      const Vec3<T>& GetKneeRotorLocation() const {return mKneeRotorLocation;}
    private:
      Vec3<T> mAbadLocation, mAbadRotorLocation;
      Vec3<T> mHipLocation, mHipRotorLocation;
      Vec3<T> mKneeLocation, mKneeRotorLocation;
  };

  template class Quadruped<double>; //for compile check
} // namespace sd::robot::model

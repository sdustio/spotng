#pragma once

#include "sd/Robot/Model.hpp"

namespace sd::robot::model{

  template <typename T>
  class Quadruped {
    public:
      explicit Quadruped(){};
      const Vec3<T>* AbadLocation() const {return &mAbadLocation;}
      const Vec3<T>* AbadRotorLocation() const {return &mAbadRotorLocation;}
      const Vec3<T>* HipLocation() const {return &mHipLocation;}
      const Vec3<T>* HipRotorLocation() const {return &mHipRotorLocation;}
      const Vec3<T>* KneeLocation() const {return &mKneeLocation;}
      const Vec3<T>* KneeRotorLocation() const {return &mKneeRotorLocation;}
    private:
      Vec3<T> mAbadLocation, mAbadRotorLocation;
      Vec3<T> mHipLocation, mHipRotorLocation;
      Vec3<T> mKneeLocation, mKneeRotorLocation;
  };
} // namespace sd::robot::model

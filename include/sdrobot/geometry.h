#pragma once

#include "sdrobot/types.h"

namespace sdrobot::geometry
{

  class SDROBOT_EXPORT Quaternions
  {
  public:
    Quaternions() = default;
    explicit Quaternions(Array4f const &quat);
    Quaternions(double w, double x, double y, double z);
    double w();
    double x();
    double y();
    double z();
    bool UpdateData(Array4f const &quat);
    bool UpdateData(double w, double x, double y, double z);
    bool GetData(Array4f &data) const;

  private:
    Array4f data_;
  };
}

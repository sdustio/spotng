#pragma once

#include "sdrobot/export.h"
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
    void SetData(Array4f const &quat);
    void SetData(double w, double x, double y, double z);
    Array4f const &Data() const;

  private:
    Array4f data_;
  };
}

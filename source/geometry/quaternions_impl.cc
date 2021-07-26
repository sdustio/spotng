#include "sdrobot/geometry.h"

namespace sdrobot::geometry
{
  Quaternions::Quaternions(Array4f const &quat) : data_(quat) {}
  Quaternions::Quaternions(double w, double x, double y, double z) : data_{w, x, y, z} {}

  double Quaternions::w() { return data_[0]; }
  double Quaternions::x() { return data_[1]; }
  double Quaternions::y() { return data_[2]; }
  double Quaternions::z() { return data_[3]; }
  void Quaternions::SetData(Array4f const &quat)
  {
    data_ = quat;
  }
  void Quaternions::SetData(double w, double x, double y, double z) {
    data_ = {w, x, y, z};
  }
  Array4f const &Quaternions::Data() const
  {
    return data_;
  }
}

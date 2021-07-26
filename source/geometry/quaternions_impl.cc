#include "sdrobot/geometry.h"

namespace sdrobot::geometry
{
  Quaternions::Quaternions(Array4f const &quat) : data_(quat) {}
  Quaternions::Quaternions(double w, double x, double y, double z) : data_{w, x, y, z} {}

  double Quaternions::w() { return data_[0]; }
  double Quaternions::x() { return data_[1]; }
  double Quaternions::y() { return data_[2]; }
  double Quaternions::z() { return data_[3]; }
  bool Quaternions::UpdateData(Array4f const &quat)
  {
    data_ = quat;
    return true;
  }
  bool Quaternions::UpdateData(double w, double x, double y, double z) {
    data_ = {w, x, y, z};
    return true;
  }
  bool Quaternions::GetData(Array4f &data) const
  {
    data = data_;
    return true;
  }
}

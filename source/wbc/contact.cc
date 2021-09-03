#include "wbc/contact.h"

#include "externlib/eigen.h"

namespace sdquadx::wbc {

Contact::Contact(model::FloatBaseModel::ConstSharedPtr const &model, int contact_idx)
    : robot_sys_(model), contact_idx_(contact_idx) {
  fpt_t mu = 0.4;

  Eigen::Map<Eigen::Matrix<fpt_t, 6, 3>> Uf(Uf_.data());

  Uf(0, 2) = 1.;

  Uf(1, 0) = 1.;
  Uf(1, 2) = mu;
  Uf(2, 0) = -1.;
  Uf(2, 2) = mu;

  Uf(3, 1) = 1.;
  Uf(3, 2) = mu;
  Uf(4, 1) = -1.;
  Uf(4, 2) = mu;

  // Upper bound of normal force
  Uf(5, 2) = -1.;
}

bool Contact::_UpdateJc() {
  Jc_ = robot_sys_->GetContactJacobians()[contact_idx_];
  return true;
}

bool Contact::_UpdateJcDotQdot() {
  JcDotQdot_ = robot_sys_->GetContactJacobiansdqd()[contact_idx_];
  // pretty_print(JcDotQdot_, std::cout, "JcDotQdot");
  return true;
}

bool Contact::_UpdateUf() { return true; }

bool Contact::_UpdateInequalityVector() {
  ieq_vec_.fill(0.);
  ieq_vec_[5] = -_max_Fz;
  return true;
}

bool Contact::UpdateContact() {
  _UpdateJc();
  _UpdateJcDotQdot();
  _UpdateUf();
  _UpdateInequalityVector();
  return true;
}

bool Contact::UpdateRFdes(SdVector3f const &Fr_des) {
  Fr_des_ = Fr_des;
  return true;
}
}  // namespace sdquadx::wbc

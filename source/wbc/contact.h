#pragma once

#include <memory>

#include "sdquadx/model.h"

namespace sdquadx::wbc {
class Contact {
 public:
  using Ptr = std::unique_ptr<Contact>;
  using SharedPtr = std::shared_ptr<Contact>;
  using ConstSharedPtr = std::shared_ptr<Contact const>;

  Contact(model::FloatBaseModel::ConstSharedPtr const &model, int contact_pt);

  int GetDim() const { return 3; }
  int GetDimRFConstraint() const { return 6; }

  SdVector3f const &GetRFdes() const { return Fr_des_; }
  bool UpdateRFdes(SdVector3f const &Fr_des);

  bool UpdateContact();

  model::ContactJacobTp const &GetContactJacobian() const { return Jc_; }
  SdVector3f const &GetJcDotQdot() const { return JcDotQdot_; }
  std::array<fpt_t, 6 * 3> const &GetRFConstraintMtx() const { return Uf_; }
  SdVector6f const &GetRFConstraintVec() const { return ieq_vec_; }

 private:
  bool _UpdateJc();
  bool _UpdateJcDotQdot();
  bool _UpdateUf();
  bool _UpdateInequalityVector();

  model::FloatBaseModel::ConstSharedPtr robot_sys_;

  std::array<fpt_t, 6 * 3> Uf_ = {};
  SdVector6f ieq_vec_;

  SdVector3f Fr_des_;

  std::array<fpt_t, 3 *consts::model::kDimConfig> Jc_ = {};
  SdVector3f JcDotQdot_ = {};

  fpt_t _max_Fz = 1500.;
  int contact_pt_;
};

}  // namespace sdquadx::wbc

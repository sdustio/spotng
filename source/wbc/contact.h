#pragma once

#include "sdrobot/model.h"
#include "eigen.h"

namespace sdrobot::wbc
{
  class Contact
  {
  public:
    using Ptr = std::unique_ptr<Contact>;
    using SharedPtr = std::shared_ptr<Contact>;

    using Jc_t = Eigen::Matrix<fpt_t, 3, params::model::dim_config>;

    Contact(model::FloatBaseModel::SharedPtr const &model, int contact_pt);

    int GetDim() const { return 3; }
    int GetDimRFConstraint() const { return 6; }

    SdVector3f const &GetRFdes() { return Fr_des_; }
    bool UpdateRFdes(SdVector3f const &Fr_des);

    bool UpdateContact();

    std::array<fpt_t, 3 * params::model::dim_config> const &GetContactJacobian() { return Jc_; }
    SdVector3f const &GetJcDotQdot() { return JcDotQdot_; }
    std::array<fpt_t, 6 * 3> const &GetRFConstraintMtx() { return Uf_; }
    SdVector6f const &GetRFConstraintVec() { return ieq_vec_; }

  private:
    bool _UpdateJc();
    bool _UpdateJcDotQdot();
    bool _UpdateUf();
    bool _UpdateInequalityVector();

    model::FloatBaseModel::SharedPtr const robot_sys_;

    std::array<fpt_t, 6 * 3> Uf_ = {};
    SdVector6f ieq_vec_;

    SdVector3f Fr_des_;

    std::array<fpt_t, 3 *params::model::dim_config> Jc_ = {};
    SdVector3f JcDotQdot_ = {};

    fpt_t _max_Fz = 1500.;
    int _contact_pt;
  };

} // namespace sdrobot::wbc

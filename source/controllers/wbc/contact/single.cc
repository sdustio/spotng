#include "controllers/wbc/contact/single.h"

namespace sdrobot::ctrl::wbc
{

    ContactSingle::ContactSingle(
        const dynamics::FBModelPtr &model, size_t contact_pt) : Contact(3), robot_sys_(model), _contact_pt(contact_pt)
    {
        Jc_ = MatrixX(dim_contact_, robot::ModelAttrs::dim_config);
        JcDotQdot_ = VectorX::Zero(dim_contact_);
        Uf_ = MatrixX::Zero(_dim_U, dim_contact_);

        double mu(0.4);

        Uf_(0, 2) = 1.;

        Uf_(1, 0) = 1.;
        Uf_(1, 2) = mu;
        Uf_(2, 0) = -1.;
        Uf_(2, 2) = mu;

        Uf_(3, 1) = 1.;
        Uf_(3, 2) = mu;
        Uf_(4, 1) = -1.;
        Uf_(4, 2) = mu;

        // Upper bound of normal force
        Uf_(5, 2) = -1.;
    }

    bool ContactSingle::_UpdateJc()
    {
        Jc_ = robot_sys_->GetJc()[_contact_pt];
        return true;
    }

    bool ContactSingle::_UpdateJcDotQdot()
    {
        JcDotQdot_ = robot_sys_->GetJcdqd()[_contact_pt];
        // pretty_print(JcDotQdot_, std::cout, "JcDotQdot");
        return true;
    }

    bool ContactSingle::_UpdateUf()
    {
        return true;
    }

    bool ContactSingle::_UpdateInequalityVector()
    {
        ieq_vec_ = VectorX::Zero(_dim_U);
        ieq_vec_[5] = -_max_Fz;
        return true;
    }

}

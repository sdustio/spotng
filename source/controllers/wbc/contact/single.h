#pragma once

#include "sdrobot/controllers/wbc.h"

namespace sdrobot::ctrl::wbc
{
  class ContactSingle : public Contact
  {
  public:
    ContactSingle(const dynamics::FBModelPtr& model, int contact_pt);

  private:
    bool _UpdateJc() override;
    bool _UpdateJcDotQdot() override;
    bool _UpdateUf() override;
    bool _UpdateInequalityVector() override;

    const dynamics::FBModelPtr robot_sys_;
    double _max_Fz = 1500.;
    int _contact_pt;
    int _dim_U = 6;
  };

}

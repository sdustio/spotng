#pragma once

#include "sdrobot/controllers/wbc.h"

namespace sdrobot::ctrl::wbc
{
  class ContactSingle : public Contact
  {
  public:
    ContactSingle(const dynamics::FBModelPtr& model, size_t contact_pt);

  private:
    bool _UpdateJc() override;
    bool _UpdateJcDotQdot() override;
    bool _UpdateUf() override;
    bool _UpdateInequalityVector() override;

    const dynamics::FBModelPtr robot_sys_;
    size_t _contact_pt;
  };

}
